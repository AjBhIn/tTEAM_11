#!/usr/bin/env python3
"""
========================================================================================
HIGH-PERFORMANCE EVASION CALCULATOR WITH DEEP STUCK RECOVERY LAYER
========================================================================================
Layer 0 - Deep Stuck Recovery Architecture:
1. TRIGGER:
   - Activates immediately when robot position cost >= MAX_SAFE_COST (85).
2. DOWNHILL GRADIENT SEARCH:
   - Probes 36 radial directions to find the steepest downhill cost descent into free space.
3. HEAD-ON ROTATION LOCK:
   - Probes 0.35m directly ahead of current robot yaw. If an obstacle is detected in front,
     it penalizes all forward angles (+-60 deg) by -500, forcing the robot to rotate away.
4. FREE-SPACE OVERSHOOT PROJECTION:
   - Instead of dropping a goal 0.45m away (which stays trapped in Nav2 inflation), it 
     projects the goal 1.2m along the escape heading out into confirmed free space (cost < 30).
5. SMOOTHING BYPASS:
   - Completely bypasses alpha smoothing so the goal snaps immediately into free space,
     preventing the old stuck coordinate from dragging the target back into the wall.
========================================================================================
"""

import math
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np

# --- GLOBAL TUNING VARIABLES ---
LOOP_RATE_SEC = 0.033           # 30Hz Control Loop Rate

# Costmap & Map Safety Thresholds (Normalized 0-100 scale)
COSTMAP_MARGIN_CELLS = 6        # Cells to clear from the grid edges[cite: 2]
MAX_SAFE_COST = 85              # Stuck Zone Threshold: cost >= 85 triggers stuck recovery[cite: 2]
MAX_EMERGENCY_COST = 95         # Strict ceiling before solid obstacle boundaries (100)[cite: 2]

# Stuck Recovery Projection Parameters
STUCK_ESCAPE_DIST = 1.2         # Projects goal 1.2m into open space to pull Nav2 out of inflation[cite: 1, 2]
HEAD_ON_PROBE_DIST = 0.35       # Forward distance to check if facing an obstacle directly[cite: 2]

# Proximity & Update Triggers
CLOSE_THREAT_DIST = 1.2         # Proximity threshold where escape speed takes priority[cite: 2]
TARGET_REPOSITION_DIST = 2.0    # Distance behind enemy matching StateMachine requirements[cite: 2, 5]
MIN_UPDATE_DIST = 0.02          # Minimum movement required to broadcast a new TF[cite: 2]

# Timeouts & Log Frequency
TF_TIMEOUT_SEC = 2.5            # Dead-man timeout for Wi-Fi latency[cite: 2]
LOG_THROTTLE_SEC = 1.0          # Console warning throttle interval[cite: 2]

# Dynamic Scoring Weights
WEIGHT_DIST = 12.0              # Base weight for distance from enemy[cite: 2]
WEIGHT_SPEED = 2.0              # Weight for candidate reach (radius)[cite: 2]
WEIGHT_TURN = 3.0               # Penalty weight for turn angle from our heading[cite: 2]
WEIGHT_COST = 0.5               # Penalty weight for costmap elevation[cite: 2]
WEIGHT_REAR_FLANK = 28.0        # Reward for circling behind enemy heading (120-180 deg)[cite: 2, 5]

class EvasionCalculator(Node):
    def __init__(self):
        super().__init__('evasion_calculator')

        raw_ns = self.get_namespace().strip('/')
        default_robot_ns = raw_ns if raw_ns else 'our_bot'

        self.declare_parameter('chaser', default_robot_ns)
        self.declare_parameter('enemy', 'enemy_bot')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', '')
        self.declare_parameter('enemy_frame', '')
        self.declare_parameter('target_frame', 'dynamic_nav_target')

        self.robot_ns = self.get_parameter('chaser').value.strip('/')
        self.enemy_ns = self.get_parameter('enemy').value.strip('/')

        self.map_frame = self.get_parameter('map_frame').value
        robot_p = self.get_parameter('robot_frame').value
        self.robot_frame = robot_p if robot_p else (f'{self.robot_ns}/base_link' if self.robot_ns else 'base_link')

        enemy_p = self.get_parameter('enemy_frame').value
        self.enemy_frame = enemy_p if enemy_p else (f'{self.enemy_ns}/base_link' if self.enemy_ns else 'base_link')
        self.target_frame = self.get_parameter('target_frame').value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.current_state = 'CHASING'
        
        state_topic = f'/{self.robot_ns}/robot_state' if self.robot_ns else '/robot_state'
        costmap_topic = f'/{self.robot_ns}/global_costmap/costmap' if self.robot_ns else '/global_costmap/costmap'

        self.state_sub = self.create_subscription(String, state_topic, self.state_cb, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, costmap_topic, self.costmap_callback, 10)

        # 4 radii * 36 angles = 144 candidate points precomputed
        self.sample_radii = np.array([0.6, 1.0, 1.5, 2.0])
        angles = np.deg2rad(np.arange(0, 360, 10))
        r_grid, a_grid = np.meshgrid(self.sample_radii, angles)
        
        self.cand_dx = (r_grid * np.cos(a_grid)).flatten()
        self.cand_dy = (r_grid * np.sin(a_grid)).flatten()
        self.cand_r = r_grid.flatten()

        self.map_data = None
        self.map_res = 0.05
        self.map_width = self.map_height = 0
        self.map_origin_x = self.map_origin_y = 0.0 
        
        self.last_target_x, self.last_target_y = None, None
        self.active_target_x, self.active_target_y = None, None
        
        self.slide_direction_bias = None 
        self.last_chosen_angle = None 

        self.last_log_time = 0.0
        self.last_tf_time = 0.0 

        self.timer = self.create_timer(LOOP_RATE_SEC, self.control_loop)
        self.get_logger().info("[EVASION CALCULATOR] Initialized with Layer 0 Deep Stuck Recovery.")

    def state_cb(self, msg):
        self.current_state = msg.data

    @staticmethod
    def get_yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def costmap_callback(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data, dtype=np.int16)
        self.map_res = msg.info.resolution
        self.map_width, self.map_height = msg.info.width, msg.info.height
        self.map_origin_x, self.map_origin_y = msg.info.origin.position.x, msg.info.origin.position.y

    def get_cell_cost(self, x: float, y: float) -> int:
        if self.map_data is None: return 100
        raw_col = int((x - self.map_origin_x) / self.map_res)
        raw_row = int((y - self.map_origin_y) / self.map_res)

        if raw_col < COSTMAP_MARGIN_CELLS or raw_col >= (self.map_width - COSTMAP_MARGIN_CELLS) or \
           raw_row < COSTMAP_MARGIN_CELLS or raw_row >= (self.map_height - COSTMAP_MARGIN_CELLS):
            return 100

        col = max(0, min(raw_col, self.map_width - 1))
        row = max(0, min(raw_row, self.map_height - 1))
        cost = self.map_data[row * self.map_width + col]
        return 100 if cost == -1 else cost

    def get_cell_costs_vectorized(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        if self.map_data is None: return np.full(xs.shape, 100)
        
        raw_cols = ((xs - self.map_origin_x) / self.map_res).astype(np.int32)
        raw_rows = ((ys - self.map_origin_y) / self.map_res).astype(np.int32)
        
        invalid = (raw_cols < COSTMAP_MARGIN_CELLS) | (raw_cols >= (self.map_width - COSTMAP_MARGIN_CELLS)) | \
                  (raw_rows < COSTMAP_MARGIN_CELLS) | (raw_rows >= (self.map_height - COSTMAP_MARGIN_CELLS))
                  
        cols = np.clip(raw_cols, 0, self.map_width - 1)
        rows = np.clip(raw_rows, 0, self.map_height - 1)
        
        costs = self.map_data[rows * self.map_width + cols]
        costs = np.where((costs == -1) | invalid, 100, costs)
        return costs

    # =========================================================================
    # LAYER 0: DEEP STUCK RECOVERY ENGINE
    # =========================================================================
    def get_stuck_recovery_target(self, our_x: float, our_y: float, our_yaw: float, 
                                  enemy_x: float, enemy_y: float, current_cost: int):
        """
        1. Finds the steepest downhill cost gradient out of the current high-cost cell.
        2. Probes directly ahead; if facing an obstacle, penalizes forward cone to force in-place rotation.
        3. Projects the goal 1.2m along the escape heading into confirmed free space (cost < 30)
           so Nav2 sees a clear path and commits to full motor drive instead of braking in inflation.
        """
        # Step A: Sample 36 radial directions around the robot at 0.5m
        probe_angles = np.linspace(0, 2 * math.pi, 36, endpoint=False)
        probe_x = our_x + 0.5 * np.cos(probe_angles)
        probe_y = our_y + 0.5 * np.sin(probe_angles)
        probe_costs = self.get_cell_costs_vectorized(probe_x, probe_y)

        # Step B: Check if facing an obstacle directly head-on
        front_probe_x = our_x + HEAD_ON_PROBE_DIST * math.cos(our_yaw)
        front_probe_y = our_y + HEAD_ON_PROBE_DIST * math.sin(our_yaw)
        front_cost = self.get_cell_cost(front_probe_x, front_probe_y)
        facing_obstacle = (front_cost >= current_cost) or (front_cost >= MAX_SAFE_COST)

        # Step C: Score directions by steepest cost drop (downhill gradient)
        cost_drop = current_cost - probe_costs
        scores = cost_drop * 10.0

        # Secondary weight: prefer directions away from enemy
        dist_to_enemy = np.hypot(probe_x - enemy_x, probe_y - enemy_y)
        scores += dist_to_enemy * 2.0

        # If facing obstacle head-on, heavily penalize forward angles to force a rotation
        turn_diff = np.abs(np.arctan2(np.sin(probe_angles - our_yaw), np.cos(probe_angles - our_yaw)))
        if facing_obstacle:
            scores = np.where(turn_diff < np.deg2rad(60.0), scores - 500.0, scores)
        else:
            scores -= turn_diff * 1.5

        # Discard angles that point straight into solid lethal barriers (100)
        valid_angles = probe_costs < MAX_EMERGENCY_COST
        if not np.any(valid_angles):
            return self.execute_emergency_escape(our_x, our_y, enemy_x, enemy_y)

        best_idx = np.argmax(np.where(valid_angles, scores, -1e9))
        best_escape_yaw = float(probe_angles[best_idx])

        # Step D: Project the goal 1.2m along the escape heading out into free space
        # March outward along best_escape_yaw until we find a clear cell (cost < 30) or reach 1.2m
        escape_dist = STUCK_ESCAPE_DIST
        for d in (1.2, 1.0, 0.8, 0.6):
            tx = our_x + d * math.cos(best_escape_yaw)
            ty = our_y + d * math.sin(best_escape_yaw)
            cost_at_d = self.get_cell_cost(tx, ty)
            if cost_at_d <= 35:
                escape_dist = d
                break

        rec_x = our_x + escape_dist * math.cos(best_escape_yaw)
        rec_y = our_y + escape_dist * math.sin(best_escape_yaw)

        self.get_logger().warn(
            f"[STUCK RECOVERY] Exit vector chosen! Yaw: {math.degrees(best_escape_yaw):.1f}° | "
            f"Facing obstacle: {facing_obstacle} | Target placed {escape_dist:.2f}m away in open space."
        )

        return rec_x, rec_y, best_escape_yaw

    def is_path_clear(self, start_x: float, start_y: float, end_x: float, end_y: float, max_cost: int) -> bool:
        if self.map_data is None: return False
        dist = math.hypot(end_x - start_x, end_y - start_y)
        if dist < 0.05: return self.get_cell_cost(end_x, end_y) <= max_cost

        step_size = 0.10 if dist <= 0.8 else 0.05
        steps = max(2, int(dist / step_size))

        t = np.linspace(0.0, 1.0, steps)
        px = start_x + t * (end_x - start_x)
        py = start_y + t * (end_y - start_y)

        raw_cols = ((px - self.map_origin_x) / self.map_res).astype(np.int32)
        raw_rows = ((py - self.map_origin_y) / self.map_res).astype(np.int32)

        if np.any(raw_cols < COSTMAP_MARGIN_CELLS) or np.any(raw_cols >= (self.map_width - COSTMAP_MARGIN_CELLS)) or \
           np.any(raw_rows < COSTMAP_MARGIN_CELLS) or np.any(raw_rows >= (self.map_height - COSTMAP_MARGIN_CELLS)):
            return False

        cols = np.clip(raw_cols, 0, self.map_width - 1)
        rows = np.clip(raw_rows, 0, self.map_height - 1)
        indices = rows * self.map_width + cols
        costs = self.map_data[indices]

        return not (np.any(costs > max_cost) or np.any(costs == -1))

    def check_path_feasibility(self, start_x, start_y, end_x, end_y, max_cost, enable_dogleg=True):
        if self.is_path_clear(start_x, start_y, end_x, end_y, max_cost):
            return True, 'STRAIGHT'
        
        if not enable_dogleg:
            return False, 'NONE'
            
        direct_angle = math.atan2(end_y - start_y, end_x - start_x)
        mid_dist = math.hypot(end_x - start_x, end_y - start_y) / 2.0
        
        test_angles = np.deg2rad([35, -35, 70, -70])
        mid_x = start_x + (mid_dist * np.cos(direct_angle + test_angles))
        mid_y = start_y + (mid_dist * np.sin(direct_angle + test_angles))
        
        for i in range(len(test_angles)):
            if self.is_path_clear(start_x, start_y, mid_x[i], mid_y[i], max_cost):
                if self.is_path_clear(mid_x[i], mid_y[i], end_x, end_y, max_cost):
                    return True, 'DOGLEG'
        return False, 'NONE'

    @staticmethod
    def get_adaptive_weights(current_cost, dist_to_enemy):
        w_dist, w_speed, w_turn, w_cost = WEIGHT_DIST, WEIGHT_SPEED, WEIGHT_TURN, WEIGHT_COST
        
        if current_cost > 20: 
            w_cost, w_turn = 1.5, 0.5
        if dist_to_enemy < CLOSE_THREAT_DIST: 
            w_dist, w_speed = 22.0, 4.0
            
        return {'w_dist': w_dist, 'w_speed': w_speed, 'w_turn': w_turn, 'w_cost': w_cost}

    def evaluate_and_select_best_escape(self, our_x, our_y, our_yaw, enemy_x, enemy_y, enemy_yaw):
        c_x = our_x + self.cand_dx
        c_y = our_y + self.cand_dy
        c_r = self.cand_r
        
        c_cost = self.get_cell_costs_vectorized(c_x, c_y)
        current_cost = self.get_cell_cost(our_x, our_y)
        
        effective_max_cost = min(MAX_EMERGENCY_COST, max(MAX_SAFE_COST, current_cost))
        
        valid_mask = c_cost <= effective_max_cost
        if not np.any(valid_mask): 
            return None, -1000.0
            
        c_x, c_y, c_r, c_cost = c_x[valid_mask], c_y[valid_mask], c_r[valid_mask], c_cost[valid_mask]
        cand_angle = np.arctan2(c_y - our_y, c_x - our_x)

        weights = self.get_adaptive_weights(current_cost, math.hypot(our_x - enemy_x, our_y - enemy_y))
        dist_to_enemy = np.hypot(c_x - enemy_x, c_y - enemy_y)
        
        scores = (dist_to_enemy * weights['w_dist']) + (c_r * weights['w_speed'])

        away_dx = our_x - enemy_x
        away_dy = our_y - enemy_y
        away_angle = math.atan2(away_dy, away_dx)
        
        direct_away_blocked = self.get_cell_cost(our_x + math.cos(away_angle), our_y + math.sin(away_angle)) > 20
        tangent_left = away_angle + (math.pi / 2.0)
        tangent_right = away_angle - (math.pi / 2.0)

        if direct_away_blocked:
            diff_left = np.abs(np.arctan2(np.sin(cand_angle - tangent_left), np.cos(cand_angle - tangent_left)))
            diff_right = np.abs(np.arctan2(np.sin(cand_angle - tangent_right), np.cos(cand_angle - tangent_right)))
            scores += (np.pi - np.minimum(diff_left, diff_right)) * 12.0

            if self.slide_direction_bias == 'LEFT':
                scores = np.where(diff_left < diff_right, scores + 20.0, scores)
            elif self.slide_direction_bias == 'RIGHT':
                scores = np.where(diff_right < diff_left, scores + 20.0, scores)
        else:
            angle_diff_away = np.abs(np.arctan2(np.sin(cand_angle - away_angle), np.cos(cand_angle - away_angle)))
            scores += (np.pi - angle_diff_away) * 10.0

        angle_from_enemy_to_cand = np.arctan2(c_y - enemy_y, c_x - enemy_x)
        front_diff = np.abs(np.arctan2(np.sin(angle_from_enemy_to_cand - enemy_yaw), np.cos(angle_from_enemy_to_cand - enemy_yaw)))

        # Continuous frontal penalty
        in_front_mask = front_diff < (np.pi / 2.0)
        continuous_front_penalty = np.where(in_front_mask, (np.cos(front_diff) ** 2) * 45.0, 0.0)
        scores -= continuous_front_penalty

        # Continuous rear-flank reward
        rear_flank_bonus = np.where(front_diff >= (np.pi / 2.0), (-np.cos(front_diff)) * WEIGHT_REAR_FLANK, 0.0)
        scores += rear_flank_bonus

        # Tactical alignment criteria for clearing StateMachine EVADING
        meets_state_criteria = (front_diff >= np.deg2rad(120.0)) & (dist_to_enemy >= (TARGET_REPOSITION_DIST - 0.2))
        scores += np.where(meets_state_criteria, 40.0, 0.0)

        if self.last_chosen_angle is not None:
            angle_diff_from_last = np.abs(np.arctan2(np.sin(cand_angle - self.last_chosen_angle), np.cos(cand_angle - self.last_chosen_angle)))
            scores -= (angle_diff_from_last * 25.0)

        cost_increase = c_cost - current_cost
        scores = np.where(cost_increase > 0, scores - (cost_increase * 3.5), scores)
        scores = np.where(c_cost > 20, scores - (c_cost * 2.5), scores)
        
        if current_cost > 10:
            cost_decrease = current_cost - c_cost
            scores = np.where(cost_decrease > 0, scores + (cost_decrease * 4.0), scores)
        scores = np.where(c_cost == 0, scores + 35.0, scores)

        turn_penalty = np.abs(np.arctan2(np.sin(cand_angle - our_yaw), np.cos(cand_angle - our_yaw)))
        scores -= turn_penalty * weights['w_turn']
        scores -= c_cost * weights['w_cost']

        if self.active_target_x is not None and self.active_target_y is not None:
            active_dists = np.hypot(c_x - self.active_target_x, c_y - self.active_target_y)
            scores = np.where(active_dists < 0.7, scores + 30.0, scores)

        top_k = min(10, len(scores))
        top_indices = np.argsort(scores)[-top_k:][::-1]

        best_point = None
        best_score = -float('inf')

        for rank, idx in enumerate(top_indices):
            tx, ty, t_angle, t_base_score = c_x[idx], c_y[idx], cand_angle[idx], scores[idx]
            
            allow_dogleg = (rank < 4)
            is_clear, path_type = self.check_path_feasibility(our_x, our_y, tx, ty, effective_max_cost, enable_dogleg=allow_dogleg)
            
            if is_clear:
                final_score = t_base_score + (15.0 if path_type == 'STRAIGHT' else 0.0)
                if final_score > best_score:
                    best_score = final_score
                    best_point = (tx, ty, t_angle)

        if best_point is not None:
            self.active_target_x, self.active_target_y = best_point[0], best_point[1]
            self.last_chosen_angle = best_point[2]
            dl = abs(math.atan2(math.sin(best_point[2] - tangent_left), math.cos(best_point[2] - tangent_left)))
            dr = abs(math.atan2(math.sin(best_point[2] - tangent_right), math.cos(best_point[2] - tangent_right)))
            self.slide_direction_bias = 'LEFT' if dl < dr else 'RIGHT'
            
        return best_point, best_score

    def execute_emergency_escape(self, global_x: float, global_y: float, enemy_x: float, enemy_y: float):
        em_radii = np.array([0.5, 0.9, 1.2])
        angles = np.deg2rad(np.arange(0, 360, 15)) 
        
        r_grid, a_grid = np.meshgrid(em_radii, angles)
        cand_x = global_x + (r_grid * np.cos(a_grid)).flatten()
        cand_y = global_y + (r_grid * np.sin(a_grid)).flatten()
        
        costs = self.get_cell_costs_vectorized(cand_x, cand_y)
        valid_mask = costs < MAX_EMERGENCY_COST

        if np.any(valid_mask):
            valid_x, valid_y, valid_costs = cand_x[valid_mask], cand_y[valid_mask], costs[valid_mask]
            dist_to_enemy = np.hypot(valid_x - enemy_x, valid_y - enemy_y)
            scores = (dist_to_enemy * 25.0) - (valid_costs * 1.5)
            
            best_idx = np.argmax(scores)
            best_x, best_y = valid_x[best_idx], valid_y[best_idx]
            best_yaw = math.atan2(best_y - global_y, best_x - global_x)
            return best_x, best_y, best_yaw

        dx, dy = global_x - enemy_x, global_y - enemy_y
        escape_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 0.01 else 0.0
        return global_x, global_y, escape_yaw

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.map_data is None: 
            return

        if self.current_state != 'EVADING':
            self.last_target_x, self.last_target_y = None, None
            self.active_target_x, self.active_target_y = None, None
            self.slide_direction_bias, self.last_chosen_angle = None, None
            return

        try:
            tf_global = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            tf_enemy_global = self.tf_buffer.lookup_transform(self.map_frame, self.enemy_frame, rclpy.time.Time())
            self.last_tf_time = now
        except tf2_ros.TransformException:
            if now - self.last_tf_time > TF_TIMEOUT_SEC:
                if now - self.last_log_time > LOG_THROTTLE_SEC:
                    self.get_logger().warn(f"[EVASION CALCULATOR] TF Data lost for >{TF_TIMEOUT_SEC}s! Holding position.")
                    self.last_log_time = now
                if self.last_target_x is not None:
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.map_frame
                    t.child_frame_id = self.target_frame
                    t.transform.translation.x = self.last_target_x
                    t.transform.translation.y = self.last_target_y
                    t.transform.rotation.w = 1.0
                    self.tf_broadcaster.sendTransform(t)
            return

        global_x = tf_global.transform.translation.x
        global_y = tf_global.transform.translation.y
        our_yaw = self.get_yaw_from_quaternion(tf_global.transform.rotation)
        enemy_global_x = tf_enemy_global.transform.translation.x
        enemy_global_y = tf_enemy_global.transform.translation.y
        enemy_global_yaw = self.get_yaw_from_quaternion(tf_enemy_global.transform.rotation)

        current_cost = self.get_cell_cost(global_x, global_y)

        # ---------------------------------------------------------------------
        # LAYER 0: DEEP STUCK RECOVERY
        # ---------------------------------------------------------------------
        is_stuck = current_cost >= MAX_SAFE_COST

        if is_stuck:
            target_x, target_y, target_yaw = self.get_stuck_recovery_target(
                global_x, global_y, our_yaw, enemy_global_x, enemy_global_y, current_cost
            )
            # CRITICAL: Bypass alpha smoothing entirely during stuck recovery!
            # Smoothing with the old target drags the goal right back into the wall.
            self.last_target_x = target_x
            self.last_target_y = target_y
            self.active_target_x = target_x
            self.active_target_y = target_y
        else:
            # Standard Tactical Evasion
            best_target, max_score = self.evaluate_and_select_best_escape(
                global_x, global_y, our_yaw, enemy_global_x, enemy_global_y, enemy_global_yaw
            )

            if best_target is None or max_score < -25.0:
                if now - self.last_log_time > LOG_THROTTLE_SEC:
                    self.get_logger().warn(f"[EVASION CALCULATOR] Standard paths blocked (score: {max_score:.1f}). EMERGENCY ESCAPE.")
                    self.last_log_time = now
                target_x, target_y, target_yaw = self.execute_emergency_escape(global_x, global_y, enemy_global_x, enemy_global_y)
            else:
                target_x, target_y, target_yaw = best_target

            # Apply standard target smoothing only when NOT stuck
            if self.last_target_x is not None and self.last_target_y is not None:
                alpha = 0.5
                target_x = alpha * target_x + (1.0 - alpha) * self.last_target_x
                target_y = alpha * target_y + (1.0 - alpha) * self.last_target_y
                if math.hypot(target_x - self.last_target_x, target_y - self.last_target_y) < MIN_UPDATE_DIST: 
                    return        
            
            self.last_target_x, self.last_target_y = target_x, target_y

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x = target_x
        t.transform.translation.y = target_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(target_yaw / 2.0)
        t.transform.rotation.w = math.cos(target_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        if now - self.last_log_time > LOG_THROTTLE_SEC:
            dist_to_enemy = math.hypot(global_x - enemy_global_x, global_y - enemy_global_y)
            angle_from_enemy = math.atan2(global_y - enemy_global_y, global_x - enemy_global_x)
            rel_angle_deg = abs(math.degrees(math.atan2(math.sin(angle_from_enemy - enemy_global_yaw), math.cos(angle_from_enemy - enemy_global_yaw))))
            mode_str = "STUCK RECOVERY" if is_stuck else "TACTICAL EVASION"
            self.get_logger().info(
                f"[EVASION] Mode: {mode_str} | Cost: {current_cost} | Dist: {dist_to_enemy:.2f}m | Angle: {rel_angle_deg:.1f}°"
            )
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EvasionCalculator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()