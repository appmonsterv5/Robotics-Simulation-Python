from machine import UART # type: ignore
from time import sleep
import math # type: ignore
from config import START_NODE_KEY_ESP, GOAL_NODE_KEY_ESP, INTERSECTION_COORDS_ESP, VALID_CONNECTIONS_ESP
from utils import (
    normalize_angle, LINE_FOLLOW_COUNTER_MAX_ESP, INTERSECTION_APPROACH_OFFSET_ESP,
    TURN_COMPLETION_THRESHOLD_ESP, ORIENTATION_CORRECTION_THRESHOLD_ERROR_ESP,
    KP_TURN_ESP, KI_TURN_ESP, KD_TURN_ESP, WAYPOINT_REACHED_THRESHOLD_ESP,
    LINE_FOLLOW_SPEED_FACTOR_ESP, a_star_search_esp, PIDController_ESP, clip_value  # <-- Add clip_value here
)
import network
import usocket as socket
import ustruct as struct

# --- Wi-Fi Setup ---
wlan = network.WLAN(network.STA_IF)
ESP32_IP_ADDRESS = wlan.ifconfig()[0] if wlan.isconnected() else "0.0.0.0"

# --- Constants (Ported from Webots, renamed with _ESP suffix for clarity) ---
MAX_SPEED_ESP = 6.28
R_ESP = 0.020  # Wheel radius
D_ESP = 0.057  # Wheelbase (distance between wheels)

# --- Global State Variables for ESP32 ---
# Initial Pose - IMPORTANT: Match this to your robot's starting pose in Webots
# The Webots script provided starts at x=0, y=0, phi=1.5707 (pi/2)
# If your grid (INTERSECTION_COORDS_ESP) assumes a different start for START_NODE_KEY_ESP, adjust one or the other.
# For START_NODE_KEY_ESP = 'B0' which is (-0.495, 0.247), phi approx 0 for path to B1
# For now, let's assume Webots starts at B0's coordinates for consistency.
# If Webots starts at (0,0,pi/2), and that corresponds to a node (e.g. 'D4'), then set START_NODE_KEY_ESP = 'D4'
# and x_esp, y_esp, phi_esp accordingly.
# Let's assume your Webots script's initial pose (0,0,pi/2) should map to 'D4' in your grid
# D4: (0,0)
x_esp = -0.495148
y_esp = 0.361619
phi_esp = 0.0
# Line Following State
line_follow_sub_state_esp = 'forward'
line_follow_counter_esp = 0

# Navigation State
navigation_state_esp = "IDLE" # Will transition to PLANNING on first data
planned_path_esp = []
current_path_segment_index_esp = 0
effective_target_coord_esp = None
previous_waypoint_actual_coord_esp = None
current_target_actual_coord_esp = None

# PID Controller state
turn_pid_esp = None

# State for orientation correction
navigation_state_before_correction_esp = None
effective_target_coord_before_correction_esp = None

# Ground sensor values received from Webots
gsValues_current_esp = [1000.0, 1000.0, 1000.0] # Default to no line

def clip_value(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def get_robot_speeds_esp(wl_local, wr_local, r_local, d_local): # Same as before
    u_local = r_local / 2.0 * (wr_local + wl_local)
    w_local = r_local / d_local * (wr_local - wl_local)
    return u_local, w_local

def get_robot_pose_esp(u_local, w_local, x_old, y_old, phi_old, delta_t_local): # Same as before
    delta_phi = w_local * delta_t_local
    phi_new = phi_old + delta_phi
    phi_new = normalize_angle(phi_new)
    delta_x = u_local * math.cos(phi_new) * delta_t_local
    delta_y = u_local * math.sin(phi_new) * delta_t_local
    x_new = x_old + delta_x
    y_new = y_old + delta_y
    return x_new, y_new, phi_new


# --- Navigation Logic (Ported) --- (Same as before, e.g., get_offset_waypoint_esp, etc.)
def get_offset_waypoint_esp(wp1_coord_tuple, wp2_coord_tuple, offset_distance, before_wp2):
    wp1_x, wp1_y = wp1_coord_tuple; wp2_x, wp2_y = wp2_coord_tuple
    direction_vector_x = wp2_x - wp1_x; direction_vector_y = wp2_y - wp1_y
    distance_wp1_wp2 = math.sqrt(direction_vector_x**2 + direction_vector_y**2)
    if distance_wp1_wp2 < 1e-6: return wp1_coord_tuple
    unit_vector_x = direction_vector_x / distance_wp1_wp2; unit_vector_y = direction_vector_y / distance_wp1_wp2
    actual_offset = min(offset_distance, distance_wp1_wp2 - 1e-3)
    if before_wp2: return (wp2_x - unit_vector_x * actual_offset, wp2_y - unit_vector_y * actual_offset)
    else: return (wp1_x + unit_vector_x * actual_offset, wp1_y + unit_vector_y * actual_offset)

def calculate_angle_to_target_esp(current_x, current_y, target_x, target_y):
    return math.atan2(target_y - current_y, target_x - current_x)

# --- PID for Line Following ---
class LineFollowPID:
    def __init__(self, kp=0.5, ki=0.0, kd=0.125, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return clip_value(output, self.output_limits[0], self.output_limits[1])

line_pid = LineFollowPID()

def determine_line_following_speeds_esp(gs_values_local, sub_state, max_s, counter_val, counter_max_val):
    # PID-based line following for straight lines, using left-right sensor difference
    l_speed = 0.0; r_speed = 0.0; new_sub_s = sub_state; new_counter = counter_val
    line_right_detected = gs_values_local[0] < 500 # gs0 is Right
    line_center_detected = gs_values_local[1] < 500 # gs1 is Middle
    line_left_detected = gs_values_local[2] < 500 # gs2 is Left

    # Error: left sensor minus right sensor (want to keep this near zero)
    error = (gs_values_local[2] - gs_values_local[0])

    # Only use PID when in 'forward' state
    if sub_state == 'forward':
        dt = 0.05  # Assume 50ms loop if not available (should be replaced with actual delta_t)
        pid_output = line_pid.update(error, dt)
        base_speed = max_s * LINE_FOLLOW_SPEED_FACTOR_ESP
        l_speed = clip_value(base_speed - pid_output, 0, max_s)
        r_speed = clip_value(base_speed + pid_output, 0, max_s)
        # State transitions for sharp corrections
        if line_right_detected and not line_left_detected and not line_center_detected:
            new_sub_s = 'turn_left'; new_counter = 0
        elif line_left_detected and not line_right_detected and not line_center_detected:
            new_sub_s = 'turn_right'; new_counter = 0
    elif sub_state == 'turn_left':
        l_speed = 0.3 * max_s * LINE_FOLLOW_SPEED_FACTOR_ESP; r_speed = 0.7 * max_s * LINE_FOLLOW_SPEED_FACTOR_ESP
        if new_counter >= counter_max_val or line_center_detected: new_sub_s = 'forward'
    elif sub_state == 'turn_right':
        l_speed = 0.7 * max_s * LINE_FOLLOW_SPEED_FACTOR_ESP; r_speed = 0.3 * max_s * LINE_FOLLOW_SPEED_FACTOR_ESP
        if new_counter >= counter_max_val or line_center_detected: new_sub_s = 'forward'
    new_counter += 1
    return l_speed, r_speed, new_sub_s, new_counter

def snap_robot_pose_esp(current_x, current_y, current_phi, prev_wp_coord_tuple, target_wp_coord_tuple, local_gs_values, snap_dist_thresh=0.402, snap_angle_thresh_rad=math.radians(10)):
    snapped_x, snapped_y, snapped_phi = current_x, current_y, current_phi; snapped_flag = False
    if prev_wp_coord_tuple is None or target_wp_coord_tuple is None: return snapped_x, snapped_y, snapped_phi, snapped_flag
    prev_wp_x, prev_wp_y = prev_wp_coord_tuple; target_wp_x, target_wp_y = target_wp_coord_tuple
    dx_segment = target_wp_x - prev_wp_x; dy_segment = target_wp_y - prev_wp_y
    
    # Check if robot is centered on a line
    is_centered_on_line = local_gs_values[1] < 500 and local_gs_values[0] > 500 and local_gs_values[2] > 500

    # Only attempt to snap if centered on a line
    if is_centered_on_line:
        print("DEBUG: Not centered on line, skipping snap.") # Uncomment for detailed snap debugging
        

    # Only attempt to snap if centered on a line
    if not is_centered_on_line:
        print("DEBUG: Not centered on line, skipping snap.") # Uncomment for detailed snap debugging
        return snapped_x, snapped_y, snapped_phi, snapped_flag

    slope_tolerance = 0.1; snapped_phi_target = None
    
    # Determine segment orientation
    if abs(dx_segment) > 1e-5 and (abs(dy_segment) < 1e-5 or abs(dy_segment / dx_segment) < slope_tolerance): # Horizontal
        ideal_y = (prev_wp_y + target_wp_y) / 2.0
        if abs(current_y - ideal_y) < snap_dist_thresh:
            snapped_y = ideal_y
            snapped_flag = True
        if dx_segment > 0: snapped_phi_target = 0.0
        else: snapped_phi_target = math.pi
    elif abs(dy_segment) > 1e-5 and (abs(dx_segment) < 1e-5 or abs(dx_segment / dy_segment) < slope_tolerance): # Vertical
        ideal_x = (prev_wp_x + target_wp_x) / 2.0
        if abs(current_x - ideal_x) < snap_dist_thresh:
            snapped_x = ideal_x
            snapped_flag = True
        if dy_segment > 0: snapped_phi_target = math.pi / 2.0
        else: snapped_phi_target = -math.pi / 2.0
            
    if snapped_phi_target is not None:
        phi_error = normalize_angle(snapped_phi_target - current_phi)
        if abs(phi_error) < snap_angle_thresh_rad:
            snapped_phi = snapped_phi_target
            snapped_flag = True
            
    if snapped_flag: # Uncomment for detailed snap debugging
        print(f"DEBUG: Snapped from ({current_x:.3f},{current_y:.3f},{math.degrees(current_phi):.1f}) to ({snapped_x:.3f},{snapped_y:.3f},{math.degrees(snapped_phi):.1f})")
    
    return snapped_x, snapped_y, snapped_phi, snapped_flag

# --- TCP Socket Server Logic ---
def process_robot_data_tcp(client_socket):
    global x_esp, y_esp, phi_esp
    global line_follow_sub_state_esp, line_follow_counter_esp
    global navigation_state_esp, planned_path_esp, current_path_segment_index_esp
    global effective_target_coord_esp, previous_waypoint_actual_coord_esp, current_target_actual_coord_esp
    global turn_pid_esp, navigation_state_before_correction_esp, effective_target_coord_before_correction_esp
    global gsValues_current_esp

    leftSpeed_cmd = 0.0
    rightSpeed_cmd = 0.0

    try:
        # Receive 6 floats (gs0, gs1, gs2, wl, wr, delta_t) = 6 * 4 = 24 bytes
        # The format string '<6f' means little-endian, 6 floats
        expected_bytes = 24
        received_binary_data = b''
        
        # Loop to ensure all expected bytes are received
        bytes_received_this_iter = 0
        while len(received_binary_data) < expected_bytes:
            packet = client_socket.recv(expected_bytes - len(received_binary_data))
            if not packet: # Connection closed by client
                print("ESP32: Client disconnected during recv (no packet).")
                return False # Indicate connection closed
            received_binary_data += packet
            bytes_received_this_iter += len(packet)

        if len(received_binary_data) != expected_bytes:
            print(f"ESP32 Error: Received {len(received_binary_data)} bytes, expected {expected_bytes}. Discarding incomplete data.")
            return False # Discard incomplete data and signal connection issue

        gs0, gs1, gs2, wl_from_webots, wr_from_webots, delta_t_from_webots = struct.unpack('<6f', received_binary_data)
        gsValues_current_esp = [gs0, gs1, gs2]

        # --- DEBUGGING PRINTS: Uncomment these as needed to trace execution ---
        # print(f"ESP32: Received data. State: {navigation_state_esp}, Pose: ({x_esp:.3f},{y_esp:.3f},{math.degrees(phi_esp):.1f}), GS: {gsValues_current_esp}, dt: {delta_t_from_webots:.3f}")
        # --- END DEBUGGING PRINTS ---

        # --- ESP32's Internal Localization (using wl, wr from Webots) ---
        u_esp, w_esp = get_robot_speeds_esp(wl_from_webots, wr_from_webots, R_ESP, D_ESP)
        x_esp, y_esp, phi_esp = get_robot_pose_esp(u_esp, w_esp, x_esp, y_esp, phi_esp, delta_t_from_webots)

        # --- ESP32's Main Control Logic (Ported State Machine - same as HTTP version) ---
        # (The entire state machine logic from the previous HTTP version's handle_request goes here)
        # This part is identical to the state machine in the previous HTTP-based main.py.
        # It uses x_esp, y_esp, phi_esp, gsValues_current_esp, and delta_t_from_webots.
        # It calculates leftSpeed_cmd and rightSpeed_cmd.

        # === BEGIN STATE MACHINE LOGIC ===
        if navigation_state_esp == "IDLE":
            leftSpeed_cmd, rightSpeed_cmd = 0.0, 0.0
            navigation_state_esp = "PLANNING"
            print("ESP32 State: IDLE -> PLANNING")
        elif navigation_state_esp == "PLANNING":
            leftSpeed_cmd, rightSpeed_cmd = 0.0, 0.0
            planned_path_esp = a_star_search_esp(START_NODE_KEY_ESP, GOAL_NODE_KEY_ESP, INTERSECTION_COORDS_ESP, VALID_CONNECTIONS_ESP)
            if planned_path_esp and len(planned_path_esp) >= 2:
                current_path_segment_index_esp = 0
                current_node_key = planned_path_esp[0]; next_node_key = planned_path_esp[1]
                previous_waypoint_actual_coord_esp = INTERSECTION_COORDS_ESP[current_node_key]
                current_target_actual_coord_esp = INTERSECTION_COORDS_ESP[next_node_key]
                initial_desired_phi = calculate_angle_to_target_esp(previous_waypoint_actual_coord_esp[0], previous_waypoint_actual_coord_esp[1], current_target_actual_coord_esp[0], current_target_actual_coord_esp[1])
                phi_error = normalize_angle(initial_desired_phi - phi_esp)
                if abs(phi_error) > TURN_COMPLETION_THRESHOLD_ESP:
                    print(f"ESP32 State: PLANNING -> INITIAL_ALIGNMENT (Err: {math.degrees(phi_error):.1f})")
                    turn_pid_esp = PIDController_ESP(KP_TURN_ESP, KI_TURN_ESP, KD_TURN_ESP, initial_desired_phi, output_limits=(-1.0, 1.0))
                    effective_target_coord_after_initial_turn = get_offset_waypoint_esp(previous_waypoint_actual_coord_esp, current_target_actual_coord_esp, INTERSECTION_APPROACH_OFFSET_ESP, before_wp2=False)
                    navigation_state_before_correction_esp = "ADJUST_AFTER_TURN"; effective_target_coord_before_correction_esp = effective_target_coord_after_initial_turn
                    navigation_state_esp = "CORRECTING_ORIENTATION"
                else:
                    effective_target_coord_esp = get_offset_waypoint_esp(previous_waypoint_actual_coord_esp, current_target_actual_coord_esp, INTERSECTION_APPROACH_OFFSET_ESP, before_wp2=False)
                    navigation_state_esp = "ADJUST_AFTER_TURN"
                    print(f"ESP32 State: PLANNING -> ADJUST_AFTER_TURN (Target: {effective_target_coord_esp[0]:.2f},{effective_target_coord_esp[1]:.2f})")
                line_follow_sub_state_esp = 'forward'
            else:
                print(f"ESP32 State: PLANNING -> ERROR (Path not found from {START_NODE_KEY_ESP} to {GOAL_NODE_KEY_ESP})")
                navigation_state_esp = "ERROR"
        elif navigation_state_esp == "FOLLOWING_PATH":
            current_pos_tuple = (x_esp, y_esp)
            dist_to_effective_target = math.sqrt((current_pos_tuple[0] - effective_target_coord_esp[0])**2 + (current_pos_tuple[1] - effective_target_coord_esp[1])**2)
            ls, rs, new_lfs_state, new_lfs_counter = determine_line_following_speeds_esp(gsValues_current_esp, line_follow_sub_state_esp, MAX_SPEED_ESP, line_follow_counter_esp, LINE_FOLLOW_COUNTER_MAX_ESP)
            leftSpeed_cmd, rightSpeed_cmd = ls, rs
            line_follow_sub_state_esp = new_lfs_state; line_follow_counter_esp = new_lfs_counter
            nx,ny,nphi,snapped = snap_robot_pose_esp(x_esp, y_esp, phi_esp, previous_waypoint_actual_coord_esp, current_target_actual_coord_esp, gsValues_current_esp)
            if snapped:
                x_esp, y_esp, phi_esp = nx,ny,nphi
                print(f"DEBUG: Snapped in FOLLOWING_PATH. New Pose: ({x_esp:.3f},{y_esp:.3f},{math.degrees(phi_esp):.1f})") # Uncomment for detailed snap debugging
            
            # --- DEBUGGING PRINTS ---
            # print(f"ESP32 State: FOLLOWING_PATH. Dist to target ({effective_target_coord_esp[0]:.2f},{effective_target_coord_esp[1]:.2f}): {dist_to_effective_target:.3f}. Speeds: L={leftSpeed_cmd:.2f}, R={rightSpeed_cmd:.2f}") # Uncomment for detailed line following debugging
            # --- END DEBUGGING PRINTS ---

            if previous_waypoint_actual_coord_esp and current_target_actual_coord_esp:
                segment_angle = calculate_angle_to_target_esp(previous_waypoint_actual_coord_esp[0], previous_waypoint_actual_coord_esp[1], current_target_actual_coord_esp[0], current_target_actual_coord_esp[1])
                phi_err_seg = normalize_angle(segment_angle - phi_esp)
                if abs(phi_err_seg) > ORIENTATION_CORRECTION_THRESHOLD_ERROR_ESP:
                    print(f"ESP32 State: FOLLOWING_PATH -> CORRECTING_ORIENTATION (Seg Err: {math.degrees(phi_err_seg):.1f})")
                    if turn_pid_esp is None: turn_pid_esp = PIDController_ESP(KP_TURN_ESP, KI_TURN_ESP, KD_TURN_ESP, segment_angle, output_limits=(-1.0, 1.0))
                    else: turn_pid_esp.set_setpoint(segment_angle)
                    navigation_state_before_correction_esp = "FOLLOWING_PATH"; effective_target_coord_before_correction_esp = effective_target_coord_esp
                    navigation_state_esp = "CORRECTING_ORIENTATION"
            if dist_to_effective_target < WAYPOINT_REACHED_THRESHOLD_ESP:
                print(f"ESP32 State: FOLLOWING_PATH -> AT_INTERSECTION (Approach for {planned_path_esp[current_path_segment_index_esp+1]})")
                navigation_state_esp = "AT_INTERSECTION"; leftSpeed_cmd, rightSpeed_cmd = 0.0, 0.0
        elif navigation_state_esp == "AT_INTERSECTION":
            leftSpeed_cmd, rightSpeed_cmd = 0.0, 0.0
            at_node_key = planned_path_esp[current_path_segment_index_esp + 1]
            # print(f"ESP32: At intersection: {at_node_key}") # Reduce verbosity
            if at_node_key == GOAL_NODE_KEY_ESP:
                print(f"ESP32 State: AT_INTERSECTION -> REACHED_GOAL")
                navigation_state_esp = "REACHED_GOAL"
            else:
                if current_path_segment_index_esp + 2 < len(planned_path_esp):
                    node_after_turn_key = planned_path_esp[current_path_segment_index_esp + 2]
                    coord_at_intersection = INTERSECTION_COORDS_ESP[at_node_key]; coord_node_after_turn = INTERSECTION_COORDS_ESP[node_after_turn_key]
                    desired_phi_for_turn = calculate_angle_to_target_esp(coord_at_intersection[0], coord_at_intersection[1], coord_node_after_turn[0], coord_node_after_turn[1])
                    if turn_pid_esp is None: turn_pid_esp = PIDController_ESP(KP_TURN_ESP, KI_TURN_ESP, KD_TURN_ESP, desired_phi_for_turn, output_limits=(-1.0,1.0))
                    else: turn_pid_esp.set_setpoint(desired_phi_for_turn)
                    print(f"ESP32 State: AT_INTERSECTION -> TURNING (To {node_after_turn_key}, Target phi: {math.degrees(desired_phi_for_turn):.1f})")
                    navigation_state_esp = "TURNING"
                else:
                    print(f"ESP32 State: AT_INTERSECTION -> ERROR (No further path segment from {at_node_key})")
                    navigation_state_esp = "ERROR"
        elif navigation_state_esp == "TURNING":
            if turn_pid_esp is None: navigation_state_esp = "ERROR"; leftSpeed_cmd, rightSpeed_cmd = 0.0,0.0
            else:
                turn_effort = turn_pid_esp.update(phi_esp, delta_t_from_webots); turn_base_speed = MAX_SPEED_ESP * 0.35
                leftSpeed_cmd = -turn_effort * turn_base_speed; rightSpeed_cmd = turn_effort * turn_base_speed
                phi_error_in_turn = normalize_angle(turn_pid_esp.setpoint - phi_esp)
                # --- DEBUGGING PRINTS ---
                # print(f"ESP32 State: TURNING. Phi error: {math.degrees(phi_error_in_turn):.1f}. Speeds: L={leftSpeed_cmd:.2f}, R={rightSpeed_cmd:.2f}") # Uncomment for detailed turning debugging
                # --- END DEBUGGING PRINTS ---

                if abs(phi_error_in_turn) < TURN_COMPLETION_THRESHOLD_ESP:
                    print(f"ESP32 State: TURNING -> ADJUST_AFTER_TURN (Turn complete. Err: {math.degrees(phi_error_in_turn):.1f})")
                    current_path_segment_index_esp += 1
                    # Check if we've reached the end of the path after incrementing
                    if current_path_segment_index_esp + 1 >= len(planned_path_esp):
                        # This means the current segment was the second to last, and the next node is the goal.
                        # We are now at the goal node, so transition to REACHED_GOAL.
                        print(f"ESP32 State: TURNING -> REACHED_GOAL (Path completed after turn)")
                        navigation_state_esp = "REACHED_GOAL"
                    else:
                        current_node_key = planned_path_esp[current_path_segment_index_esp]; next_node_key = planned_path_esp[current_path_segment_index_esp + 1]
                        previous_waypoint_actual_coord_esp = INTERSECTION_COORDS_ESP[current_node_key]; current_target_actual_coord_esp = INTERSECTION_COORDS_ESP[next_node_key]
                        effective_target_coord_esp = get_offset_waypoint_esp(previous_waypoint_actual_coord_esp, current_target_actual_coord_esp, INTERSECTION_APPROACH_OFFSET_ESP, before_wp2=False)
                        navigation_state_esp = "ADJUST_AFTER_TURN"; line_follow_sub_state_esp = 'forward'
                        # print(f"  New segment: {current_node_key} -> {next_node_key}. Targeting DEPART: ({effective_target_coord_esp[0]:.2f},{effective_target_coord_esp[1]:.2f})")
        elif navigation_state_esp == "ADJUST_AFTER_TURN":
            current_pos_tuple = (x_esp, y_esp)
            dist_to_effective_target = math.sqrt((current_pos_tuple[0] - effective_target_coord_esp[0])**2 + (current_pos_tuple[1] - effective_target_coord_esp[1])**2)
            ls, rs, new_lfs_state, new_lfs_counter = determine_line_following_speeds_esp(gsValues_current_esp, line_follow_sub_state_esp, MAX_SPEED_ESP * 0.7, line_follow_counter_esp, LINE_FOLLOW_COUNTER_MAX_ESP)
            leftSpeed_cmd, rightSpeed_cmd = ls, rs
            line_follow_sub_state_esp = new_lfs_state; line_follow_counter_esp = new_lfs_counter
            
            # --- DEBUGGING PRINTS ---
            # print(f"ESP32 State: ADJUST_AFTER_TURN. Dist to target ({effective_target_coord_esp[0]:.2f},{effective_target_coord_esp[1]:.2f}): {dist_to_effective_target:.3f}. Speeds: L={leftSpeed_cmd:.2f}, R={rightSpeed_cmd:.2f}") # Uncomment for detailed adjustment debugging
            # --- END DEBUGGING PRINTS ---

            if dist_to_effective_target < WAYPOINT_REACHED_THRESHOLD_ESP:
                # Check if this is the last segment before the goal
                if current_path_segment_index_esp + 1 == len(planned_path_esp) -1: # The next node is the goal
                    print(f"ESP32 State: ADJUST_AFTER_TURN -> REACHED_GOAL (Approached final segment target)")
                    navigation_state_esp = "REACHED_GOAL"
                else:
                    effective_target_coord_esp = get_offset_waypoint_esp(previous_waypoint_actual_coord_esp, current_target_actual_coord_esp, INTERSECTION_APPROACH_OFFSET_ESP, before_wp2=True)
                    print(f"ESP32 State: ADJUST_AFTER_TURN -> FOLLOWING_PATH (Target approach for {planned_path_esp[current_path_segment_index_esp+1]}: {effective_target_coord_esp[0]:.2f},{effective_target_coord_esp[1]:.2f})")
                    navigation_state_esp = "FOLLOWING_PATH"
        elif navigation_state_esp == "CORRECTING_ORIENTATION":
            if turn_pid_esp is None:
                print(f"ESP32 State: CORRECTING_ORIENTATION -> {navigation_state_before_correction_esp} (PID None, returning to previous state)")
                navigation_state_esp = navigation_state_before_correction_esp
            else:
                turn_effort = turn_pid_esp.update(phi_esp, delta_t_from_webots); correction_turn_speed = MAX_SPEED_ESP * 0.3
                leftSpeed_cmd = -turn_effort * correction_turn_speed; rightSpeed_cmd = turn_effort * correction_turn_speed
                phi_error_correction = normalize_angle(turn_pid_esp.setpoint - phi_esp)
                # --- DEBUGGING PRINTS ---
                # print(f"ESP32 State: CORRECTING_ORIENTATION. Phi error: {math.degrees(phi_error_correction):.1f}. Speeds: L={leftSpeed_cmd:.2f}, R={rightSpeed_cmd:.2f}") # Uncomment for detailed correction debugging
                # --- END DEBUGGING PRINTS ---

                if abs(phi_error_correction) < TURN_COMPLETION_THRESHOLD_ESP:
                    print(f"ESP32 State: CORRECTING_ORIENTATION -> {navigation_state_before_correction_esp} (Correction complete. Err: {math.degrees(phi_error_correction):.1f})")
                    navigation_state_esp = navigation_state_before_correction_esp; effective_target_coord_esp = effective_target_coord_before_correction_esp
                    line_follow_sub_state_esp = 'forward'
        elif navigation_state_esp == "REACHED_GOAL":
            leftSpeed_cmd, rightSpeed_cmd = 0.0, 0.0 # Goal reached, stop.
            print(f"ESP32: Goal {GOAL_NODE_KEY_ESP} reached! Pose: x={x_esp:.3f},y={y_esp:.3f},phi={math.degrees(phi_esp):.1f}")
        elif navigation_state_esp == "ERROR":
            leftSpeed_cmd, rightSpeed_cmd = 0.0, 0.0 # Error state, stop.
            print(f"ESP32: Robot in ERROR state. Stopping.")
        # === END STATE MACHINE LOGIC ===

        # Clip final speeds
        leftSpeed_cmd = clip_value(leftSpeed_cmd, -MAX_SPEED_ESP, MAX_SPEED_ESP)
        rightSpeed_cmd = clip_value(rightSpeed_cmd, -MAX_SPEED_ESP, MAX_SPEED_ESP)

        # Prepare and send response (2 floats: leftSpeed_cmd, rightSpeed_cmd)
        # Format string '<2f' means little-endian, 2 floats
        response_binary_data = struct.pack('<2f', leftSpeed_cmd, rightSpeed_cmd)
        client_socket.sendall(response_binary_data)
        return True # Indicate success

    except OSError as e: # Catch socket errors specifically (e.g., ETIMEDOUT if Webots stops sending)
        print(f"ESP32 Socket OS Error during processing: {e}")
        return False # Indicate connection issue
    except Exception as e:
        print(f"ESP32 Error processing data: {e}")
        # Try to send zero motor speeds if possible, or just close
        try:
            error_response = struct.pack('<2f', 0.0, 0.0)
            client_socket.sendall(error_response)
        except Exception as e2:
            print(f"ESP32: Error sending error response: {e2}")
        return False # Indicate failure
# --- Main Server Loop ---
def start_server(host='0.0.0.0', port=65432): # Port matches Webots controller
    global wlan, ESP32_IP_ADDRESS
    if not wlan.isconnected():
        print("ESP32: Cannot start server, WiFi not connected.")
        return

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1) # Listen for 1 connection
    print(f'ESP32: TCP server listening on {ESP32_IP_ADDRESS}:{port}')

    while True: # Outer loop to allow re-connection if client disconnects
        client_sock = None # Ensure client_sock is None at the start of each new connection attempt
        try:
            print("ESP32: Waiting for a client connection...")
            client_sock, client_addr = server_socket.accept()
            print(f'ESP32: Client connected from {client_addr}')
            client_sock.settimeout(5.0) # Set a timeout for client operations (e.g., 5 seconds)

            while True: # Inner loop to handle data from the connected client
                if not process_robot_data_tcp(client_sock):
                    print("ESP32: Processing failed or client disconnected. Breaking inner loop.")
                    break # Break from inner loop to wait for new connection
        
        except OSError as e:
            print(f"ESP32 Socket OS Error in server loop (client likely disconnected or timeout): {e}")
        except Exception as e:
            print(f"ESP32: Error in server loop: {e}")
        finally:
            if client_sock:
                client_sock.close()
                print("ESP32: Client socket closed.")

    server_socket.close() # Should not be reached in normal operation
    print("ESP32: Server socket closed.")

def reset_initial_esp_state():
    global x_esp, y_esp, phi_esp, line_follow_sub_state_esp, line_follow_counter_esp
    global navigation_state_esp, planned_path_esp, current_path_segment_index_esp
    global effective_target_coord_esp, previous_waypoint_actual_coord_esp, current_target_actual_coord_esp
    global turn_pid_esp, navigation_state_before_correction_esp, effective_target_coord_before_correction_esp
    global gsValues_current_esp

    x_esp = -0.495148
    y_esp = 0.361619
    phi_esp = 0.0
    line_follow_sub_state_esp = 'forward'
    line_follow_counter_esp = 0
    navigation_state_esp = "IDLE" # Important: reset to IDLE
    planned_path_esp = []
    current_path_segment_index_esp = 0
    effective_target_coord_esp = None
    previous_waypoint_actual_coord_esp = None
    current_target_actual_coord_esp = None
    turn_pid_esp = None
    navigation_state_before_correction_esp = None
    effective_target_coord_before_correction_esp = None
    gsValues_current_esp = [1000.0, 1000.0, 1000.0]
    print("ESP32: Global states reset for new run/connection.")


def run():
    reset_initial_esp_state()
    start_server()

# --- Start the server when main.py is executed ---
if __name__ == "__main__":
    run()

