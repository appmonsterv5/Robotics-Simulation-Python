"""line_following_with_localization_and_astar.py controller."""

from controller import Robot, DistanceSensor, Motor
import numpy as np
import heapq # For A* priority queue

#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())  # [ms]
delta_t = timestep / 1000.0  # [s]

# Line Following Sub-states (from original code)
line_follow_states = ['forward', 'turn_right', 'turn_left']
line_follow_sub_state = 'forward'  # initial state for line following

# Counter for line following sub-state duration
line_follow_counter = 0
LINE_FOLLOW_COUNTER_MAX = 3 # Original counter max

################################################################################
# Adjust the initial values to match the initial robot pose in your simulation #
# Initial robot pose
x = -0.495148  # position in x [m]
y = 0.361619  # position in y [m]
phi = 0  # orientation [rad]
################################################################################

# Robot wheel speeds
wl = 0.0  # angular speed of the left wheel [rad/s]
wr = 0.0  # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0  # linear speed [m/s]
w = 0.0  # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.020  # radius of the wheels: 20.5mm [m]
D = 0.057  # distance between the wheels: 52mm [m]
# A = 0.05  # distance from the center of the wheels to the point of interest (not used in this version)

# --- Intersection Data (Provided by User) ---
intersection_coords = {
    'A0': (-0.495148, +0.361619), 'A1': (-0.392342, +0.361619), 'A2': (-0.292898, +0.361619), 'A3': (-0.190444, +0.361631),
    'B0': (-0.495148, +0.247345), 'B1': (-0.392342, +0.247345), 'B2': (-0.292898, +0.247345), 'B3': (-0.190444, +0.247345), 'B4': (0, +0.247345), 'B8': (+0.495148, +0.247345),
    'C4': (0, +0.1), 'C8': (+0.495148, +0.1),
    'D0': (-0.495148, 0), 'D4': (0, 0), 'D8': (+0.495148, 0),
    'E0': (-0.495148, -0.1), 'E4': (0, -0.1),
    'F0': (-0.495148, -0.247345), 'F4': (0, -0.247345), 'F5': (+0.190444, -0.247345), 'F6': (+0.292898, -0.247345), 'F7': (+0.392342, -0.247345), 'F8': (+0.495148, -0.247345),
    'G5': (+0.190444, -0.361619), 'G6': (+0.292898, -0.361619), 'G7': (+0.392342, -0.361619), 'G8': (+0.495148, -0.361619),
}
valid_connections = {
    'A0': ['B0'], 'A1': ['B1'], 'A2': ['B2'], 'A3': ['B3'],
    'B0': ['A0', 'B1', 'D0'], 'B1': ['A1', 'B0', 'B2'], 'B2': ['A2', 'B1', 'B3'], 'B3': ['A3', 'B2', 'B4'], 'B4': ['B3', 'B8', 'C4'], 'B8': ['B4', 'C8'],
    'C4': ['B4', 'D4'], 'C8': ['B8', 'D8'], # Adjusted C8 based on likely map structure (C4 likely connects to D4, not C8 to C4)
    'D0': ['B0', 'E0', 'D4'], 'D4': ['C4', 'D0', 'D8', 'E4'], 'D8': ['C8', 'D4', 'F8'],
    'E0': ['D0', 'E4', 'F0'], 'E4': ['C4', 'D4', 'E0', 'F4'],
    'F0': ['E0', 'F4'], 'F4': ['E4', 'F0', 'F5'], 'F5': ['F4', 'F6', 'G5'], 'F6': ['F5', 'F7', 'G6'], 'F7': ['F6', 'F8', 'G7'], 'F8': ['D8', 'F7', 'G8'],
    'G5': ['F5'], 'G6': ['F6'], 'G7': ['F7'], 'G8': ['F8']
}
# Small correction to C8's connections assuming typical grid adjacencies.
# If C8 does indeed connect to C4 directly, revert the change.
# For example, if C4 and C8 are on the same Y and D4, D8 on another Y, then C8->D8 and C4->D4 seems more logical for grid.
# Based on the coordinates, C4 (0, 0.1) and C8 (0.495, 0.1) are on a horizontal line.
# D4 (0,0) and D8 (0.495,0) are on another.
# B4 (0, 0.247) and B8 (0.495, 0.247) are on another.
# It seems C8 should connect to D8, B8. And C4 to D4, B4.
# User's provided: 'C8': ['B8', 'C4', 'D8'] - this means C8 and C4 are connected. Okay, will use user's.

# --- A* Pathfinding and Navigation Variables ---
START_NODE_KEY = 'B0' # Example: Start at D0
GOAL_NODE_KEY = 'G7'  # Example: Goal at G8

navigation_state = "IDLE"  # "IDLE", "PLANNING", "INITIAL_ALIGNMENT", "FOLLOWING_PATH", "AT_INTERSECTION", "TURNING", "ADJUST_AFTER_TURN", "CORRECTING_ORIENTATION", "REACHED_GOAL", "ERROR"
planned_path = []  # List of intersection keys from A*
current_path_segment_index = 0 # Index for the current segment (from planned_path[i] to planned_path[i+1])

effective_target_coord = None # The immediate (x,y) point the robot is moving towards (can be offset)
# For snapping and segment identification
previous_waypoint_actual_coord = None # Actual coord of the start of the current segment
current_target_actual_coord = None   # Actual coord of the end of the current segment (an intersection)

# Waypoint navigation parameters
WAYPOINT_REACHED_THRESHOLD = 0.025  # 2.5 cm, for reaching effective_target_coord
INTERSECTION_APPROACH_OFFSET = 0.005 # 3 cm, for pre/post intersection points
LINE_FOLLOW_SPEED_FACTOR = 0.8 # Speed factor during line following
TURN_COMPLETION_THRESHOLD = np.deg2rad(10)  # Angle error (in rad) to consider a turn complete
ORIENTATION_CORRECTION_THRESHOLD_ERROR = np.deg2rad(20) # If orientation error > X deg, stop and turn

# PID Controller for Turning
KP_TURN = 2.0   # Proportional gain for turning (NEEDS TUNING)
KI_TURN = 0  # Integral gain for turning (NEEDS TUNING)
KD_TURN = 0   # Derivative gain for turning (NEEDS TUNING)
turn_pid = None # Will be initialized as PIDController instance
TURN_EARLY_EXIT_THRESHOLD = np.deg2rad(15) # Angle (in radians) to consider turn "complete enough" to start moving
# To restore state after orientation correction
navigation_state_before_correction = None
effective_target_coord_before_correction = None

#-------------------------------------------------------
# Initialize devices

# distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2'] # gs0: right, gs1: middle, gs2: left
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
oldEncoderValues = []

# motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#-------------------------------------------------------
# Robot Localization functions (from original code)

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t_local):
    if not oldEncoderValues or delta_t_local == 0:
        return 0.0, 0.0
    wl_local = (encoderValues[0] - oldEncoderValues[0]) / delta_t_local
    wr_local = (encoderValues[1] - oldEncoderValues[1]) / delta_t_local
    return wl_local, wr_local

def get_robot_speeds(wl_local, wr_local, r_local, d_local):
    u_local = r_local / 2.0 * (wr_local + wl_local)
    w_local = r_local / d_local * (wr_local - wl_local)
    return u_local, w_local

def get_robot_pose(u_local, w_local, x_old, y_old, phi_old, delta_t_local):
    delta_phi = w_local * delta_t_local
    phi_new = phi_old + delta_phi
    
    # Normalize phi to be between -pi and pi
    while phi_new > np.pi:
        phi_new -= 2 * np.pi
    while phi_new < -np.pi:
        phi_new += 2 * np.pi
    
    # Use average phi for dead-reckoning update if w_local is large, or new phi if small
    # phi_avg = (phi_old + phi_new)/2.0 # More accurate for large angle changes
    # For small timesteps, using phi_new or phi_old for delta_x/y is usually fine
    
    delta_x = u_local * np.cos(phi_new) * delta_t_local # Using new phi for heading
    delta_y = u_local * np.sin(phi_new) * delta_t_local
    
    x_new = x_old + delta_x
    y_new = y_old + delta_y
    return x_new, y_new, phi_new

#-------------------------------------------------------
# PID Controller Class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-MAX_SPEED, MAX_SPEED), integral_limits=(-100, 100)): # Added integral limits
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limits = output_limits
        self.integral_limits = integral_limits # Anti-windup for integral

    def update(self, measured_value, dt):
        if dt == 0: return 0 # Avoid division by zero
        error = self.setpoint - measured_value
        
        # Handle angle wrapping for orientation PID: error should be +/- pi
        if error > np.pi:
            error -= 2 * np.pi
        elif error < -np.pi:
            error += 2 * np.pi
            
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1]) # Anti-windup

        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def set_setpoint(self, setpoint):
        # Normalize setpoint to be between -pi and pi if it's an angle
        while setpoint > np.pi:
            setpoint -= 2 * np.pi
        while setpoint < -np.pi:
            setpoint += 2 * np.pi
        self.setpoint = setpoint
        self.reset()

#-------------------------------------------------------
# A* Pathfinding Algorithm
def heuristic(node_key, goal_key, coords):
    n1 = coords[node_key]
    n2 = coords[goal_key]
    return np.sqrt((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2)

def a_star_search(start_key, goal_key, coords, connections):
    if start_key not in coords or goal_key not in coords:
        print(f"Error: Start ({start_key}) or Goal ({goal_key}) node not in coordinates.")
        return None

    open_set = []
    heapq.heappush(open_set, (0, start_key)) # (f_score, node_key)
    
    came_from = {} 
    g_score = {node: float('inf') for node in coords}
    g_score[start_key] = 0
    f_score = {node: float('inf') for node in coords}
    f_score[start_key] = heuristic(start_key, goal_key, coords)
    
    open_set_hash = {start_key}

    while open_set:
        _, current_key = heapq.heappop(open_set)
        
        if current_key not in open_set_hash: # Already processed
            continue
        open_set_hash.remove(current_key)


        if current_key == goal_key:
            path = []
            temp_key = current_key
            while temp_key in came_from:
                path.append(temp_key)
                temp_key = came_from[temp_key]
            path.append(start_key)
            return path[::-1] 

        if current_key not in connections:
            continue

        for neighbor_key in connections[current_key]:
            if neighbor_key not in coords: # Skip if neighbor doesn't have coordinates
                print(f"Warning: Neighbor {neighbor_key} of {current_key} not in intersection_coords.")
                continue
            cost = heuristic(current_key, neighbor_key, coords) 
            tentative_g_score = g_score[current_key] + cost
            
            if tentative_g_score < g_score[neighbor_key]:
                came_from[neighbor_key] = current_key
                g_score[neighbor_key] = tentative_g_score
                f_score[neighbor_key] = tentative_g_score + heuristic(neighbor_key, goal_key, coords)
                if neighbor_key not in open_set_hash: # Add to queue only if not processed or not in queue
                    heapq.heappush(open_set, (f_score[neighbor_key], neighbor_key))
                    open_set_hash.add(neighbor_key) # Mark as being in the queue
    print(f"Path from {start_key} to {goal_key} not found.")
    return None

#-------------------------------------------------------
# Helper functions for Navigation

def get_offset_waypoint(wp1_coord_tuple, wp2_coord_tuple, offset_distance, before_wp2):
    wp1_coord = np.array(wp1_coord_tuple)
    wp2_coord = np.array(wp2_coord_tuple)
    direction_vector = wp2_coord - wp1_coord
    distance_wp1_wp2 = np.linalg.norm(direction_vector)
    
    if distance_wp1_wp2 < 1e-6: 
        return wp1_coord 

    unit_vector = direction_vector / distance_wp1_wp2
    
    if before_wp2: # Point 'offset_distance' before wp2, along the line from wp1
        # If offset is larger than segment length, effectively target wp1 to avoid going past it backward
        actual_offset = min(offset_distance, distance_wp1_wp2 - 1e-3) # ensure it's not exactly wp2
        return wp2_coord - unit_vector * actual_offset
    else: # Point 'offset_distance' after wp1, along the line towards wp2
        actual_offset = min(offset_distance, distance_wp1_wp2 - 1e-3)
        return wp1_coord + unit_vector * actual_offset

def calculate_angle_to_target(current_x, current_y, target_x, target_y):
    return np.arctan2(target_y - current_y, target_x - current_x)

def determine_line_following_speeds(gs_values_local, sub_state, max_s, counter_val, counter_max_val):
    """ Encapsulates the original line following logic.
        Returns (leftSpeed, rightSpeed, new_sub_state, new_counter)
    """
    l_speed = 0.0
    r_speed = 0.0
    new_sub_s = sub_state
    new_counter = counter_val

    line_right_detected = gs_values_local[0] < 500  # gs0 is right sensor
    line_center_detected = gs_values_local[1] < 500 # Assuming 400 is a threshold for center line
    line_left_detected = gs_values_local[2] < 500   # gs2 is left sensor

    if sub_state == 'forward':
        l_speed = max_s * LINE_FOLLOW_SPEED_FACTOR
        r_speed = max_s * LINE_FOLLOW_SPEED_FACTOR
        if line_right_detected and not line_left_detected and not line_center_detected: # Veer left (robot sees line on its right)
            new_sub_s = 'turn_left' # Turn robot left to correct
            new_counter = 0
        elif line_left_detected and not line_right_detected and not line_center_detected: # Veer right
            new_sub_s = 'turn_right'
            new_counter = 0
        # If only center is detected, or multiple, stay forward (or add more refined logic)

    elif sub_state == 'turn_left': # Robot is turning left
        l_speed = 0.3 * max_s * LINE_FOLLOW_SPEED_FACTOR # Slower wheel on the inside of the turn
        r_speed = 0.7 * max_s * LINE_FOLLOW_SPEED_FACTOR
        if new_counter >= counter_max_val or line_center_detected: # Finish turn after some steps or if line centered
            new_sub_s = 'forward'
            
    elif sub_state == 'turn_right': # Robot is turning right
        l_speed = 0.7 * max_s * LINE_FOLLOW_SPEED_FACTOR
        r_speed = 0.3 * max_s * LINE_FOLLOW_SPEED_FACTOR
        if new_counter >= counter_max_val or line_center_detected:
            new_sub_s = 'forward'
            
    new_counter += 1
    return l_speed, r_speed, new_sub_s, new_counter

def snap_robot_pose(current_x, current_y, current_phi, prev_wp_coord_tuple, target_wp_coord_tuple, snap_dist_thresh=0.402, snap_angle_thresh_rad=np.deg2rad(10)):
    """Snaps robot's x, y, and phi if it's aligned on a known grid line."""
    global x,y,phi # To modify them directly
    
    if prev_wp_coord_tuple is None or target_wp_coord_tuple is None:
        return

    prev_wp_coord = np.array(prev_wp_coord_tuple)
    target_wp_coord = np.array(target_wp_coord_tuple)

    dx_segment = target_wp_coord[0] - prev_wp_coord[0]
    dy_segment = target_wp_coord[1] - prev_wp_coord[1]

    # Check if centered on line: gs1 high, gs0 and gs2 low
    # Thresholds might need adjustment based on your specific sensor readings for line/no line
    is_centered_on_line = gs[1].getValue() < 500 and gs[0].getValue() > 500 and gs[2].getValue() > 500
    if not is_centered_on_line:
        return

    # Determine if segment is primarily horizontal or vertical
    slope_tolerance = 0.1 # Consider segment straight if |slope| < tolerance or 1/|slope| < tolerance

    snapped_phi_target = None

    if abs(dx_segment) > 1e-5 and (abs(dy_segment) < 1e-5 or abs(dy_segment / dx_segment) < slope_tolerance):  # Horizontal segment
        ideal_y = (prev_wp_coord[1] + target_wp_coord[1]) / 2.0 # Use average, or one of them if perfectly horizontal
        if abs(current_y - ideal_y) < snap_dist_thresh:
            y = ideal_y # Snap Y
            print(f"Snapped Y to {y:.3f}")
        if dx_segment > 0: snapped_phi_target = 0.0      # Moving East
        else: snapped_phi_target = np.pi                 # Moving West
        
    elif abs(dy_segment) > 1e-5 and (abs(dx_segment) < 1e-5 or abs(dx_segment / dy_segment) < slope_tolerance):  # Vertical segment
        ideal_x = (prev_wp_coord[0] + target_wp_coord[0]) / 2.0
        if abs(current_x - ideal_x) < snap_dist_thresh:
            x = ideal_x # Snap X
            print(f"Snapped X to {x:.3f}")
        if dy_segment > 0: snapped_phi_target = np.pi / 2.0   # Moving North
        else: snapped_phi_target = -np.pi / 2.0               # Moving South

    if snapped_phi_target is not None:
        phi_error = snapped_phi_target - current_phi
        while phi_error > np.pi: phi_error -= 2 * np.pi
        while phi_error < -np.pi: phi_error += 2 * np.pi
        if abs(phi_error) < snap_angle_thresh_rad:
            phi = snapped_phi_target # Snap Phi
            print(f"Snapped Phi to {np.rad2deg(phi):.1f}")


#-------------------------------------------------------
# Main loop:
print(f"Controller started. Initial Pose: x={x:.3f}, y={y:.3f}, phi={np.rad2deg(phi):.1f} deg")
print(f"Targeting from {START_NODE_KEY} to {GOAL_NODE_KEY}")

while robot.step(timestep) != -1:
    # Store current pose to calculate speed for next iteration if needed
    # x_prev_step, y_prev_step, phi_prev_step = x, y, phi

    # Update sensor readings
    psValues = [s.getValue() for s in ps]
    gsValues = [s.getValue() for s in gs] # gs0: R, gs1: M, gs2: L
    encoderValues = [e.getValue() for e in encoder]

    if not oldEncoderValues: # First iteration
        oldEncoderValues = list(encoderValues) # Use list() for a copy

    # --- Localization ---
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
    
    # Define motor speeds, default to zero unless set by logic below
    leftSpeed = 0.0
    rightSpeed = 0.0

    # --- Navigation State Machine ---
    if navigation_state == "IDLE":
        leftSpeed, rightSpeed = 0.0, 0.0
        navigation_state = "PLANNING"
        print("State: IDLE -> PLANNING")

    elif navigation_state == "PLANNING":
        leftSpeed, rightSpeed = 0.0, 0.0
        planned_path = a_star_search(START_NODE_KEY, GOAL_NODE_KEY, intersection_coords, valid_connections)
        if planned_path and len(planned_path) >= 2:
            current_path_segment_index = 0
            
            # Setup for the very first segment
            # Start node of the path
            current_node_key = planned_path[current_path_segment_index]
            # Next node in the path (end of the first segment)
            next_node_key = planned_path[current_path_segment_index + 1]
            
            previous_waypoint_actual_coord = intersection_coords[current_node_key]
            current_target_actual_coord = intersection_coords[next_node_key]

            # Check initial orientation relative to the first segment
            initial_desired_phi = calculate_angle_to_target(previous_waypoint_actual_coord[0], previous_waypoint_actual_coord[1],
                                                            current_target_actual_coord[0], current_target_actual_coord[1])
            
            phi_error = initial_desired_phi - phi
            while phi_error > np.pi: phi_error -= 2 * np.pi
            while phi_error < -np.pi: phi_error += 2 * np.pi

            if abs(phi_error) > TURN_COMPLETION_THRESHOLD: # Use a tighter threshold for initial alignment if desired
                print(f"State: PLANNING -> INITIAL_ALIGNMENT (Error: {np.rad2deg(phi_error):.1f} deg)")
                turn_pid = PIDController(KP_TURN, KI_TURN, KD_TURN, initial_desired_phi, output_limits=(-1.0, 1.0))
                # Target after alignment: depart point of the start node
                effective_target_coord_after_initial_turn = get_offset_waypoint(previous_waypoint_actual_coord, 
                                                                                current_target_actual_coord, 
                                                                                INTERSECTION_APPROACH_OFFSET, 
                                                                                before_wp2=False) # depart point
                navigation_state_before_correction = "ADJUST_AFTER_TURN" 
                effective_target_coord_before_correction = effective_target_coord_after_initial_turn
                navigation_state = "CORRECTING_ORIENTATION" # Use correcting state for the turn
            else:
                # Orientation is okay, proceed to set depart point and then follow path
                effective_target_coord = get_offset_waypoint(previous_waypoint_actual_coord, 
                                                              current_target_actual_coord, 
                                                              INTERSECTION_APPROACH_OFFSET, 
                                                              before_wp2=False) # depart point
                navigation_state = "ADJUST_AFTER_TURN"
                print(f"State: PLANNING -> ADJUST_AFTER_TURN (Target: {effective_target_coord[0]:.2f},{effective_target_coord[1]:.2f})")
            line_follow_sub_state = 'forward' # Reset line follower
        else:
            print(f"State: PLANNING -> ERROR (Path not found or too short from {START_NODE_KEY} to {GOAL_NODE_KEY})")
            navigation_state = "ERROR"
            
    elif navigation_state == "FOLLOWING_PATH":
        current_pos = np.array([x, y])
        dist_to_effective_target = np.linalg.norm(current_pos - np.array(effective_target_coord))

        # Basic line following logic
        ls, rs, new_lfs_state, new_lfs_counter = determine_line_following_speeds(
            gsValues, line_follow_sub_state, MAX_SPEED, line_follow_counter, LINE_FOLLOW_COUNTER_MAX)
        leftSpeed, rightSpeed = ls, rs
        line_follow_sub_state = new_lfs_state
        line_follow_counter = new_lfs_counter

        # Coordinate Snapping
        snap_robot_pose(x, y, phi, previous_waypoint_actual_coord, current_target_actual_coord)

        # Check for large orientation error w.r.t current segment
        if previous_waypoint_actual_coord and current_target_actual_coord:
            segment_angle = calculate_angle_to_target(previous_waypoint_actual_coord[0], previous_waypoint_actual_coord[1],
                                                      current_target_actual_coord[0], current_target_actual_coord[1])
            phi_err_seg = segment_angle - phi
            while phi_err_seg > np.pi: phi_err_seg -= 2*np.pi
            while phi_err_seg < -np.pi: phi_err_seg += 2*np.pi
            
            if abs(phi_err_seg) > ORIENTATION_CORRECTION_THRESHOLD_ERROR:
                print(f"State: FOLLOWING_PATH -> CORRECTING_ORIENTATION (Seg. Angle Err: {np.rad2deg(phi_err_seg):.1f} deg)")
                if turn_pid is None: turn_pid = PIDController(KP_TURN, KI_TURN, KD_TURN, segment_angle, output_limits=(-1.0, 1.0))
                else: turn_pid.set_setpoint(segment_angle)
                
                navigation_state_before_correction = "FOLLOWING_PATH" # Return to following this path
                effective_target_coord_before_correction = effective_target_coord # Keep same target
                navigation_state = "CORRECTING_ORIENTATION"

        if dist_to_effective_target < WAYPOINT_REACHED_THRESHOLD:
            # Reached the approach point of an intersection
            print(f"State: FOLLOWING_PATH -> AT_INTERSECTION (Reached approach for {planned_path[current_path_segment_index+1]})")
            navigation_state = "AT_INTERSECTION"
            leftSpeed, rightSpeed = 0.0, 0.0 # Stop briefly at intersection point

    elif navigation_state == "AT_INTERSECTION":
        leftSpeed, rightSpeed = 0.0, 0.0 # Stop while deciding
        
        # The intersection we are AT is current_target_actual_coord (which was planned_path[current_path_segment_index + 1])
        at_node_key = planned_path[current_path_segment_index + 1]
        print(f"Robot is at intersection: {at_node_key}")

        if at_node_key == GOAL_NODE_KEY:
            print(f"State: AT_INTERSECTION -> REACHED_GOAL")
            navigation_state = "REACHED_GOAL"
        else:
            # Determine turn needed for the *next* segment
            # Current segment was from planned_path[i] to planned_path[i+1] (at_node_key)
            # Next segment is from planned_path[i+1] to planned_path[i+2]
            if current_path_segment_index + 2 < len(planned_path):
                node_after_turn_key = planned_path[current_path_segment_index + 2]
                coord_at_intersection = intersection_coords[at_node_key]
                coord_node_after_turn = intersection_coords[node_after_turn_key]

                desired_phi_for_turn = calculate_angle_to_target(coord_at_intersection[0], coord_at_intersection[1],
                                                                 coord_node_after_turn[0], coord_node_after_turn[1])
                
                if turn_pid is None: turn_pid = PIDController(KP_TURN, KI_TURN, KD_TURN, desired_phi_for_turn, output_limits=(-1.0,1.0))
                else: turn_pid.set_setpoint(desired_phi_for_turn)
                
                print(f"State: AT_INTERSECTION -> TURNING (To face {node_after_turn_key}. Target phi: {np.rad2deg(desired_phi_for_turn):.1f})")
                navigation_state = "TURNING"
            else: # Should have been caught by GOAL_NODE_KEY check if path is valid
                print(f"State: AT_INTERSECTION -> ERROR (No further path segment to turn to from {at_node_key})")
                navigation_state = "ERROR"
                
    elif navigation_state == "TURNING":
        turn_effort = turn_pid.update(phi, delta_t) # PID output is typically -1 to 1 (or as per output_limits)
        turn_base_speed = MAX_SPEED * 0.35 # Adjust this factor for turn speed
        leftSpeed = -turn_effort * turn_base_speed
        rightSpeed = turn_effort * turn_base_speed

        phi_error_in_turn = turn_pid.setpoint - phi
        while phi_error_in_turn > np.pi: phi_error_in_turn -= 2*np.pi
        while phi_error_in_turn < -np.pi: phi_error_in_turn += 2*np.pi
        print(f"DEBUG Turn: Setpoint={np.rad2deg(turn_pid.setpoint):.1f} deg, Actual Phi={np.rad2deg(phi):.1f} deg, Error={np.rad2deg(phi_error_in_turn):.1f} deg")
        print(f"DEBUG Turn Thresholds: EarlyExit={np.rad2deg(TURN_EARLY_EXIT_THRESHOLD):.1f} deg, Completion={np.rad2deg(TURN_COMPLETION_THRESHOLD):.1f} deg")
        
        # --- MODIFIED TURN COMPLETION LOGIC FOR LINE CUTTING ---
        # The robot considers the turn "complete enough" if:
        # 1. It has reached a very precise alignment (original condition), OR
        # 2. It's within a larger 'early exit' threshold AND it has some linear speed (to prevent waiting indefinitely)
        if abs(phi_error_in_turn) < TURN_COMPLETION_THRESHOLD or \
           (abs(phi_error_in_turn) < TURN_EARLY_EXIT_THRESHOLD and abs(phi_error_in_turn) > TURN_COMPLETION_THRESHOLD and abs(u) > 0.05):
            # Check if the robot is already moving. `u` is the linear speed from localization.
            # This 'abs(u) > 0.05' ensures it doesn't exit turn just because it hit the early threshold while stopped.

            print(f"State: TURNING -> FOLLOWING_PATH (Turn complete early. Error: {np.rad2deg(phi_error_in_turn):.1f} deg)")
            leftSpeed, rightSpeed = MAX_SPEED * LINE_FOLLOW_SPEED_FACTOR, MAX_SPEED * LINE_FOLLOW_SPEED_FACTOR # Start moving immediately

            current_path_segment_index += 1 # Advance to the new segment we just turned for

            # Setup for the new segment
            # Start of new segment is the intersection we just turned at
            current_node_key = planned_path[current_path_segment_index]
            # End of new segment
            next_node_key = planned_path[current_path_segment_index + 1]

            previous_waypoint_actual_coord = intersection_coords[current_node_key]
            current_target_actual_coord = intersection_coords[next_node_key]

            # --- KEY CHANGE: Directly target the APPROACH POINT of the NEXT intersection ---
            # Instead of a 'depart point', we immediately set the target to the approach point
            # of the next major intersection along the path. This forces the robot to
            # transition directly into active line following for the next segment.
            effective_target_coord = get_offset_waypoint(previous_waypoint_actual_coord,
                                                         current_target_actual_coord,
                                                         INTERSECTION_APPROACH_OFFSET,
                                                         before_wp2=True) # Target the approach point of the next intersection

            navigation_state = "FOLLOWING_PATH" # Directly jump to line following
            line_follow_sub_state = 'forward' # Reset line follower to actively find the line
            print(f"  New segment: {current_node_key} -> {next_node_key}. Directly targeting approach: {effective_target_coord[0]:.2f},{effective_target_coord[1]:.2f}")

        # If turn not complete, continue turning
        # The speeds (leftSpeed, rightSpeed) are already set above by the PID output
    elif navigation_state == "ADJUST_AFTER_TURN":
        # Move towards the "depart point" using line following primarily
        current_pos = np.array([x, y])
        dist_to_effective_target = np.linalg.norm(current_pos - np.array(effective_target_coord))

        ls, rs, new_lfs_state, new_lfs_counter = determine_line_following_speeds(
            gsValues, line_follow_sub_state, MAX_SPEED * 0.7, line_follow_counter, LINE_FOLLOW_COUNTER_MAX) # Slightly slower
        leftSpeed, rightSpeed = ls, rs
        line_follow_sub_state = new_lfs_state
        line_follow_counter = new_lfs_counter
        
        # Optional: Minor orientation adjustments if not perfectly aligned by turn
        # angle_to_depart = calculate_angle_to_target(x,y,effective_target_coord[0],effective_target_coord[1])
        # error_to_depart = angle_to_depart - phi 
        # (normalize error_to_depart)
        # if abs(error_to_depart) > np.deg2rad(5): # small adjustment
        #    adj_speed = MAX_SPEED * 0.1 * np.sign(error_to_depart)
        #    leftSpeed += adj_speed
        #    rightSpeed -= adj_speed

        if dist_to_effective_target < WAYPOINT_REACHED_THRESHOLD:
            # Reached depart point, now target the "approach point" of the next intersection
            # Current segment is from previous_waypoint_actual_coord to current_target_actual_coord
            effective_target_coord = get_offset_waypoint(previous_waypoint_actual_coord, 
                                                          current_target_actual_coord, 
                                                          INTERSECTION_APPROACH_OFFSET, 
                                                          before_wp2=True) # approach point
            print(f"State: ADJUST_AFTER_TURN -> FOLLOWING_PATH (Target approach for {planned_path[current_path_segment_index+1]}: {effective_target_coord[0]:.2f},{effective_target_coord[1]:.2f})")
            navigation_state = "FOLLOWING_PATH"

    elif navigation_state == "CORRECTING_ORIENTATION":
        # This state is used for initial alignment and fixing large deviations
        turn_effort = turn_pid.update(phi, delta_t)
        correction_turn_speed = MAX_SPEED * 0.3 # Slower, more controlled turn
        leftSpeed = -turn_effort * correction_turn_speed
        rightSpeed = turn_effort * correction_turn_speed

        phi_error_correction = turn_pid.setpoint - phi
        while phi_error_correction > np.pi: phi_error_correction -= 2*np.pi
        while phi_error_correction < -np.pi: phi_error_correction += 2*np.pi

        if abs(phi_error_correction) < TURN_COMPLETION_THRESHOLD: # Use same threshold
            print(f"State: CORRECTING_ORIENTATION -> {navigation_state_before_correction} (Correction complete. Error: {np.rad2deg(phi_error_correction):.1f} deg)")
            navigation_state = navigation_state_before_correction
            effective_target_coord = effective_target_coord_before_correction # Restore target
            # Robot should now resume its previous activity (e.g., line following)
            line_follow_sub_state = 'forward'

    elif navigation_state == "REACHED_GOAL":
        leftSpeed, rightSpeed = 0.0, 0.0
        if robot.getTime() % 2 < 1: # Print every second or so
             print(f"Goal {GOAL_NODE_KEY} reached! Final Pose: x={x:.3f}, y={y:.3f}, phi={np.rad2deg(phi):.1f}. Sim Time: {robot.getTime():.2f}s")
        # To stop the simulation or controller after reaching goal:
        # return # or robot.simulationSetMode(Robot.SIMULATION_MODE_PAUSE) if applicable

    elif navigation_state == "ERROR":
        leftSpeed, rightSpeed = 0.0, 0.0
        if robot.getTime() % 2 < 1:
            print("Navigation Error. Robot stopped.")
        # return

    # --- Actuate Motors ---
    # Clip speeds to be safe, although PID and logic should handle this
    leftSpeed = np.clip(leftSpeed, -MAX_SPEED, MAX_SPEED)
    rightSpeed = np.clip(rightSpeed, -MAX_SPEED, MAX_SPEED)
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # --- Update old values for next iteration ---
    oldEncoderValues = list(encoderValues) # Make a copy

    # --- Debugging Output (selective) ---
    if robot.getTime() % 1.0 < delta_t*2 : # Print roughly every second
        print(f"SimT: {robot.getTime():.2f} NavState: {navigation_state} SubState: {line_follow_sub_state} Pose: x={x:.3f} y={y:.3f} φ={np.rad2deg(phi):.1f}° GS: L{gsValues[2]} M{gsValues[1]} R{gsValues[0]}")
        if effective_target_coord is not None:
             print(f"  Targeting: ({effective_target_coord[0]:.3f}, {effective_target_coord[1]:.3f}) Dist: {np.linalg.norm(np.array([x,y]) - np.array(effective_target_coord)):.3f}m")
        # print(f"LS: {leftSpeed:.2f} RS: {rightSpeed:.2f}")