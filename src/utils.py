from config import led_yellow, led_blue, led_green, led_red
import math # type: ignore
import heapq

def normalize_angle(angle_rad):
    while angle_rad > math.pi: angle_rad -= 2 * math.pi
    while angle_rad < -math.pi: angle_rad += 2 * math.pi
    return angle_rad

WAYPOINT_REACHED_THRESHOLD_ESP = 0.025
INTERSECTION_APPROACH_OFFSET_ESP = 0.005
LINE_FOLLOW_SPEED_FACTOR_ESP = 0.8
TURN_COMPLETION_THRESHOLD_ESP = math.radians(10)
ORIENTATION_CORRECTION_THRESHOLD_ERROR_ESP = math.radians(20)
KP_TURN_ESP = 2.0
KI_TURN_ESP = 0.0
KD_TURN_ESP = 0.0
TURN_EARLY_EXIT_THRESHOLD_ESP = math.radians(15) # This constant is not used in the provided code.
LINE_FOLLOW_COUNTER_MAX_ESP = 3

def heuristic_esp(node_key, goal_key, coords):
    n1 = coords[node_key]
    n2 = coords[goal_key]
    return math.sqrt((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2)

def a_star_search_esp(start_key, goal_key, coords, connections):
    if start_key not in coords or goal_key not in coords:
        print(f"A* Error: Start ({start_key}) or Goal ({goal_key}) not in coords.")
        return None
    open_set = []
    heapq.heappush(open_set, (0, start_key))
    came_from = {}
    g_score = {node: float('inf') for node in coords}
    g_score[start_key] = 0
    f_score = {node: float('inf') for node in coords}
    f_score[start_key] = heuristic_esp(start_key, goal_key, coords)
    open_set_hash = {start_key}
    while open_set:
        _, current_key = heapq.heappop(open_set)
        if current_key not in open_set_hash: continue
        open_set_hash.remove(current_key)
        if current_key == goal_key:
            path = []
            temp_key = current_key
            while temp_key in came_from:
                path.append(temp_key)
                temp_key = came_from[temp_key]
            path.append(start_key)
            return path[::-1]
        if current_key not in connections: continue
        for neighbor_key in connections[current_key]:
            if neighbor_key not in coords:
                print(f"A* Warning: Neighbor {neighbor_key} of {current_key} not in coords.")
                continue
            cost = heuristic_esp(current_key, neighbor_key, coords)
            tentative_g_score = g_score[current_key] + cost
            if tentative_g_score < g_score.get(neighbor_key, float('inf')):
                came_from[neighbor_key] = current_key
                g_score[neighbor_key] = tentative_g_score
                new_f_score = tentative_g_score + heuristic_esp(neighbor_key, goal_key, coords)
                f_score[neighbor_key] = new_f_score
                heapq.heappush(open_set, (new_f_score, neighbor_key))
                open_set_hash.add(neighbor_key)
    print(f"A* Path from {start_key} to {goal_key} not found.")
    return None


class PIDController_ESP:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-MAX_SPEED_ESP, MAX_SPEED_ESP), integral_limits=(-100, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = normalize_angle(setpoint)
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limits = output_limits
        self.integral_limits = integral_limits

    def update(self, measured_value, dt):
        if dt == 0: return 0
        error = self.setpoint - measured_value
        error = normalize_angle(error)
        self.integral += error * dt
        self.integral = clip_value(self.integral, self.integral_limits[0], self.integral_limits[1])
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        if self.output_limits:
            output = clip_value(output, self.output_limits[0], self.output_limits[1])
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def set_setpoint(self, setpoint):
        self.setpoint = normalize_angle(setpoint)
        self.reset()
        
        