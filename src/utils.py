from config import led_yellow, led_blue, led_green, led_red
import math # type: ignore
import heapq

def read_Sensor_Status_OuterLine(msg_bytes) -> tuple[bool, bool, bool]:
    """
    Reads the sensor status from the received message bytes.
    Returns a tuple with the status of the left, center, and right sensors.
    """
    # Convert bytes to string
    msg_str = str(msg_bytes, 'UTF-8')
    
    # Extract sensor status from the message string
    line_left = msg_str[-4:-3] == '1' # left sensor, '1' means line detected
    line_center = msg_str[-3:-2] == '1' # center sensor
    line_right = msg_str[-2:-1] == '1' # right sensor
    
    return (line_left, line_center, line_right)

def read_Sensor_Status_AStar(msg_bytes) -> tuple[bool, bool, bool, float, float, float]:
    """
    Reads the sensor status from the received message bytes.
    Returns a tuple with the status of the left, center, and right sensors.
    """
    # Convert bytes to string
    msg_str = str(msg_bytes, 'UTF-8')
    
    parts = msg_str.split(' ')
    if len(parts) != 4:
        raise ValueError("Received message does not contain enough parts to extract sensor status.")
    
    try:
        # parts[0] is expected to contain the sensor status in the format '1xx' where '1' indicates line detected.
        # Extract sensor status from the message string
        line_left = parts[0][-4:-3] == '1' # left sensor, '1' means line detected
        line_center = parts[0][-3:-2] == '1' # center sensor
        line_right = parts[0][-2:-1] == '1' # right sensor
        
        # parts[1] is expected to contain the delta_x
        delta_x = float(parts[1])
        
        # parts[2] is expected to contain the delta_y
        delta_y = float(parts[2])
        
        # parts[3] is expected to contain the delta_phi
        delta_phi = float(parts[3])
    except (IndexError, ValueError) as e:
        raise ValueError(f"Error parsing message: {msg_str}. Ensure it contains the correct format.") from e
    
    return (line_left, line_center, line_right, delta_x, delta_y, delta_phi)

def led_Control(yellow, blue, green, red) -> None:
    """
    Controls the state of the LEDs based on the provided parameters.
    yellow: bool - state for yellow LED
    blue: bool - state for blue LED
    green: bool - state for green LED
    red: bool - state for red LED
    """
    led_yellow.value(yellow)  # Turn on yellow LED when moving forward
    led_blue.value(blue)
    led_green.value(green)
    led_red.value(red)
    

class PathPlanner:
    def __init__(self, connections, coords):
        self.connections = connections
        self.coords = coords
    
    def get_neighbors(self, node):
        """Get valid neighbors for a given node"""
        return self.connections.get(node, [])
    
    def heuristic(self, node1, node2):
        """Calculate heuristic distance between two nodes"""
        x1, y1 = self.coords[node1]
        x2, y2 = self.coords[node2]
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  # Return the first element since
        
    def find_path(self, start, goal):
        """Find shortest valid path using A* algorithm"""
        if start not in self.coords or goal not in self.coords:
            return []
        
        if start == goal:
            return [start]
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {node: float('inf') for node in self.coords}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.coords}
        f_score[start] = self.heuristic(start, goal)
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []  # No path found