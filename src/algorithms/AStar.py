from machine import UART # type: ignore
from time import sleep
import math # type: ignore
from config import COUNTER_MAX, COUNTER_STOP, led_board, button_right, AstarStart, AstarEnd, intersection_coords, valid_connections
from utils import read_Sensor_Status_AStar, led_Control, PathPlanner

#localization of the robot in the world
def get_robot_pose(x_old, y_old, phi_old, delta_x, delta_y, delta_phi):
    x = x_old + delta_x
    y = y_old + delta_y
    phi = phi_old + delta_phi
    # phi_avg = (phi_old + phi)/2   
    if phi >= math.pi:
        phi = phi - 2*math.pi
    elif phi < -math.pi:
        phi = phi + 2*math.pi
    
    return x, y, phi

def run():
    print("A* algorithm is running...")
    # Initialize UART communication
    uart = UART(1, baudrate=115200, tx=1, rx=3)  # Adjust pins as necessary
    
    # Initial status of the line sensor: updated by Webots via serial
    line_left = False
    line_center = False
    line_right = False
    delta_x = 0.0  # change in x [m]
    delta_y = 0.0  # change in y [m]
    delta_phi = 0.0  # change in orientation [rad]
    
    # starting position of the robot in the world
    x = 0.00    # position in x [m]
    y = 0.0    # position in y [m]
    phi = 1.5708  # orientation [rad]

    # Variables to implement the line-following state machine
    current_state = 'forward'
    counter = 0
    state_updated = True

    while True:
        
        ##################   See   ###################
        
        if uart.any():
            try: 
                line_left, line_center, line_right, delta_x, delta_y, delta_phi = read_Sensor_Status_AStar(uart.read())
                led_Control(0, line_left, line_center, line_right)  # Update LED status based on sensor readings

                x, y, phi = get_robot_pose(x, y, phi, delta_x, delta_y, delta_phi)  # Initial robot pose

            except Exception as e:
                print(f"Error reading sensor data: {e}")
                return
        
        ##################   Think   ###################
        
        #AStar algorithm
        # Initialize path planner
        path_planner = PathPlanner(valid_connections, intersection_coords)

        # Define current path - example: go from B0 to F8  
        current_path = []
        current_waypoint_index = 0
        
        print("=== Planning Path ===")
        current_path = path_planner.find_path(AstarStart, AstarEnd)
        if current_path:
            print(f"Path found from {AstarStart} to {AstarEnd}:")
            for i, node in enumerate(current_path):
                coord = intersection_coords[node]
                print(f"  {i}: {node} -> ({coord[0]:.3f}, {coord[1]:.3f})")
            
            # Set initial target
            target_node = current_path[current_waypoint_index]
            target_x, target_y = intersection_coords[target_node]
        else:
            print(f"No valid path found from {AstarStart} to {AstarEnd}")
            target_x, target_y = intersection_coords[AstarStart]
            target_node = AstarStart

        
        # if current_state == 'forward':
        #     pass

        # elif current_state == 'turn_left':
        #     pass

        # elif current_state == 'turn_right':
        #     pass

        # elif current_state == 'turn_around':
        #     pass

        # elif current_state == 'stop':
        #     pass
        
        ##################   Act   ###################
        

# Add coordinate system,
# Add Snapping to grid points,
# Add a*,
# Add following planned path,
# Add obstacle detection,
# Add replanning and rerunning route.