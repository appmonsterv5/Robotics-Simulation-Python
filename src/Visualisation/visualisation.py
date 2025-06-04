import socket
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import threading
import queue

# Grid definition with node positions
GRID_LAYOUT = [
    ['A0', 1, 'A1', 1, 'A2', 1, 'A3', 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    ['B0', 0, 0, 0, 0, 0, 0, 0, 'B4', 0, 0, 0, 0, 0, 0, 0, 'B8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 'C4', 0, 0, 0, 0, 0, 0, 0, 'C8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    ['D0', 0, 0, 0, 0, 0, 0, 0, 'D4', 0, 0, 0, 0, 0, 0, 0, 'D8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    ['E0', 0, 0, 0, 0, 0, 0, 0, 'E4', 1, 1, 1, 1, 1, 1, 1, 'E8'],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    ['F0', 0, 0, 0, 0, 0, 0, 0, 'F4', 0, 'F5', 0, 'F6', 0, 'F7', 0, 'F8'],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 'G5', 1, 'G6', 1, 'G7', 1, 'G8']
]

class RobotVisualizer:
    def __init__(self, host='192.168.121.2', port=65433):
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.robot_position = None  # Current node (e.g., 'E4')
        self.planned_path = []      # List of nodes (e.g., ['B0', 'B4', 'C4'])
        self.robot_marker = None
        self.path_line = None
        self.connected = False
        self.vis_path_nodes = []
        self.vis_current_node = None
        
        # Setup socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(0.1)
        self.host = host
        self.port = port
        self.data_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.recv_buffer = ""
        self.socket_thread = None
        self.try_connect()
        
        # Setup grid visualization
        self.setup_grid()
        
    def try_connect(self):
        if not self.connected:
            try:
                if self.socket:
                    self.socket.close()
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(2.0)
                self.socket.connect((self.host, self.port))
                self.connected = True
                print(f"Connected to robot at {self.host}:{self.port}")
                # Start background thread for receiving data
                self.stop_event.clear()
                self.socket_thread = threading.Thread(target=self.socket_reader, daemon=True)
                self.socket_thread.start()
            except socket.error:
                self.connected = False
                print(f"Waiting for connection at {self.host}:{self.port}")
                time.sleep(1)  # Add delay between retries

    def socket_reader(self):
        """Background thread: read from socket and put complete JSON messages in queue."""
        while not self.stop_event.is_set() and self.connected:
            try:
                data = self.socket.recv(1024)
                if not data:
                    self.connected = False
                    break
                self.recv_buffer += data.decode('utf-8')
                # Split buffer on newline delimiter
                while '\n' in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split('\n', 1)
                    if line.strip():
                        try:
                            robot_data = json.loads(line)
                            self.data_queue.put(robot_data)
                        except Exception as e:
                            print(f"JSON decode error: {e}")
                            continue
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Socket reader error: {e}")
                self.connected = False
                break

    def find_node_position(self, node):
        """Convert node name (e.g., 'E4') to grid position"""
        for i, row in enumerate(GRID_LAYOUT):
            for j, cell in enumerate(row):
                if cell == node:
                    return j, 14-i  # Flip y-axis for plotting
        return None

    def setup_grid(self):
        self.ax.clear()
        
        # Draw grid cells
        for i, row in enumerate(GRID_LAYOUT):
            for j, cell in enumerate(row):
                if cell == 1:  # Wall/obstacle
                    self.ax.add_patch(plt.Rectangle((j-0.5, 14-i-0.5), 1, 1, 
                                                  fill=True, facecolor='lightgray'))
                elif cell == 0 or isinstance(cell, str):  # Path or node
                    self.ax.add_patch(plt.Rectangle((j-0.5, 14-i-0.5), 1, 1, 
                                                  fill=False, edgecolor='black'))
                    if isinstance(cell, str):  # Node
                        self.ax.scatter(j, 14-i, c='blue', s=50)
                        self.ax.annotate(cell, (j, 14-i), xytext=(5, 5), 
                                       textcoords='offset points')
        
        # Set plot properties
        self.ax.set_title('Robot Navigation Visualization')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1, 17)
        self.ax.set_ylim(-1, 15)

    def update_plot(self, frame):
        if not self.connected:
            self.try_connect()
            return

        updated = False
        while not self.data_queue.empty():
            robot_data = self.data_queue.get()
            updated = True
            # Update planned path (blue dotted line)
            if 'planned_path' in robot_data and robot_data['planned_path']:
                self.vis_path_nodes = robot_data['planned_path']
                path_coords = [self.find_node_position(node) for node in self.vis_path_nodes]
                path_coords = [p for p in path_coords if p is not None]
                if path_coords:
                    path_x = [coord[0] for coord in path_coords]
                    path_y = [coord[1] for coord in path_coords]
                    if self.path_line is None:
                        self.path_line, = self.ax.plot(path_x, path_y, 'b:', alpha=0.7, linewidth=2, marker='o', markersize=6)
                    else:
                        self.path_line.set_data(path_x, path_y)
            # Always update robot position if 'current_node' is present (even if no planned_path)
            if 'current_node' in robot_data and robot_data['current_node']:
                node = robot_data['current_node']
                pos = self.find_node_position(node)
                if pos:
                    self.vis_current_node = node
                    # Remove previous robot marker and plot new one
                    if self.robot_marker is not None:
                        self.robot_marker.remove()
                        self.robot_marker = None
                    self.robot_marker = self.ax.scatter(*pos, c='red', s=120, marker='o', zorder=10)
        if updated:
            self.fig.canvas.draw_idle()

    def close(self):
        self.stop_event.set()
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        if self.socket_thread and self.socket_thread.is_alive():
            self.socket_thread.join(timeout=1)

    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot,
                          interval=300,  # Slower interval to match robot update rate
                          cache_frame_data=False,
                          save_count=1000)
        plt.show()

if __name__ == "__main__":
    visualizer = RobotVisualizer()
    try:
        visualizer.run()
    finally:
        visualizer.close()
