# Add localization
# add coordinate communication,

"""AStar Path Planner for the e-puck robot in Webots"""
# This program implements Hardware-in-the-Loop simulation of the e-puck robot.
# Ground sensor data is pre-processed and transfered to an external board 
# that must decide the next state of the robot. Communication between Webots
# and the external board is implemented via Serial port.

# Tested on Webots R2023a, on Windows 11 running Python 3.10.5 64-bit
# communicating with MicroPython v1.25.0 on generic ESP32 module with ESP32

# Author: R. Kleine
# Date: 1 June 2025
# Last update: 2 June 2025

from controller import Robot # type: ignore
import numpy as np # type: ignore
import struct
import socket # MicroPython's socket module

# --- Webots Specific Initialization ---
robot = Robot()
timestep = int(robot.getBasicTimeStep())
delta_t = timestep / 1000.0

# Initialize Webots devices

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']

for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
oldEncoderValues = [0.0, 0.0] # Initialize to avoid errors on first step

gs = []
gsNames = ['gs0', 'gs1', 'gs2']

for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# --- Socket Communication Setup (Client) ---
HOST = '192.168.137.42'  # Standard loopback interface address (localhost) - replace with ESP32 IP
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

print(f"Attempting to connect to ESP32 at {HOST}:{PORT}...")
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    print("Successfully connected to ESP32!")

except socket.error as e:
    print(f"Failed to connect to ESP32: {e}")
    print("Please ensure the ESP32 server is running and its IP is correct.")
    robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE) # Pause simulation on connection failure
    exit() # Exit Webots controller

# Initial robot pose (adjust to match your simulation's starting point if different from ESP32's expectation)
# Note: The ESP32 will maintain its own internal pose. This is mainly for initial setup consistency.
x = 0.0
y = 0.0
phi = 0

# Main control loop
while robot.step(timestep) != -1:
    # --- Sense ---
    encoderValues = [enc.getValue() for enc in encoder]
    gsValues = [s.getValue() for s in gs]

    # Calculate wheel speeds for odometry on ESP32
    if oldEncoderValues[0] == 0.0 and oldEncoderValues[1] == 0.0: # First iteration hack
        wl = 0.0
        wr = 0.0

    else:
        wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t
        wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t

    # Prepare sensor data for sending: [gs0, gs1, gs2, wl, wr, delta_t]
    # Pack as floats (4 bytes each)
    data_to_send = struct.pack('<6f', gsValues[0], gsValues[1], gsValues[2], wl, wr, delta_t)

    try:
        # --- Send Sensor Data to ESP32 ---
        client_socket.sendall(data_to_send)
        # --- Receive Motor Commands from ESP32 ---
        # Expecting 2 floats (leftSpeed, rightSpeed)
        received_data = client_socket.recv(8) # 2 floats * 4 bytes/float = 8 bytes
        if not received_data:
            print("ESP32 disconnected or sent empty data.")
            break
        leftSpeed, rightSpeed = struct.unpack('<2f', received_data)

        # --- Act ---
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

    except socket.error as e:
        print(f"Socket communication error: {e}")
        print("Stopping robot and pausing simulation.")
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)
        break

    # Update old encoder values for next iteration
    oldEncoderValues = encoderValues[:]

    # Debugging (optional, as ESP32 will print detailed state)
    #print(f"Webots: GS: {gsValues[0]:.0f},{gsValues[1]:.0f},{gsValues[2]:.0f} | Sent: Wl:{wl:.2f}, Wr:{wr:.2f} | Rcvd: L:{leftSpeed:.2f}, R:{rightSpeed:.2f}")

print("Webots controller finished.")
client_socket.close()
