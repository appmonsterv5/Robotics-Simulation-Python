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

#-------------------------------------------------------
# Open serial port to communicate with the microcontroller

import serial as Ser# type: ignore
try:
    # Change the port parameter according to your system
    serial = Ser.Serial(port='COM7', baudrate=115200, timeout=5) 
except:
    print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
    raise
    
#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

# create the Robot instance for the simulation.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]
delta_t = timestep/1000.0    # [s]

# states
states = ['forward', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'

# counter: used to maintain an active state for a number of cycles
counter = 0
COUNTER_MAX = 3

################################################################################
# Adjust the initial values to match the initial robot pose in your simulation #
# Initial robot pose
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]
################################################################################

# Robot velocity and acceleration
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
RADIUS = 0.020    # radius of the wheels: 20.5mm [m]
DISTANCE = 0.057    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]

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
gsNames = ['gs0', 'gs1', 'gs2']
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
# Robot Localization functions

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    #Encoder values indicate the angular position of the wheel in radians
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


def get_robot_pose(u, w, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t    
    delta_x = u * np.cos(phi) * delta_t
    delta_y = u * np.sin(phi) * delta_t
    
    return delta_x, delta_y, delta_phi


#-------------------------------------------------------
# Main loop:
# perform simulation steps until Webots is stopping the controller
# Implements the see-think-act cycle

while robot.step(timestep) != -1:

    ############################################
    #                  See                     #
    ############################################

    # Update sensor readings
    # Update sensor readings
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]
        
    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())   
            
    # Process sensor data
    line_right = gsValues[0] < 600
    line_center = gsValues[1] < 600
    line_left = gsValues[2] < 600
    
    # Prepare message to send to the microcontroller
    message = ''
    message += '1' if line_left else '0'  # left sensor
    message += '1' if line_center else '0'  # center sensor
    message += '1' if line_right else '0'  # right sensor

    # Robot Localization
    # Compute new robot pose
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, RADIUS, DISTANCE)
    [delta_x, delta_y, delta_phi] = get_robot_pose(u, w, delta_t)

    message += ' '  # add a space to separate sensor data from state
    message += f'{delta_x} '
    message += f'{delta_y} '
    message += f'{delta_phi}'
    msg_bytes = bytes(message + '\n', 'UTF-8')

    ############################################
    #                 Think                    #
    ############################################

    # Serial communication: if something is received, then update the current state
    if serial.in_waiting:
        value = str(serial.readline(), 'UTF-8')[:-1]  # ignore the last character
        current_state = value
    
    # Update speed according to the current state
    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
    elif current_state == 'turn_right':
        leftSpeed = 1 * speed
        rightSpeed = -0.5 * speed
    elif current_state == 'turn_left':
        leftSpeed = -0.5 * speed
        rightSpeed = 1 * speed
    elif current_state == 'turn_around':
        leftSpeed = 0.5 * speed
        rightSpeed = -0.5 * speed
    elif current_state == 'stop':
        leftSpeed = 0.0
        rightSpeed = 0.0
    else:
        print(f'Unknown state: {current_state}. Using default speed values.')
        leftSpeed = speed
        rightSpeed = speed


    ############################################
    #                  Act                     #
    ############################################

    # Update velocity commands for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    oldEncoderValues = encoderValues
    
    # Print sensor message and current state for debugging
    print(f'Sensor message: {msg_bytes} - Current state: {current_state}')

    # Send message to the microcontroller 
    serial.write(msg_bytes)  

serial.close()