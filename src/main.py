# Implements communication with Webots via Serial over USB.
# Works with the "line_following_with_HIL" controller in Webots.

# Tested with MicroPython v1.25.0 on ESP32 WROOM 32 dev kit board
# communicating with Webots R2023a, on Windows 11 running Python 3.10.5 64-bit

# To use ulab, you need to install a micropython interpreter that contains this
# module. In my tests, I used the this one:
# https://gitlab.com/rcolistete/micropython-firmwares/-/blob/master/ESP32/v1.12_with_ulab/ulab_v0.54.0_2020-07-29/Generic_flash-4MB/esp32_idf4_ulab_dp_thread_v1.12-663-gea4670d5a_2020-07-29.bin

# Author: R. Kleine 
# Date: 1 June 2025
# Last update: 1 June 2025


###### Close Thonny after start running this code #######
# This is necessary because the serial port cannot be used by
# two different programs (Thonny and Webots) at the same time.


from machine import Pin, UART # type: ignore
from time import sleep
# import ulab
from config import COUNTER_MAX, COUNTER_STOP, led_board, button_left, button_right
from utils import read_Sensor_Status, led_Control


# Set serial to UART0 to guarantee USB communication in if current_state == of reset
# uart = UART(0, 115200, tx=1, rx=3)

# Button value is normally False. Returns True when clicked.

# Wait for the button click before changing the serial port to UART1.
# During the wait period, the program can be stopped using the STOP button.
print("Click the button on the ESP32 to continue. Then, close Thonny and run the Webots simulation.")
print("Or click STOP in Thonny to return to the REPL.")
while button_left() == False:
    sleep(0.25)
    led_board.value(not led_board())

# Set serial to UART1 using the same pins as UART0 to communicate via USB
uart = UART(1, 115200, tx=1, rx=3)

# Initial status of the line sensor: updated by Webots via serial
line_left = False
line_center = False
line_right = False

# Variables to implement the line-following state machine
current_state = 'forward'
counter = 0
state_updated = True

while True:
    
    ##################   See   ###################
    
    # Check if anything was received via serial to update sensor status
    if uart.any():
        line_left, line_center, line_right = read_Sensor_Status(uart.read())
        print(f"Line sensors: left={line_left}, center={line_center}, right={line_right}")

    ##################   Think   ###################

    # Implement the line-following state machine transitions    
    if current_state == 'forward':
        led_Control(1, 0, 0, 0)  # Turn on yellow LED when moving forward
        counter = 0
        # Check if the button is pressed to stop
        if button_right.value() == True:
            current_state = 'stop'
            state_updated = True

        # Check the line sensor status to determine the next state
        elif line_left and not line_right:
            current_state = 'turn_right'    
            state_updated = True
        elif not line_left and line_right:
            current_state = 'turn_left'
            state_updated = True
        elif not line_left and not line_right and not line_center: # lost the line
            current_state = 'turn_left'
            state_updated = True

    elif current_state == 'turn_right':
        led_Control(0, 1, 0, 0)  # Turn on blue LED when turning right
        if button_right.value() == True:
            current_state = 'stop'
            state_updated = True
        elif counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True

    elif current_state == 'turn_left':
        led_Control(0, 0, 1, 0)  # Turn on green LED when turning left
        if button_right.value() == True:
            current_state = 'stop'
            state_updated = True
        elif counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True

    elif current_state == 'stop':
        led_Control(0, 0, 0, 1)  # Turn on red LED when stopped
        led_board.value(1)
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_update = True
            led_board.value(0)


    ##################   Act   ###################

    # Send the new state when updated
    if state_updated == True:
        uart.write(current_state + '\n')
        state_updated = False

    counter += 1    # increment counter
    sleep(0.02)     # wait 0.02 seconds
