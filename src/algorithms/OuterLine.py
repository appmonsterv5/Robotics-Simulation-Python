from machine import UART # type: ignore
from time import sleep
from config import COUNTER_MAX, COUNTER_STOP, led_board, button_right
from utils import read_Sensor_Status_OuterLine, led_Control

def run():
    print("Outer Line follower is running...")
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
            line_left, line_center, line_right = read_Sensor_Status_OuterLine(uart.read())
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