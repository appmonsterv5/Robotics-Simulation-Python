from config import led_yellow, led_blue, led_green, led_red

def read_Sensor_Status(msg_bytes) -> tuple[bool, bool, bool]:
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
            