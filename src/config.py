from machine import Pin # type: ignore

COUNTER_MAX = 5
COUNTER_STOP = 50

led_board = Pin(2, Pin.OUT)     # Define ESP32 onboard LED
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)
button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)