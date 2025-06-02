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


AstarStart = "D4" # Starting point for A* algorithm
AstarEnd = "G8" # Ending point for A* algorithm
intersection_coords = {
    # Row 0 (A series)
    'A0': (-0.495148, +0.361619),
    'A1': (-0.392342, +0.361619),
    'A2': (-0.292898, +0.361619),
    'A3': (-0.190444, +0.361631),
    
    # Row 2 (B series)
    'B0': (-0.495148, +0.247345),
    'B1': (-0.392342, +0.247345),
    'B2': (-0.292898, +0.247345),
    'B3': (-0.190444, +0.247345),
    'B4': (0, +0.247345),
    'B8': (+0.495148, +0.247345),
    
    # Row 4 (C series)
    'C4': (0, +0.1),
    'C8': (+0.495148, +0.1),
    
    # Row 6 (D series)
    'D0': (-0.495148, 0),
    'D4': (0, 0),
    'D8': (+0.495148, 0),
    
    # Row 8 (E series)
    'E0': (-0.495148, -0.1),
    'E4': (0, -0.1),
    
    # Row 10 (F series)
    'F0': (-0.495148, -0.247345),
    'F4': (0, -0.247345),
    'F5': (+0.190444, -0.247345),
    'F6': (+0.292898, -0.247345),
    'F7': (+0.392342, -0.247345),
    'F8': (+0.495148, -0.247345),
    
    # Row 12 (G series)
    'G5': (+0.190444, -0.361619),
    'G6': (+0.292898, -0.361619),
    'G7': (+0.392342, -0.361619),
    'G8': (+0.495148, -0.361619),
}

# Define valid path connections based on your constraints
valid_connections = {
    'A0': ['B0'],
    'A1': ['B1'],
    'A2': ['B2'],
    'A3': ['B3'],
    'B0': ['A0', 'B1', 'D0'],
    'B1': ['A1', 'B0', 'B2'],
    'B2': ['A2', 'B1', 'B3'],
    'B3': ['A3', 'B2', 'B4'],
    'B4': ['B3', 'B8', 'C4'],
    'B8': ['B4', 'C8'],
    'C4': ['B4', 'D4', 'E4'],
    'C8': ['B8', 'C4', 'D8'],
    'D0': ['B0', 'E0', 'D4'],
    'D4': ['C4', 'D0', 'D8', 'E4'],
    'D8': ['C8', 'D4', 'F8'],
    'E0': ['D0', 'E4', 'F0'],
    'E4': ['C4', 'D4', 'E0', 'F4'],
    'F0': ['E0', 'F4'],
    'F4': ['E4', 'F0', 'F5'],
    'F5': ['F4', 'F6', 'G5'],
    'F6': ['F5', 'F7', 'G6'],
    'F7': ['F6', 'F8', 'G7'],
    'F8': ['D8', 'F7', 'G8'],
    'G5': ['F5'],
    'G6': ['F6'],
    'G7': ['F7'],
    'G8': ['F8']
}