from machine import UART # type: ignore
from time import sleep
#import ulab # type: ignore
from config import COUNTER_MAX, COUNTER_STOP, led_board, button_right
from utils import read_Sensor_Status, led_Control

def run():
    print("A* algorithm is running...")
    # TODO: Implement the A* algorithm logic here