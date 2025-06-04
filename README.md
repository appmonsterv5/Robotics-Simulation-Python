# Robotics-Simulation-Python

## Overview

This project implements a Hardware-in-the-Loop (HIL) simulation for the e-puck robot using Python and MicroPython. The system enables real-time communication between Webots (robot simulator) and an ESP32 microcontroller, supporting both line-following and A* path planning algorithms.

## Requirements

### Hardware

- ESP32 development board (tested with ESP32 WROOM 32 dev kit)
- PC running Webots (tested on Windows 11)
- USB cable for ESP32-PC connection

### Software

- **Python**: 3.10+ (for Webots controllers)
- **MicroPython**: v1.25.0 (for ESP32)
- **Webots**: R2023a or newer
- **pip packages** (for PC/Webots):
  - `numpy`
  - `pyserial`
- **MicroPython modules** (for ESP32):
  - `network`
  - `usocket`
  - `ustruct`
  - `machine`
  - `time`
  - `heapq` (or `uheapq`)
  - `math`

## Setup Instructions

### 1. Flash MicroPython to ESP32

- Download MicroPython firmware (v1.25.0 or compatible).
- Use [esptool](https://docs.micropython.org/en/latest/esp32/tutorial/intro.html) to flash the firmware.

### 2. Upload ESP32 Code

- Copy the following files from `src/ESP_code/` to the ESP32 using ampy, Thonny, or WebREPL:
  - `boot.py`
  - `main.py`
  - `config.py`
  - `utils.py`
  - `algorithms/OuterLine.py (copy OuterLine.py directly to the ESP32)`
  - `algorithms/AStar.py (copy Astar.py directly to the ESP32)`

### 3. Configure Wi-Fi

- Edit `src/ESP_code/boot.py` and set your Wi-Fi SSID and password.
- On boot, ESP32 will connect to Wi-Fi and print its IP address.

### 4. Webots Setup

- Open Webots and load the e-puck simulation world.
- Place the provided controllers in the appropriate Webots project folders:
  - `Webots_code/OuterLine_webots.py`
  - `Webots_code/AStar_webots.py`
- Adjust the serial port in `OuterLine_webots.py` (e.g., `COM7`) to match your system.
- For A* path planning, set the ESP32 IP address in `AStar_webots.py` (`HOST = '...'`).

### 5. Install Python Dependencies (on PC)

```bash
pip install numpy pyserial
```

### 6. Running the Experiment

#### a. Start ESP32

- Reset or power on the ESP32.
- Wait for Wi-Fi connection (check serial output for confirmation).

#### b. Start Webots

- Run the desired controller:
  - For line following: `OuterLine_webots.py`
  - For A* path planning: `AStar_webots.py`

#### c. Select Algorithm

- On ESP32, press:
  - **Left button**: Start Outer Line Follower
  - **Right button**: Start A* Path Planner

#### d. Observe

- The robot will follow the line or navigate using A* path planning.
- Debug output is available via serial (ESP32) and Webots console.

## File Structure

```
src/
  ESP_code/
    boot.py                # Wi-Fi setup for ESP32
    main.py                # Entry point for ESP32
    config.py              # Pin and map configuration
    utils.py               # Utility functions and PID
    algorithms/
      OuterLine.py         # Outer line following logic
      AStar.py             # A* path planning logic
  Webots_code/
    OuterLine_webots.py    # Webots controller for line following
    AStar_webots.py        # Webots controller for A* path planning
```

## Reproducing the Main Experiment

1. **Connect ESP32 to Wi-Fi** (see serial output for confirmation).
2. **Start Webots** and run the appropriate controller.
3. **Press the button** on ESP32 to select the algorithm.
4. **Observe robot behavior** in Webots simulation.

## Notes

- Ensure only one program accesses the ESP32 serial port at a time (close Thonny before running Webots).
- Adjust pin numbers and serial ports as needed for your hardware.
- For troubleshooting, check serial output from ESP32 and Webots console logs.

## License

This project is for educational use as part of the "Robotics and Python" assignment.
