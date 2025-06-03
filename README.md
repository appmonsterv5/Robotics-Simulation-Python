# Robotics Simulation Python

This repository contains code for simulating and controlling a mobile robot (e-puck) using Webots and an ESP32 microcontroller. The project demonstrates hardware-in-the-loop (HIL) simulation, line following, and A* path planning.

## Project Structure

- `src/Webots_code/`: Webots controllers for simulation.
  - `AStar_webots.py`: Webots controller for A* path planning with TCP communication to ESP32.
  - `OuterLine_webots.py`: Webots controller for simple line following with serial communication.
- `src/algorithms/`: Algorithms running on the ESP32.
  - `AStar.py`: A* path planning and navigation logic for ESP32.
  - `OuterLine.py`: Outer line following state machine for ESP32.
- `src/`: Core configuration and utility files.
  - `main.py`: Entry point for ESP32, selects algorithm based on button press.
  - `config.py`: Hardware pin configuration and map definitions.
  - `utils.py`: Utility functions, PID controller, and A* implementation.
  - `boot.py`: ESP32 boot script for Wi-Fi connection.
- `.gitattributes`: Git settings.

## Requirements

- **Webots** (tested on R2023a)
- **Python 3.10+** for Webots controllers
- **MicroPython** (with `ulab` and `usocket` modules) for ESP32
- **ESP32** development board

## Setup

### 1. Webots Simulation

- Open Webots and load your world with the e-puck robot.
- Assign the appropriate controller (`AStar_webots.py` or `OuterLine_webots.py`) to the robot.
- Adjust the serial port in `OuterLine_webots.py` if needed (e.g., `COM7`).

### 2. ESP32 Firmware

- Flash MicroPython firmware to your ESP32.
- Copy the contents of `src/` (excluding `Webots_code/`) to the ESP32.
- Edit `boot.py` to set your Wi-Fi SSID and password.
- On boot, ESP32 will connect to Wi-Fi and wait for a button press to start either the line follower or A* algorithm.

### 3. Running the System

- Start Webots simulation.
- Press the left button on ESP32 for line following, or the right button for A* path planning.
- For A*, ensure both ESP32 and Webots are on the same network and the IP/port match in `AStar_webots.py` and `AStar.py`.

## Usage

- **Line Following:** Uses ground sensors and a simple state machine to follow a line. Communication is via serial.
- **A* Path Planning:** Computes a path on a grid and navigates using PID control. Communication is via TCP sockets.

## Troubleshooting

- Ensure only one program accesses the ESP32 serial port at a time (close Thonny before running Webots).
- If Wi-Fi connection fails, check credentials in `boot.py`.
- For TCP, verify ESP32 IP address and port.

## License

MIT License

---

**Author:** R. Kleine  
**Last update:** June 2025
