
# System Design Documentation

## Project Title
**Hand Gesture Controlled Drone for Topographic Mapping with Photogrammetry**

## Overview
This system integrates hand gesture recognition, autonomous flight control, and aerial mapping using a downward-facing ESP32-CAM mounted on a drone. The entire workflow involves a ground control station (laptop), a telemetry module, a flight controller running INAV firmware, and an ESP32-CAM. Gesture commands are recognized on the laptop and transmitted wirelessly to control the drone and image capture during flight.

---

## Software System Architecture

![Software Architecture](Software_Architecture.jpg)

---

## Subsystems and Modules

### 1. **Flight Controller**
- Firmware: **INAV**
- Role: Executes flight commands sent from the ground control station via telemetry.

### 2. **Telemetry Module (NodeMCU)**
- Firmware: [**MSPWiFiBridge**](https://github.com/fpv-wtf/MSPWiFiBridge)
- Role: Acts as a Wi-Fi Access Point and bridges MSP messages for:
  - Drone control and telemetry data transfer
  - Communication with ESP32-CAM and Ground Control Station

### 3. **Ground Control Station (Laptop)**
- **ESP Cam Control Module**
  - Sends `START` and `STOP` commands to the ESP32-CAM to initiate and terminate image capture
- **Drone Control Module**
  - Uses MSP, MAVLink, or DroneKit to send movement commands to the flight controller
- **Hand Gesture Recognition Module**
  - Detects predefined gestures from webcam input using MediaPipe
- **Hand Gesture Control Module**
  - Maps recognized gestures to drone movement or camera trigger commands

### 4. **ESP32-CAM**
- Firmware: `ESPcamcontrol.ino` (based on Arduino)
- Role:
  - Connects to the NodeMCU access point
  - Listens for capture commands from the ground station
  - Captures and stores or transmits images accordingly

---

## File Overview

| File/Folder              | Description |
|--------------------------|-------------|
| `ESPcamcontrol.ino`      | Firmware for the ESP32-CAM |
| `gesture_control/`       | Python module for gesture recognition and command mapping |
| `msp_bridge/`            | Configuration for NodeMCU running MSPWiFiBridge |
| `drone_control/`         | Scripts for controlling drone via MAVLink/MSP/DroneKit |
| `README.md`              | Project overview and setup instructions |
| `Software_Architecture.jpg` | System architecture image used in documentation |

---

## Acknowledgements

- **MSPWiFiBridge** - The NodeMCU firmware used to bridge telemetry and drone commands: [https://github.com/fpv-wtf/MSPWiFiBridge](https://github.com/fpv-wtf/MSPWiFiBridge)
- **ESPcamcontrol.ino** - Arduino sketch adapted from open-source examples for ESP32-CAM remote capture.

---

## Notes

- The system is under **active development** with stable modules currently in use.
- Logic is modular and will be extended with obstacle avoidance, SLAM, and real-time video processing in future versions.

