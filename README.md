# Ohmigos Arduino ReadMe

## Milestone 1

### Code Files for Milestone
**M1-Basic-Software-Functionality.ino**

### Features
- **Bluetooth**: Has dual direction communication using the ESP32's Bluetooth
- **INA219**: Reads and displays voltage, current, and power draw at any given time
- **Motor Control**: Provides PWM-based precise speed control
- **Steering Control**: Provides PWM-based precise turning control

## How to Use
1. **Setup**:
   - Connect the motor, servo, INA219 sensor, and ESP32 according to their datasheets and this project's circuit design.
   - Flash the code onto the ESP32 using the Arduino IDE.

2. **Bluetooth Pairing**:
   - Pair the ESP32 with your Bluetooth device using the name `ECE362CarTeam02`.
   - Open a serial terminal or Bluetooth terminal application to send and receive messages.

3. **Operation**:
   - Use the serial terminal to interact with the ESP32.
   - Monitor sensor data and adjust motor speed and steering angle through the provided interfaces.

---

## Documentation Plan

### Introduction
- Explain the purpose of the Ohmigos robot and its relationship to ECE technology.
  - Overview of how the competition integrates electrical and computer engineering concepts like sensor data acquisition, motor control, and communication protocols.
  - Brief description of how the robot demonstrates core ECE skills, such as embedded systems, power management, and communication interfaces.

### UI User Manual
- **Bluetooth Control**:
  - How to connect to the robot over Bluetooth.
  - Command set for interacting with the robot (e.g., change state, speed, etc.).
- **LED Array**:
  - Interpreting the battery charge status.
  - LED indications for robot states (idle, drive, charge).
- **Adjustable Speed**:
  - Instructions for modifying the robot's speed using Bluetooth.

### How to Get and Use the Software
1. **Prerequisites**:
   - Required hardware components (list with specifications).
   - Required software (e.g., Arduino IDE, specific libraries like `Wire.h`, `Adafruit_INA219.h`).
2. **Setup**:
   - Step-by-step guide for wiring the robot components.
   - Instructions for downloading, modifying, and flashing the code.
3. **Execution**:
   - Pairing the robot with a Bluetooth device.
   - Operating the robot using a serial terminal or custom interface.

### API Documentation
- **Overview**:
  - Description of functions provided in the code for motor and steering control, INA219 data reading, and Bluetooth communication.
- **Function Reference**:
  - **Motor Control**:
    - Function to set motor speed (parameters and usage).
  - **Steering Control**:
    - Function to set servo angle (parameters and usage).
  - **Power Monitoring**:
    - Functions to retrieve voltage, current, and power draw.
  - **Bluetooth Communication**:
    - Functions for sending and receiving messages over Bluetooth.
- **Adding New Programs**:
  - How to extend the codebase to include new features or control options.

### Educational Features (Placeholder)
- Future additions to make the robot accessible to users with little or no experience in coding and robotics:
  - **Graphical Programming Interface**: Drag-and-drop interface to control the robot.
  - **Interactive Tutorials**: Guided lessons to learn about sensors, motors, and communication.
  - **Pre-Built Modes**: Demonstration programs for obstacle avoidance, line-following, etc.
  - **Hands-On Activities**: Tasks to explore sensor functionality, motor control, and more.
  - **Gamification**: Challenges and rewards to engage users and encourage learning.

### Troubleshooting
- Common issues during setup or operation:
  - **Bluetooth Pairing**: Solutions for pairing failures.
  - **Motor Control Issues**: Diagnosing problems with speed or turning control.
  - **INA219 Data Reading**: Ensuring accurate voltage, current, and power measurements.
  - **Robot States**: Resolving inconsistencies in state transitions (idle, drive, charge).

---