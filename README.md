# Ohmigos Arduino ReadMe

## Milestone 1

### Code Files for Milestone
**M1-Basic-Software-Functionality.ino**

### Features
- **Bluetooth**: Has dual direction comunication using the ESP32's bluetooth
- **INA219**: Reads and displays voltage, current, and power draw at any given time
- **Motor Control**: Provides PWM-based precise speed control
- **Steering Control**: Provides PWN-based control precise turning control

## How to Use
1. **Setup**:
   - Connect the motor, servo, INA219 sensor, and ESP32 according to their  datasheets and this project's circuit design.
   - Flash the code onto the ESP32 using the Arduino IDE.

2. **Bluetooth Pairing**:
   - Pair the ESP32 with your Bluetooth device using the name `ECE362CarTeam02`.
   - Open a serial terminal or Bluetooth terminal application to send and receive messages.

3. **Operation**:
   - Use the serial terminal to interact with the ESP32.
   - Monitor sensor data and adjust motor speed and steering angle through the provided interfaces.