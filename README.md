# Ohmigos Arduino ReadMe

### Overview
Ohmigos is an Arduino-based autonomous robot with Bluetooth-controlled modes for driving, charging, and idling. It features real-time power monitoring via the INA219 sensor, uses a PID-based line-following algorithm, and is built around a state machine for robust task management.

### Directory Structure
- **Images:** Location of the images used in this ReadMe.
- **Libraries:** Location of all the extra libraries used throughout the project.
- **Most-Recent-Version-LIVE-BUILD:** Location of the most up to date version of the software.
- **Previous-Code-Versions:** Location of all the previous versions of our code that led up to the LIVE code. Useful for seeing progess or rolling back software.
- **Autonomous_Vehicle_Guide:** A guide to provide a high-level overview about ***how*** the project works.

### Features
- **Autonomous Driving**: Implements self-driving via image processing of a reference target path and a simple control algorithm
- **Bluetooth**: Has dual direction communication using the ESP32's Bluetooth
- **INA219**: Reads and displays voltage, current, and power draw at any given time
- **Motor Control**: Provides PWM-based precise speed control
- **Steering Control**: Provides PWM-based precise turning control, which is used by the autonomous line following algorithm

## System Architecture
- The vehicle is implemented as a "state machine"
   - A state machine is like a flowchart that shows how something changes based on different actions. It has states and rules for moving between them. For example, think about a vending machine: it starts in a "waiting for money" state. When you put in $1, it moves to a "has $1" state. If you add another $1, it goes to a "has $2" state. Once you have enough money and pick a snack, it gives you the snack and goes back to "waiting for money." Every time you put money in or make a choice, you're causing the machine to change its state!
- The car is the same way: we have "DRIVE", "IDLE" and "CHARGE" states, and through the user interface, you can change the state the car is in.

## How to Use
1. **Setup**:
   - Ensure that the vehicle electronics are hooked up to the power supply.
   - Check that there are no loose screws, and that everything is snugly fit to the vehicle.
   - Flash the code onto the ESP32 using the Arduino IDE, making sure that the switch for the power supply is turned OFF: the car is powered over USB for programming.

2. **Bluetooth Pairing**:
   - Pair the ESP32 with your laptop. Turn on bluetooth, then turn on the vehicle and find a device name `Blue!!!` and connect to it.
   - Open device manager on your laptop, go to "Ports" and find the "Standard Serial over Bluetooth link (COMX)." There should be two COM ports a distance of 1 away from eachother (e.g COMX & COM(X+1))
   - Open a serial terminal or Bluetooth terminal application, such as TeraTerm, and create a connection to the larger of the two COM ports (COM(X+1)) that were shown in device manager.
   - Press a key on the laptops keyboard. The UI will appear.  
   *NOTE:* If nothing appears, try connecting to the smaller COMX port instead.

3. **Commandline User Interface**:
   - Use TeraTerm serial terminal to interact with the ESP32.  
   *NOTE:* A key must be pressed for the UI to appear once connected. Once the full UI is
   displayed, it is recommended to resize the terminal so that only one instant of the UI
   is shown at a time.
   - <ins>Input From User:</ins>  
   This is where you will see your input to the device. It acts as an array of `char` until 
   ENTER is pressed.
   - <ins>Commands (not case sensitive):</ins>  
   This is where you will the avaliable control of the vehicle. The text within [] is the 
   sequence of `char` that need to be typed for the vehicle to recongize the command. The 
   instructions to use each command are listed in the UI.  
   *NOTE:* While the example commands are capitilized, the typed commands do not need to be, as the commands are not case sensitive.
   - <ins>Parameters:</ins>  
   This is where all the variable parameters for the vehicle are displayed.  
   *NOTE:* The list only updates once a key is pressed.  
   *NOTE:* The SPEED parameter will be set to 60 as long as the batter voltage is above a
   certain level. This is to prevent the vehicle from going to fast with a fresh battery.
   The speed may be increased at user discretion after enough energy was sufficiently used.
   - <ins>Data & Clear Data:</ins>  
   These commands are used to display the collected data (or to dispose of the data for a new run). When DATA is typed, a two collum table will display with the collected voltage and current values from the run.  
   *More information on Data collection:* The car will begin to log the bus voltage from the first transition into DRIVE (from IDLE), through the CHARGE phase, until the car is set back to idle. The data is stored as two separate arrays: one for voltage over time, another for the state over time. The indices are equivalent to 0.5s time steps, and thus can be useful for plotting the change in energy over a competition run.
   - Appearance: ![alt text](https://github.com/Rose-Hulman-ECE-Junior-Design/Sec01-Team02-GMOS/blob/main/Images/PresentUI.png "Our actual UI will appear here once completed")
 4. **Reconnecting After a Disconnection** :
   - Power cycle the ESP32
   - Disconnect from TeraTerm (File->Disconnect)
   - Create a new connection to the same COM port (File-New Connection)
   - Press a key to refresh the UI
<!-- 
## Documentation Plan (not offical documentation yet)

### Introduction
- The purpose of the <ins>Ohm</ins>igos robot and its relationship to ECE technology.
  - Overview of how the competition integrates electrical and computer engineering concepts like sensor data acquisition, motor control, and communication protocols.
  - Brief description of how the robot demonstrates core ECE skills, like embedded systems, power management, communication interfaces.
      - power tracking
      - circuits
      - object oriented code
      - etc

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
   - Required software (e.g., Arduino IDE, libraries such as `Wire.h`, `Adafruit_INA219.h`).
2. **Setup**:
   - Step-by-step guide for wiring the robot components.
   - Instructions for downloading, modifying, and flashing the code.
3. **Execution**:
   - Pairing the robot with a Bluetooth device.
   - Operating the robot using Tera Term.

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
         - Will most likely be parsing the incoming BT message for the data we want (from the robot), then accumulating the total power consumed as well as total power recharged (can differentiate via state of robot) and outputting the data to the serial monitor
  - **Bluetooth Communication**:
    - Functions for sending and receiving messages over Bluetooth.
- **Adding New Programs**:
  - How to extend the codebase to include new features or control options.

### Educational Features
- Future additions to make the robot accessible to users with little experience in coding and to allow them to learn what parts of the code does
  - **Pre-Built Modes**: Demonstration programs for obstacle avoidance, line-following, etc.

### FAQ Troubleshooting
- Common issues during setup or operation:
  - **Bluetooth Pairing**: Solutions for pairing failures.
  - **Motor Control Issues**: Diagnosing problems with speed or turning control.

### User Interface
- The Serial Monitor in Arduino will act as the UI
      - Will print out all necessary information (power, car state, etc) at pre-determined intervals
      - Will use the terminal to start/stop vehicle as well as update existing parameters
--- -->
