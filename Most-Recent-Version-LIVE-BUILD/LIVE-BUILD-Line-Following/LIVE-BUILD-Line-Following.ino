/**
 * @file    M2-Line-Following-V1.ino
 * @brief   Implements a state machine for the car with IDLE, DRIVE, and CHARGE states.
 * @details Line following implemented in DRIVE state. State changes done via terminal.
 * @author  Eric Soo-Yoshimura & Ben Olenick
 * @date    Due on March 31st 2025
 */
 
// Library imports:
#include <Wire.h>                 // I2C
#include <Adafruit_INA219.h>      // DMM sensor
#include <HUSKYLENS.h>            // Camera
#include <BluetoothSerial.h>      // BT communication
#include <esp_timer.h>            // ESP32 timer
 
// INA219 instance:
Adafruit_INA219 ina219;
 
// HUSKYLENS instance:
HUSKYLENS huskylens;
int ID1 = 1;                      // Required parameter to pull data from camera
 
// FSM parameters:
enum State { IDLE, DRIVE, CHARGE };
State currentState = IDLE;
 
// Constants:
const int steeringPin = 32;
const int speedMotorPin = 33;
const int pwmFrequency = 50;
const int pwmResolution = 12;
const int pwmMax = 4095;
 
const float minPulseWidth = 1.0; // 1ms pulse width for 0 degrees
const float maxPulseWidth = 2.0; // 2ms pulse width for 180 degrees
const float periodMs = 20.0;
 
const int baudRate = 115200;
 
const int STRAIGHT = 90;
const int IDLE_SPEED = 0;
const int MIN_SPEED = 45;
 
// Default values:
int steeringAngle = STRAIGHT;   // start centered
int motorSpeed = IDLE_SPEED;    // start centered
 
// Control variables
int setpoint = 160;             // Diplay centerline position
int error = 0;                  // Feedback variable
int kp = 0.6;                   // Experimental value for proportional controller
 
//***************************** Begin Bluetooth Config
// Device Name (For Bluetooth):
String deviceName = "Blue!!!";
BluetoothSerial SerialBT; //renaming BluetoothSerial to SerialBT for so it reads better
 
// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
 
// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif
 
//Misc debugging values for BT. Not important for functionality
int testingNumber = 0;
char str[50];

bool isPaired = false;
bool wasPaired = false;
//***************************** End Bluetooth Config
 
// Timer handle
esp_timer_handle_t stateTimer;
esp_timer_handle_t INA219BT_timer;

// For Servo Turn Radius Debugging
String angleBuffer = "";

// Variables for cumulative power consumed during run
int totalVoltage = 0;
int totalCurrent = 0;
int totalPower = 0;
float totalTimeElapsed = 0.0;
float totalRunTimeElapsed = 0.0;
State previousState = IDLE;

bool resetUI = false; //used in deciding if we need to resend the UI interface

char inputBuffer[200];  // Buffer to hold the input
int inputIndex = 0;

// Defining our Functions prior to setup()
// void setSteeringAngle(int angle);
// void setMotorSpeed(int speed);
// void printState(void* arg);
// void readINA219BT(void* arg);
// void loadTimer(int time);
// void timerLog(int time);
 
//******************************************** Begining of Function-Definitions

/**
 * @brief Initializes and starts a periodic timer to execute a callback function.
 * 
 * This function sets up a timer that periodically triggers the specified callback 
 * function, allowing for repeated execution at defined intervals.
 * 
 * @param time The interval in microseconds at which the timer should trigger the callback.
 *             This determines how frequently the callback function is executed.
 * 
 * @return None
 */
void loadTimer(int time) {
  const esp_timer_create_args_t stateTimerArgs = {
    .callback = &printState, // Function to call on timer expiration
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "state_timer"
  };
  esp_timer_create(&stateTimerArgs, &stateTimer);
  esp_timer_start_periodic(stateTimer, time); // time in microseconds
}


/**
 * @brief Sets up a periodic timer to log INA219 sensor data via Bluetooth.
 * 
 * This function creates and starts a periodic timer that triggers the 
 * readINA219BT function at specified intervals to log voltage, current, 
 * and power data from the INA219 sensor over Bluetooth.
 * 
 * @param time The interval in microseconds at which the timer should trigger.
 *             This determines how frequently the INA219 data is logged.
 * 
 * @return None
 */
void timerLog(int time) {
  const esp_timer_create_args_t INA219BTTimerArgs = {
    .callback = &readINA219BT,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "INA219BT_timer"
  };
  esp_timer_create(&INA219BTTimerArgs, &INA219BT_timer);
  esp_timer_start_periodic(INA219BT_timer, time);
}


// Timer callback function, prints the current state every 5 seconds
void printState(void* arg) {
    // switch (currentState) {
    //     case IDLE:
    //         Serial.println("State: IDLE");
    //         SerialBT.println("State: IDLE");
    //         break;
    //     case DRIVE:
    //         Serial.println("State: DRIVE");
    //         SerialBT.println("State: DRIVE");
    //         break;
    //     case CHARGE:
    //         Serial.println("State: CHARGE");
    //         SerialBT.println("State: CHARGE");
    //         break;
    // }
}

/**
 * @brief Follows the line using the HuskyLens sensor.
 */
void followLine() {
    huskylens.request(ID1);
    HUSKYLENSResult result = huskylens.read();
    int error = (int32_t)result.xTarget - setpoint;
   
    if (abs(error) > 40) {
        setMotorSpeed(motorSpeed);
    } else {
        setMotorSpeed(motorSpeed + 20);
    }
    setSteeringAngle(STRAIGHT - 0.59 * error); // The factor of 0.62 was found via experimentation.
                                              // It allows for the steering servo to turn tight enough on the corners while
                                              // not oscillating to much on the straights
}
 
/**
 * @brief Handles incoming serial communication commands.
 */
void handleSerialCommunication() {
  bluetoothPairedCheck(); // Check if Bluetooth is connected and update connection status
  
  // Used for Serial Communication
  //*************************************
  // if (Serial.available()) { 
  //   char command = Serial.read();
  //   SerialBT.write(command);
  //   processCommand(tolower(command));
  // }  
  //*************************************

  if (SerialBT.available()) {
    char command = SerialBT.read();  

    resetUI = true;
    UIimplementation(command);
  }
}

void UIimplementation(char command) {

  if (resetUI) {
    resetUI = false; // Prevent the UI from constantly displaying the same information
    SerialBT.println("Press key to change state...");
    SerialBT.println("D to Drive");
    SerialBT.println("I to Idle");
    SerialBT.println("C to Charge");
    SerialBT.println("Speed to change Speed");
    SerialBT.println("EC to change the Error Constant");
    SerialBT.println("Input From User: ");
    inputFromUser(command);

    SerialBT.println("-------------------------------");
  }
}

void inputFromUser(char command) {

  if (command != '\n') { // if ENTER is not pressed
    inputBuffer[inputIndex++] = command; // Add pressed key to the buffer
    
    printOutStoredArray(inputBuffer);
  }
  else if (command == '\b' || command == 127) { // BACKSPACE or DELETE
      if (inputIndex > 0) {
        inputIndex--;                     
        inputBuffer[inputIndex] = '\0';   // remove last char

        printOutStoredArray(inputBuffer);
      }
  }

  else { // ENTER is pressed
    inputBuffer[inputIndex] = '\0'; // put Null-Term at end of buffer

    //Turn the entire buffer to lowercase: We do this (instead of turnning each individual letter to lower case) because we use a pointer for processCommand(). As tolower() returns the 
    //ASCI value of the char, we can't give processCommand() a pointer to an int instead of a char, so we have to put the raw, typed in value first, then change it to lower case
    for (int i = 0; i < sizeof(inputBuffer); i++ ) {
      inputBuffer[i] = (char)tolower(inputBuffer[i]);
    }

    processCommand(inputBuffer);

    // Reset the buffer
    for (int i = 0; i < inputIndex; i++) {
      inputBuffer[i] = '/0';
    }
    inputIndex = 0;
  }

}

void printOutStoredArray(char *inputBuffer) {
  SerialBT.print(inputBuffer);
  SerialBT.println(' ');
}

void bluetoothPairedCheck() {
  isPaired = SerialBT.hasClient();
  if (isPaired &&!wasPaired) {
    SerialBT.println("Paired with a device!");
    Serial.println("Paired with a device!");
    wasPaired = true;
  } else if (!isPaired && wasPaired) {
    SerialBT.println("Lost connection with a device.");
    Serial.println("Lost connection with a device.");
    wasPaired = false;
  }
}

/**
  *
  * @brief Allows for manual changing of servo via the serial monitor
  * @param command The character recieved from the serial monitor
  *
  */
void serialChangeServoAngle(char command) {
  if (isDigit(command)) {
    angleBuffer += command;
  } else if (command == '\n' || command == ' ') { // End of number
    if (angleBuffer.length() > 0) { // While buffer isnt empty
      int angle = angleBuffer.toInt(); // Convert to integer
      setSteeringAngle(angle);

      Serial.print("Set steering angle to: ");
      Serial.println(angle);

      angleBuffer = ""; // Clear buffer
    }
  }
}
 
/**
 * @brief Processes command input and changes state accordingly.
 * @param command The command character received.
 */
void processCommand(char* command) {
  if (strncmp(command, "i", 1) == 0) { // If command is i
    currentState = IDLE;
  }
  else if (strncmp(command, "d", 1) == 0) {
    currentState = DRIVE;
  }
  else if (strncmp(command, "i", 1) == 0) {
    currentState = CHARGE;
  }
  else if (strncmp(command, "speed ", 6) == 0) {
    int newSpeed = atoi(command + 6); // skips first 6 characters then turns the rest into speed
  }
  else if ((strncmp(command, "EC ", 3) == 0)) {

  }
}
 
/**
 * @brief Inputs a steering angle and sets the steering lock accordingly via PWM signal.
 * @param angle is the target steering angle, where 0 is is fully left and 180 is fully right.
 */
void setSteeringAngle(int angle) { 
    angle = constrain(angle, 0, 205);
    float rangeMs = maxPulseWidth - minPulseWidth;
    float pulseWidthMs = minPulseWidth + ((float)angle / 180.0) * rangeMs;
    int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
    ledcWrite(steeringPin, dutyCycle);
}
 
/**
 * @brief Inputs a motor speed and sets it accordingly via PWM signal.
 * @param speed is the target speed, where 0 is is stopped and 180 is max speed.
 */
void setMotorSpeed(int speed) {
    speed = constrain(speed, 0, 120);
    float rangeMs = maxPulseWidth - minPulseWidth;
    float pulseWidthMs = minPulseWidth + ((float)speed / 180.0) * rangeMs;
    int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
    ledcWrite(speedMotorPin, dutyCycle);
}
 
/**
 * @brief Outputs the car's bus voltage, current and power via I2C to the serial monitor.
 */
void readINA219() {
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
   
    Serial.print("Bus Voltage: "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power: "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("----------------------");
}
 
/**
 * @brief Outputs the car's bus voltage, current and power via BT to the terminal.
 */
void readINA219BT(void* arg) {
    // float busVoltage = ina219.getBusVoltage_V();
    // float current_mA = ina219.getCurrent_mA();
    // float power_mW = ina219.getPower_mW();
    // totalTimeElapsed += 0.5; // As we log every 0.5 seconds, increment by 0.5 seconds
    
    // if (currentState != IDLE) {
    //   totalVoltage += busVoltage;
    //   totalCurrent += current_mA;
    //   totalPower += power_mW;
    //   totalRunTimeElapsed += 0.5; // As we log every 0.5 seconds, increment by 0.5 seconds
    //   previousState = currentState; 
    // }
    // else if ((currentState == IDLE) && (previousState != IDLE)) { //only want to print this out once when state changes back to IDLE
    //   // SerialBT.print("Voltage Total During Run: ");
    //   // SerialBT.print(totalVoltage);
    //   // SerialBT.print(", ");
    //   // SerialBT.print("Current Total During Run: ");
    //   // SerialBT.print(totalCurrent);
    //   // SerialBT.print(", ");
    //   SerialBT.print("Power Total During Run: ");
    //   SerialBT.print(totalPower);
    //   SerialBT.print(", ");
    //   SerialBT.print("Time Elapsed: ");
    //   SerialBT.print(totalRunTimeElapsed);
    //   SerialBT.print(", ");
    //   SerialBT.print("Energy Total During Run: ");
    //   SerialBT.println(totalPower * totalRunTimeElapsed); // Energy = Power * Time
      
    //   totalVoltage = 0;
    //   totalCurrent = 0;
    //   totalPower = 0;
    //   totalRunTimeElapsed = 0;
    //   previousState = currentState; // Reset previous state to IDLE
    // }
   
    // SerialBT.print("Voltage: ");
    // SerialBT.print(busVoltage);
    // SerialBT.print(", ");
    // SerialBT.print("Current: ");
    // SerialBT.print(current_mA);
    // SerialBT.print(", ");
    // SerialBT.print("Power: "); 
    // SerialBT.print(power_mW);
    // SerialBT.print(", ");
    // SerialBT.print("Time: ");
    // SerialBT.println(totalTimeElapsed);
    // SerialBT.print("Time: ");
    // SerialBT.println(totalTimeElapsed);
}
 
/**
 * @brief Outputs camera data to the serial monitor.
 * @param result is the HUSKYLENS object with camera data.
 */
void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
    //delay(1000);
}

/*
  Beginning of kernal
*/
void setup() {
    Serial.begin(baudRate);
    Wire.begin();                 //For I2C communication
    SerialBT.begin(deviceName);   //Start SerialBT
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());
   
    // INA219
    if(!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while(1);
    }
    Serial.println("INA219 Initialized");
   
    // HUSKYLENS Initilization
    while(!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
    Serial.println(F("HUSKYLENS Initialized"));
    huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
 
    // Steering initialization:
    ledcAttach(steeringPin, pwmFrequency, pwmResolution);
    setSteeringAngle(steeringAngle);
 
    // Motor initialization:
    ledcAttach(speedMotorPin, pwmFrequency, pwmResolution);
    delay(1000); // at least 1s delay to allow driver to recognize servo attachment
    setMotorSpeed(motorSpeed); // the driver needs to first see 0-speed in order to calibrate
    delay(3000); // this delay may need to be up to 5s (5000ms) for the driver to calibrate to 0-speed
    motorSpeed = 70; // set a default motor speed
 
    // Timer Value setup
    loadTimer(5000000); //5 seconds in microseconds
 
    // Timer setup for logging every 0.5s (500,000 microseconds)
    timerLog(500000);
}
 
void loop() {
    handleSerialCommunication();
   
    switch (currentState) {
        case IDLE: // Set to IDLE at beginning and end of run
            setMotorSpeed(IDLE_SPEED);
            setSteeringAngle(STRAIGHT);
            break;
 
        case DRIVE:
            followLine();
            break;
 
        case CHARGE:
            setMotorSpeed(IDLE_SPEED);
            setSteeringAngle(STRAIGHT);
            break;
    }
}
