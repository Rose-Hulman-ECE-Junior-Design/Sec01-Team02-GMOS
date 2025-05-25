/**
 * @file    M2-Line-Following-V1.ino
 * @brief   Implements a state machine for the car with IDLE, DRIVE, and CHARGE states.
 * @details Line following implemented in DRIVE state. State changes done via terminal.
 * @author  Eric Soo-Yoshimura & Ben Olenick
 * @date    Due on March 31st 2025
 */

//TODO for readme
/*
- Include all the libraries used in the libary folder for easy download
- Instructions on how to connect/reconnect to robot. How to start a new run if the robot disconnects
- Make code more modular. Group variables in header area
- Unknown Bugs (Car randomly just starts turning left until reset)
*/


// Library imports:
#include <Wire.h>            // I2C
#include <Adafruit_INA219.h> // DMM sensor
#include <HUSKYLENS.h>       // Camera
#include <BluetoothSerial.h> // BT communication
#include <esp_timer.h>       // ESP32 timer
#include <PID_v1.h>          // PID controller

// PID controller
// PID tuning parameters
double Kp = 0.65;    // proportional gain 
double Ki = 0.00;    // integral gain     
double Kd = 0.1;    // derivative gain   


double error,    // the error from the camera
       pidOutput,   // the correction to send to the steering servo
       pidSetpoint; // the ideal error (0)

// create the PID object
PID linePid(&error, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// INA219 instance:
Adafruit_INA219 ina219;

// HUSKYLENS instance:
HUSKYLENS huskylens;
int ID1 = 1; // Required parameter to pull data from camera

// FSM parameters:
enum State
{
  IDLE,
  DRIVE,
  CHARGE
};
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
int steeringAngle = STRAIGHT; // start centered
int motorSpeed = IDLE_SPEED;  // start centered

// Control variables
int setpoint = 160; // Diplay centerline position

//***************************** Begin Bluetooth Config
// Device Name (For Bluetooth):
String deviceName = "Blue!!!";
BluetoothSerial SerialBT; // renaming BluetoothSerial to SerialBT for so it reads better

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

// Misc debugging values for BT. Not important for functionality
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



bool resetUI = false; // used in deciding if we need to resend the UI interface

char inputBuffer[200]; // Buffer to hold the input
int inputIndex = 0;

float busVoltage = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;

float initialBatteryVoltage = 0.0;
float initialBatteryCurrent = 0.0;
float initialBatteryPower = 0.0;
float driveBatteryVoltage = 0.0;
float driveBatteryCurrent = 0.0;
float driveBatteryPower = 0.0;
float totalBatteryVoltageUsed = 0.0;
float totalBatteryCurrentUsed = 0.0;
float totalBatteryPowerUsed = 0.0;
float totalEnergyUsed = 0.0;
float batteryChargeVoltage = 0.0;
float batteryChargeCurrent = 0.0;
float batteryChargePower = 0.0;
float FinalBatteryChargeVoltage = 0.0;
float FinalBatteryChargeCurrent = 0.0;
float FinalBatteryChargePower = 0.0;
float totalEnergyGained = 0.0;

float totalElapsedRunTime = 0.0;
bool onceDuringRun = true;
State previousState = IDLE;

int recordedValuesIndex = 0;
int arraySize = 720; // Car should run for 180 seconds (90sec run + 45sec charge + 45sec run). We sample every 0.5sec, so 180 * 2 = 360. Multiply it by 2 to be safe to have enough room.
float voltageArray[720]; 
float currentArray[720];
bool running = false;

double maxPCBVoltage = 10.4;
double minPCBVoltage = 5.7;

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
void loadTimer(int time)
{
  const esp_timer_create_args_t stateTimerArgs = {
      .callback = &printState, // Function to call on timer expiration
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "state_timer"};
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
void timerLog(int time)
{
  const esp_timer_create_args_t INA219BTTimerArgs = {
      .callback = &readINA219BT,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "INA219BT_timer"};
  esp_timer_create(&INA219BTTimerArgs, &INA219BT_timer);
  esp_timer_start_periodic(INA219BT_timer, time);
}

// Timer callback function, prints the current state every 5 seconds
void printState(void *arg)
{
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
void followLine()
{
  huskylens.request(ID1);
  HUSKYLENSResult result = huskylens.read();

  error = (int32_t)result.xTarget - setpoint;
  linePid.SetTunings(Kp, Ki, Kd); // update the kp ki and kd values in the controller
  linePid.Compute(); // calculate the output
  setSteeringAngle(STRAIGHT + (int)pidOutput);

  adjustSpeed();

}

void adjustSpeed() 
{
  if (abs(error) > 40) // Slow down on curves
  {
    setMotorSpeed(motorSpeed);
  }
  else
  {
    setMotorSpeed(motorSpeed + 20);
  }

  // We need to dynamically change the speed of the car based on the voltage being read from the INA219, as 
  // if we go to fast due to a high voltage, we will run off the track, so we force the robot to go slower. minPCBVoltage/5 = 1.14
  if (busVoltage > (maxPCBVoltage - minPCBVoltage/5))
  {
    motorSpeed = 60;
  }

}

/**
 * @brief Handles incoming serial communication commands.
 */
void handleSerialCommunication()
{
  bluetoothPairedCheck(); // Check if Bluetooth is connected and update connection status

  // Used for Serial Communication
  //*************************************
  // if (Serial.available()) {
  //   char command = Serial.read();
  //   SerialBT.write(command);
  //   processCommand(tolower(command));
  // }
  //*************************************

  if (SerialBT.available())
  {
    char command = SerialBT.read();

    UIimplementation(command);
  }
}

void UIimplementation(char command)
{

  SerialBT.println("Input From User: ");
  inputFromUser(command);
  SerialBT.println(" ");
  SerialBT.println("Press ENTER to confirm...");
  SerialBT.println(" ");
  SerialBT.println("Commands (not case sensitive): ");
  SerialBT.println("Type [D] to Drive");
  SerialBT.println("Type [I] to Idle");
  SerialBT.println("Type [C] to Charge");
  SerialBT.println(" ");
  SerialBT.println("Type [Speed X] to change Speed, where X is an integer between 45 -> 120. DEFAULT: 70");
  SerialBT.println(" ");
  SerialBT.println("Type [Kp X] to change the proportional constant, where X is a non negative number. DEFAULT: 0.65");
  SerialBT.println(" ");
  SerialBT.println("Type [Ki X] to change the integral constant, where X is a non negative number. DEFAULT: 0");
  SerialBT.println(" ");
  SerialBT.println("Type [Kd X] to change the derivative constant, where X is a non negative number. DEFAULT: 0.1");
  SerialBT.println(" ");
  SerialBT.println("Parameters: ");
  SerialBT.print("Current State: ");
  SerialBT.println(stateName(currentState));
  SerialBT.print("Speed: ");
  SerialBT.println(motorSpeed);
  SerialBT.print("Kp: ");
  SerialBT.println(Kp, 6); // print to 6 decimal places
  SerialBT.print("Ki: ");
  SerialBT.println(Ki, 6);
  SerialBT.print("Kd: ");
  SerialBT.println(Kd, 6);
  SerialBT.println(" ");
  SerialBT.println("Type [Data] to collect the data from the run"); // TODO: Data collection. Implement clear data command
  SerialBT.println("Type [Clear Data] to clear the data from the previous run (does not need to be done if vehicle is reset)");
  SerialBT.println(" ");
  SerialBT.println("ERROR:");
  SerialBT.println(error,6);
  SerialBT.println("PIDoutput");
  SerialBT.println(pidOutput);
  SerialBT.println("-------------------------------");
}

const char *stateName(State s)
{
  switch (s)
  {
  case IDLE:
    return "IDLE";
  case DRIVE:
    return "DRIVE";
  case CHARGE:
    return "CHARGE";
  default:
    return "UNKNOWN";
  }
}

void inputFromUser(char command)
{

  if (command == 0x0D) // ENTER is pressed
  {                                 
    inputBuffer[inputIndex] = '\0'; // put Null-Term at end of buffer

    // Turn the entire buffer to lowercase: We do this (instead of turnning each individual letter to lower case) because we use a pointer for processCommand(). As tolower() returns the
    // ASCI value of the char, we can't give processCommand() a pointer to an int instead of a char, so we have to put the raw, typed in value first, then change it to lower case
    for (int i = 0; i < sizeof(inputBuffer); i++)
    {
      inputBuffer[i] = (char)tolower(inputBuffer[i]);
    }

    processCommand(inputBuffer);

    memset(inputBuffer, 0, sizeof(inputBuffer)); // fill ALL 200 bytes with '\0'
    inputIndex = 0;
  }

  else if (command == '\b' || command == 127)
  { // BACKSPACE or DELETE
    if (inputIndex > 0)
    {
      inputIndex--;
      inputBuffer[inputIndex] = '\0';
    }

    printOutStoredArray(inputBuffer);
  }

  else // any key is pressed
  {                                      
    inputBuffer[inputIndex++] = command; // Add pressed key to the buffer

    printOutStoredArray(inputBuffer);
  }
}

void printOutStoredArray(char *inputBuffer)
{
  SerialBT.print(inputBuffer);
  SerialBT.println(' ');
}

void bluetoothPairedCheck()
{
  isPaired = SerialBT.hasClient();
  if (isPaired && !wasPaired)
  {
    SerialBT.println("Paired with a device!");
    Serial.println("Paired with a device!");
    wasPaired = true;
  }
  else if (!isPaired && wasPaired)
  {
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
void serialChangeServoAngle(char command)
{
  if (isDigit(command))
  {
    angleBuffer += command;
  }
  else if (command == '\n' || command == ' ')
  { // End of number
    if (angleBuffer.length() > 0)
    {                                  // While buffer isnt empty
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
void processCommand(char *command)
{
  if ((strncmp(command, "i", 1) == 0) && (inputIndex == 1))
  { // If command is i and only i
    currentState = IDLE;
  }
  else if ((strncmp(command, "d", 1) == 0) && (inputIndex == 1))
  {
    currentState = DRIVE;
  }
  else if ((strncmp(command, "c", 1) == 0) && (inputIndex == 1))
  {
    currentState = CHARGE;
  }
  else if (strncmp(command, "speed ", 6) == 0)
  {
    int newSpeed = atoi(command + 6); // skips first 6 characters then turns the rest into speed
    if (newSpeed <= 120 && newSpeed >= 45)
    {
      motorSpeed = newSpeed;
    }
    else
    {
      SerialBT.print("Please Pick a speed between 45 & 120");
    }
  }
  else if ((strncmp(command, "kp ", 3) == 0))
  {
    float newValue = atof(command + 3);

    if (newValue >= 0)
    {
      Kp = newValue;
    }
    else
    {
      SerialBT.print("Cannot do a negative Kp");
    }
  }
  else if ((strncmp(command, "ki ", 3) == 0))
  {
    float newValue = atof(command + 3);

    if (newValue >= 0)
    {
      Ki = newValue;
    }
    else
    {
      SerialBT.print("Cannot do a negative Ki");
    }
  }
    else if ((strncmp(command, "kd ", 3) == 0))
  {
    float newValue = atof(command + 3);

    if (newValue >= 0)
    {
      Kd = newValue;
    }
    else
    {
      SerialBT.print("Cannot do a negative Kd");
    }
  }
  else if ((strncmp(command, "data", 4) == 0))
  {
    SerialBT.print("Voltage: ");
    for (int i = 0; i < recordedValuesIndex; i++) 
    {
      SerialBT.print (voltageArray[i]); 
      if (i == recordedValuesIndex - 1) SerialBT.print(", ");
    }
    SerialBT.println(" ");
    SerialBT.print("Current: ");
    for (int i = 0; i < recordedValuesIndex; i++) 
    {
      SerialBT.print (currentArray[i]); 
      if (i == recordedValuesIndex - 1) SerialBT.print(", ");
    }
  }
  else if ((strncmp(command, "clear data", 10) == 0))
  {
    memset(voltageArray, 0, sizeof(voltageArray)); // fill ALL with '\0'
    memset(currentArray, 0, sizeof(currentArray)); // fill ALL with '\0'
    recordedValuesIndex = 0;
  }
}

/**
 * @brief Inputs a steering angle and sets the steering lock accordingly via PWM signal.
 * @param angle is the target steering angle, where 0 is is fully left and 180 is fully right.
 */
void setSteeringAngle(int angle)
{
  angle = constrain(angle, -25, 195); // Dont let the servo burn itself out by turning to far
  float rangeMs = maxPulseWidth - minPulseWidth;
  float pulseWidthMs = minPulseWidth + ((float)angle / 180.0) * rangeMs;
  int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
  ledcWrite(steeringPin, dutyCycle);
}

/**
 * @brief Inputs a motor speed and sets it accordingly via PWM signal.
 * @param speed is the target speed, where 0 is is stopped and 180 is max speed.
 */
void setMotorSpeed(int speed)
{
  speed = constrain(speed, 0, 120);
  float rangeMs = maxPulseWidth - minPulseWidth;
  float pulseWidthMs = minPulseWidth + ((float)speed / 180.0) * rangeMs;
  int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
  ledcWrite(speedMotorPin, dutyCycle);
}

/**
 * @brief Updates variable to record the car's bus voltage, current and power via I2C to the serial monitor.
 */
void readINA219()
{
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
}

/**
 * @brief Outputs the car's bus voltage, current and power via BT to the terminal.
 */
void readINA219BT(void *arg)
{
  readINA219(); // Update Variables

  /*
  IDLE -> DRIVE -> CHARGE -> DRIVE -> IDLE
  */
  if ((currentState != IDLE) && (onceDuringRun == true)) // When the car starts driving, record the initial battery parameters
  {
    initialBatteryVoltage = busVoltage;
    initialBatteryCurrent = current_mA;
    initialBatteryPower = power_mW;

    // Set everything to 0 and start recording values
    onceDuringRun = false;
    running = true;
    totalElapsedRunTime = 0.0;
    recordedValuesIndex = 0;
  }
  else if (currentState == DRIVE) // Once we start driving from a stop
  {
    // Save current Bat Voltage and stuff
    driveBatteryVoltage = busVoltage;
    driveBatteryCurrent = current_mA;
    driveBatteryPower = power_mW;

    if (recordedValuesIndex < arraySize) 
    {
      voltageArray[recordedValuesIndex] = driveBatteryVoltage;
      currentArray[recordedValuesIndex] = driveBatteryCurrent;
    }
    else
    {
      SerialBT.println("No space left in storage, please reset data");
    }
  }
  else if (currentState == CHARGE) 
  {
    batteryChargeVoltage = busVoltage;
    batteryChargeCurrent = current_mA;
    batteryChargePower = power_mW;

    FinalBatteryChargeVoltage = batteryChargeVoltage - driveBatteryVoltage;  
    FinalBatteryChargeCurrent = batteryChargeCurrent - driveBatteryCurrent;
    FinalBatteryChargePower = batteryChargePower - driveBatteryPower;  
    
    if (recordedValuesIndex < arraySize) 
    {
    voltageArray[recordedValuesIndex] = batteryChargeVoltage;
    currentArray[recordedValuesIndex] = batteryChargeCurrent;
    }
    else
    {
      SerialBT.println("No space left in storage, please reset data");
    }
  }
  else if (currentState == IDLE) // At the end of the run, find total charge used
  {
    totalBatteryVoltageUsed = initialBatteryVoltage - driveBatteryVoltage;
    totalBatteryCurrentUsed = initialBatteryCurrent - driveBatteryCurrent;
    totalBatteryPowerUsed = initialBatteryPower - driveBatteryPower;

    totalEnergyUsed = totalBatteryPowerUsed * totalElapsedRunTime;
    totalEnergyGained = FinalBatteryChargePower * totalElapsedRunTime;

    onceDuringRun = true; // Prepare the car to run again
    running = false; // Stop the elapsed time and index incrementing
  }

  if (running) 
  {
     totalElapsedRunTime += 0.5; // Update run time
     recordedValuesIndex += 1; // Increment Index
  }

}

/**
 * @brief Outputs camera data to the serial monitor.
 * @param result is the HUSKYLENS object with camera data.
 */
void printResult(HUSKYLENSResult result)
{
  if (result.command == COMMAND_RETURN_BLOCK)
  {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  }
  else if (result.command == COMMAND_RETURN_ARROW)
  {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  }
  else
  {
    Serial.println("Object unknown!");
  }
  // delay(1000);
}

/*
  Beginning of kernal
*/
void setup()
{
  Serial.begin(baudRate);
  Wire.begin();               // For I2C communication
  SerialBT.begin(deviceName); // Start SerialBT
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());

  // INA219
  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
      ;
  }
  Serial.println("INA219 Initialized");

  // HUSKYLENS Initilization
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  Serial.println(F("HUSKYLENS Initialized"));
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); // Switch the algorithm to line tracking.

  // Steering initialization:
  ledcAttach(steeringPin, pwmFrequency, pwmResolution);
  setSteeringAngle(steeringAngle);

  // Motor initialization:
  ledcAttach(speedMotorPin, pwmFrequency, pwmResolution);
  delay(1000);               // at least 1s delay to allow driver to recognize servo attachment
  setMotorSpeed(motorSpeed); // the driver needs to first see 0-speed in order to calibrate
  delay(3000);               // this delay may need to be up to 5s (5000ms) for the driver to calibrate to 0-speed
  motorSpeed = 70;           // set a default motor speed

  // Timer Value setup
  loadTimer(5000000); // 5 seconds in microseconds

  // Timer setup for logging every 0.5s (500,000 microseconds)
  timerLog(500000);

  // For PID controller
  pidSetpoint = 0;  // we want the error to be zero
  linePid.SetMode(AUTOMATIC); // turn the PID on
  linePid.SetSampleTime(5); // choose how often to sample (in milliseconds)
  linePid.SetOutputLimits(-90,90); // arbitrary values. This allows the Pid controller to go negative rather than clamping to 0 -> 255
}

void loop()
{
  handleSerialCommunication();

  switch (currentState)
  {
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
