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
State currentState = DRIVE;
 
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
String deviceName = "Milton!!!";
String deviceName = "Milton!!!";
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
//***************************** End Bluetooth Config
 
// Timer handle
esp_timer_handle_t stateTimer;
esp_timer_handle_t INA219BT_timer;
 
// Timer callback function, prints the current state every 5 seconds
void printState(void* arg) {
    switch (currentState) {
        case IDLE:
            Serial.println("State: IDLE");
            break;
        case DRIVE:
            Serial.println("State: DRIVE");
            break;
        case CHARGE:
            Serial.println("State: CHARGE");
            break;
    }
}
 
void setup() {
    Serial.begin(baudRate);
    Wire.begin();                 //For I2C communication
    SerialBT.begin(deviceName);   //Start SerialBT
    Serial.begin(baudRate);
    Wire.begin();                 //For I2C communication
    SerialBT.begin(deviceName);   //Start SerialBT
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());
   
    // INA219
    if(!ina219.begin()) {
    if(!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while(1);
        while(1);
    }
    Serial.println("INA219 Initialized");
   
    // HUSKYLENS Initilization
    while(!huskylens.begin(Wire))
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
    motorSpeed = 50; // set a default motor speed
 
    // Timer setup
    const esp_timer_create_args_t stateTimerArgs = {
        .callback = &printState, // Function to call on timer expiration
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "state_timer"
    };
    esp_timer_create(&stateTimerArgs, &stateTimer);
    esp_timer_start_periodic(stateTimer, 5000000); // 5 seconds in microseconds
 
    // Timer setup for logging every 0.5s (500,000 microseconds)
    const esp_timer_create_args_t INA219BTTimerArgs = {
        .callback = &readINA219BT,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "INA219BT_timer"
    };
    esp_timer_create(&INA219BTTimerArgs, &INA219BT_timer);
    esp_timer_start_periodic(INA219BT_timer, 500000);
}
 
void loop() {
    handleSerialCommunication();
   
    switch (currentState) {
        case IDLE:
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
    setSteeringAngle(STRAIGHT - 0.6 * error);
}
 
/**
 * @brief Handles incoming serial communication commands.
 */
void handleSerialCommunication() {
  if (Serial.available()) {
    char command = Serial.read();
    SerialBT.write(command);
    processCommand(tolower(command));
  }  
  if (SerialBT.available()) {
    char command = SerialBT.read();  
    Serial.write(command);
    processCommand(tolower(command));
  }
}
 
/**
 * @brief Processes command input and changes state accordingly.
 * @param command The command character received.
 */
void processCommand(char command) {
    switch (command) {
        case 'i':
            currentState = IDLE;
            break;
        case 'd':
            currentState = DRIVE;
            break;
        case 'c':
            currentState = CHARGE;
            break;
    }
}
 
/**
 * @brief Inputs a steering angle and sets the steering lock accordingly via PWM signal.
 * @param angle is the target steering angle, where 0 is is fully left and 180 is fully right.
 */
void setSteeringAngle(int angle) { //Could try LEDCWRITE instead of sall this John
    angle = constrain(angle, 5, 180);
    angle = constrain(angle, 5, 180);
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
 * @brief Outputs the car's bus voltage, current and power via BT to the serial monitor.
 */
void readINA219BT(void* arg) {
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
   
    SerialBT.print(busVoltage);
    SerialBT.print(",");
    SerialBT.print(current_mA);
    SerialBT.print(",");
    SerialBT.println(power_mW);
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