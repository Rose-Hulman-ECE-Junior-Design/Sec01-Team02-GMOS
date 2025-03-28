#include <Wire.h>
#include <Adafruit_INA219.h>
#include <HUSKYLENS.h>
#include <FastPID.h>
#include <BluetoothSerial.h>

// INA219 instance
Adafruit_INA219 ina219;

// ctes:
const int steeringPin = 32;
const int speedMotorPin = 33;
const int pwmFrequency = 50;
const int pwmResolution = 12;
const int pwmMax = 4095;

const float minPulseWidth = 1.0; // 1ms pulse width for 0 degrees
const float maxPulseWidth = 2.0; // 2ms pulse width for 180 degrees
const float periodMs = 20.0;

// default steering angle:
int steeringAngle = 90; // start centered
int motorSpeed = 0; // start centered

HUSKYLENS huskylens;

int ID1 = 1;

// Control variables
int left = 0, right = 0;
int setpoint = 160;  // Center line position
int output = 0;

// PID Constants
const float kp = 0.8, ki = 0.0, kd = 0.000; //kp = 0.60, ki = 0.1, kd = 0.002;
const float hz = 50.0;
FastPID pid(kp, ki, kd, hz, 8, true);

//***************************** Begin Bluetooth Config
// Device Name (For Bluetooth):
String deviceName = "ECE362CarTeam02";
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

void setup() {
    Serial.begin(115200);
    Wire.begin(); //For I2C communication
    SerialBT.begin(deviceName); //Start SerialBT
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());
    
    // INA219
    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while (1);
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
    huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.

    // steering:
    ledcAttach(steeringPin, pwmFrequency, pwmResolution);
    setSteeringAngle(steeringAngle);

    // motor:
    ledcAttach(speedMotorPin, pwmFrequency, pwmResolution);
    delay(1000); // at least 1s delay to allow driver to recognize servo attachment
    setMotorSpeed(motorSpeed); // the driver needs to first see 0-speed in order to calibrate
    delay(3000); // this delay may need to be up to 5s (5000ms) for the driver to calibrate to 0-speed
    motorSpeed = 51; // set a default motor speed
}

void loop() {
    huskylens.request(ID1);
    HUSKYLENSResult result = huskylens.read();
    // printResult(result);

    // Calculate the error:
    int error = (int32_t)result.xTarget - (int32_t)160; // 160 is the center of the frame (320x320)
    // Serial.println(error);
    // Serial.println("\n");

    if(abs(error) > 40) { //lower the number, the closer it will follow the line
      setMotorSpeed(motorSpeed); //set motor speed to a lower value while turning
    } else {
      setMotorSpeed(motorSpeed+20); //set motor speed to a higher value on the straights
    }

    output = pid.step(setpoint, result.xTarget); //output of the PID controller
    setSteeringAngle(steeringAngle + output*10); //steeringAngle + output
    // setSteeringAngle(180);
    // delay(1000);
    // setSteeringAngle(0);
    // delay(1000);


    // bluetoothSerialCommunication();
}

void bluetoothSerialCommunication() {

    /*
    SerialBT.read:  Read a single byte from SerialBT
    SerialBT.write: Write a single byte to connection 

    SerialBT is Tera Term
    Serial is Serial Monitor       
    */

    //Bluetooth
    if (Serial.available()) { //Checks for any data from Arduino IDE: Computer --> Car
        SerialBT.write(Serial.read()); //If data, send it to the connected device

        //Debugging Code
        // Serial.println("Computer --> Car | Serial");
        // SerialBT.println("Computer --> Car | BT");
    }
    if (SerialBT.available()) { //Checks for any data from the connected device: Car --> Computer
        Serial.write(SerialBT.read()); //If data, display in the serial monitor
        
        //Debugging Code
        // Serial.println("Car --> Computer | Serial");
        // SerialBT.println("Car --> Computer | BT");
    }

    //Debugging Code (Prints a number continuously so you know the serial is working)
    // Serial.println(testingNumber);
    // SerialBT.println("From SerialBT3");
    // testingNumber++;
}

void setSteeringAngle(int angle) { //Could try LEDCWRITE instead of sall this John
    angle = constrain(angle, 5, 180);
    float rangeMs = maxPulseWidth - minPulseWidth;
    float pulseWidthMs = minPulseWidth + ((float)angle / 180.0) * rangeMs;
    int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
    ledcWrite(steeringPin, dutyCycle);
}

void setMotorSpeed(int speed) {
    speed = constrain(speed, 0, 120);
    float rangeMs = maxPulseWidth - minPulseWidth;
    float pulseWidthMs = minPulseWidth + ((float)speed / 180.0) * rangeMs;
    int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
    ledcWrite(speedMotorPin, dutyCycle);
}

void readINA219() {
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();
    
    Serial.print("Bus Voltage: "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Current: "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power: "); Serial.print(power_mW); Serial.println(" mW");
    Serial.println("----------------------");
}

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
