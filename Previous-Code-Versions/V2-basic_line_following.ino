#include <Wire.h>
#include <Adafruit_INA219.h>
#include <HUSKYLENS.h>
#include <FastPID.h>

// INA219 instance
Adafruit_INA219 ina219;

// ctes:
const int steeringPin = 32;
const int speedMotorPin = 33;
const int pwmFrequency = 50;
const int pwmResolution = 12;
const int pwmMax = 4095;

const float minPulseWidth = 1.0;
const float maxPulseWidth = 2.0;
const float periodMs = 20.0;

// default steering angle:
int centerSteeringAngle = 90; // start centered
int motorSpeed = 0; // start centered

HUSKYLENS huskylens;

int ID1 = 1;

// Control variables
int left = 0, right = 0;
int setpoint = 160;  // Center line position
int output = 0;

// PID Constants
const float kp = 0.75, ki = 0.0, kd = 0.000;
const float hz = 50.0;
FastPID pid(kp, ki, kd, hz, 8, true);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while (1);
    }
    Serial.println("INA219 Initialized");
    
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
    setSteeringAngle(centerSteeringAngle);

    // motor:
    ledcAttach(speedMotorPin, pwmFrequency, pwmResolution);
    delay(1000); // at least 1s delay to allow driver to recognize servo attachment
    setMotorSpeed(motorSpeed); // the driver needs to first see 0-speed in order to calibrate
    delay(3000); // this delay may need to be up to 5s (5000ms) for the driver to calibrate to 0-speed
    motorSpeed = 45;
}

void loop() {
    
    //int32_t error;
    motorSpeed = 45;
    
    huskylens.request(ID1);
    HUSKYLENSResult result = huskylens.read();
    printResult(result);

    // Calculate the error:
    int error = (int32_t)result.xTarget - (int32_t)160;
    Serial.println(error);
    Serial.println("\n");
    if(abs(error) > 40) {
      motorSpeed = 50;
      setMotorSpeed(motorSpeed);
    } else {
      setMotorSpeed(motorSpeed+10);
    }
    

    output = pid.step(setpoint, result.xTarget);
    delay(10);
    setSteeringAngle(160); //centerSteeringAngle+output*1.5
    //delay(10);
}

void setSteeringAngle(int angle) {
    angle = constrain(angle, 20, 160);
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