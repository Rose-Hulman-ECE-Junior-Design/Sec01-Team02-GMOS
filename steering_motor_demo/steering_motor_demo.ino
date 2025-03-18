#include <Wire.h>
#include <Adafruit_INA219.h>
#include <BluetoothSerial.h>

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
int steeringAngle = 90; // start centered

// Device Name (For Bluetooth)
String deviceName = "ECE362CarTeam02";
BluetoothSerial SerialBT //renaming BluetoothSerial to SerialBT for so it reads better

void setup() {
    Serial.begin(115200);
    Wire.begin(); //For I2C communication
    SerialBT.begin(ECE362CarTeam02);
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with 
    Bluetooth!\n", ECE362CarTeam02.c_str());
    
    
    if (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        while (1);
    }
    Serial.println("INA219 Initialized");
    
    // steering:
    ledcAttach(steeringPin, pwmFrequency, pwmResolution);
    setSteeringAngle(steeringAngle);

    // motor:
    ledcAttach(speedMotorPin, pwmFrequency, pwmResolution);
    delay(1000); // at least 1s delay to allow driver to recognize servo attachment
    setMotorSpeed(0); // the driver needs to first see 0-speed in order to calibrate
    delay(3000); // this delay may need to be up to 5s (5000ms) for the driver to calibrate to 0-speed
}

void loop() {
    // sweep speed from 0 to 120 and back
    /*
    for (int speed = 0; speed <= 120; speed += 10) {
        setMotorSpeed(speed);
        readINA219();
        delay(500);
    }
    
    for (int speed = 120; speed >= 0; speed -= 10) {
        setMotorSpeed(speed);
        readINA219();
        delay(500);
    }
    
    // sweep steering from 20 to 160 deg:
    for (int angle = 20; angle <= 160; angle += 10) {
        setSteeringAngle(angle);
        delay(500);
    }
    for (int angle = 160; angle >= 20; angle -= 10) {
        setSteeringAngle(angle);
        delay(500);
    }
    */
    setMotorSpeed(120);
    setSteeringAngle(160);
    readINA219();
    delay(500);

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
