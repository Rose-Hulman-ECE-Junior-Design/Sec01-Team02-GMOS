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

void setup() {
    // steering:
    ledcAttach(steeringPin, pwmFrequency, pwmResolution);
    setSteeringAngle(steeringAngle);

    // motor:
    ledcAttach(speedMotorPin, 50, 12);
    delay(1000); // at least 1s delay to allow driver to recognize servo attachment
    setMotorSpeed(0); // the driver needs to first see 0-speed in order to calibrate
    delay(3000); //this delay may need to be up to 5s (5000ms) for the driver to calibrate to 0-speed
}

void loop() {
    /*
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
    
    // sweep steering from 20 to 160 deg:
    for (int speed = 0; speed <= 120; speed += 10) {
        setMotorSpeed(speed);
        delay(500);
    }
    for (int speed = 120; speed >= 0; speed -= 10) {
        setMotorSpeed(speed);
        delay(500);
    }
    

}

void setSteeringAngle(int angle) {
    // limit steering angle:
    angle = constrain(angle, 20, 160);
    
    float rangeMs = maxPulseWidth - minPulseWidth;
    float pulseWidthMs = minPulseWidth + ((float)angle / 180.0) * rangeMs;
    
    // pulse width to PWM:
    int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
    
    ledcWrite(steeringPin, dutyCycle);
}

void setMotorSpeed(int speed) {
    // limit speed:
    speed = constrain(speed, 0, 120);
    
    float rangeMs = maxPulseWidth - minPulseWidth;
    float pulseWidthMs = minPulseWidth + ((float)speed / 180.0) * rangeMs;
    
    // pulse width to PWM:
    int dutyCycle = (int)(pwmMax * (pulseWidthMs / periodMs));
    
    ledcWrite(speedMotorPin, dutyCycle);
}
