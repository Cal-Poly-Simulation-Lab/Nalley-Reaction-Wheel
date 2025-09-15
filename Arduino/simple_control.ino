#include "MPU9250.h"

MPU9250 mpu;

const int pwmPin = 9;     // PWM output: torque magnitude
const int dirPin = 8;     // Digital output: direction
const int switchPin = 4;  // Control motor controller on/off

const float supplyVoltage = 5.0;
const int pwmResolution = 255;
const float maxTorque = 0.5;

// PID Gains (tune as needed)
/*
const float Kp = 0.008;
const float Ki = 0.001;
const float Kd = 0.02;
*/
const float Kp = 0.008;
const float Ki = 0.001;
const float Kd = 0.02;

const float maxIntegral = 50.0;

float desiredYaw = 0.0;
float previousYaw = 0.0;
float previousTime = 0.0;
float integralError = 0.0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(switchPin, OUTPUT);
    digitalWrite(switchPin, HIGH);

if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU connection failed.");

            delay(5000);
        }
    }
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            control_loop();
            prev_ms = millis();
        }
    }
}

void control_loop() {
    float currentYaw = mpu.getYaw();
    float currentTime = millis() / 1000.0;
    float dt = currentTime - previousTime;
    if (dt <= 0.0) return;  // Prevent division by zero

    // Error wrapping
    float error = desiredYaw - currentYaw;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    // Integrate error over time
    integralError += error * dt;
    // Clamp integral to avoid windup
    integralError = constrain(integralError, -maxIntegral, maxIntegral);

    // Derivative
    float yawRate = (currentYaw - previousYaw) / dt;
    if (yawRate > 180.0 / dt) yawRate -= 360.0 / dt;
    if (yawRate < -180.0 / dt) yawRate += 360.0 / dt;

    // PID Control
    float torqueCommand = Kp * error + Ki * integralError - Kd * yawRate;
    torqueCommand = constrain(torqueCommand, -maxTorque, maxTorque);

    // Determine direction
    bool direction = (torqueCommand >= 0);
    digitalWrite(dirPin, direction ? HIGH : LOW);

    // Set PWM duty cycle proportional to magnitude
    int pwmValue = int(fabs(torqueCommand / maxTorque) * pwmResolution);
    pwmValue = constrain(pwmValue, 0, pwmResolution);
    analogWrite(pwmPin, pwmValue);

    // Print to serial monitor
    Serial.print("Yaw: ");
    Serial.print(currentYaw, 2);
    Serial.print(" | Error: ");
    Serial.print(error, 2);
    Serial.print(" | Torque: ");
    Serial.print(torqueCommand, 4);
    Serial.print(" | PWM: ");
    Serial.println(pwmValue);

    // Update state
    previousYaw = currentYaw;
    previousTime = currentTime;
}
