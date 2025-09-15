#include <SPI.h>
#include "mcp_can.h"
#include "MPU9250.h"

// ——— PIN DEFINITIONS ———
const int pwmPin    = 9;   // PWM output: torque magnitude
const int dirPin    = 8;   // Digital output: direction
const int switchPin = 4;   // Motor controller on/off
// const int CAN_CS    = 10;  // SPI chip-select for MCP_CAN
#define CAN_CS 10

// ——— IMU & CAN OBJECTS ———
MPU9250 mpu;
MCP_CAN CAN(CAN_CS);

// ——— CANOPEN PARAMETERS ———
const byte NODE_ID   = 127;
const unsigned long REQ_ID  = 0x600 + NODE_ID;
const unsigned long RESP_ID = 0x580 + NODE_ID;
const byte INDEX_LOW  = 0x6C;  // velocity index low byte
const byte INDEX_HIGH = 0x60;  // velocity index high byte
const byte SUBINDEX   = 0x00;
const byte TORQ_LOW   = 0x77;
const byte TORQ_HIGH  = 0x60;

// ——— PID GAINS & LIMITS ———
const float Kp = 0.008;
const float Ki = 0.001;
const float Kd = 0.02;
const float maxTorque   = 0.5;    // Nm
const float maxIntegral = 50.0;

// ——— TIMING ———
const unsigned long controlInterval = 25;   // ms
const unsigned long logInterval     = 25;  // ms
unsigned long prevControlMs = 0;
unsigned long prevCANReqMs  = 0;

// ——— PID STATE ———
float desiredYaw    = 0.0;
float previousYaw   = 0.0;
float previousTime  = 0.0;
float integralError = 0.0;

// ——— CAN POLLING STATE ———
enum ReadState { READ_VELOCITY, READ_TORQUE };
ReadState currentState = READ_VELOCITY;
int32_t  velocity = 0;
int16_t  torque   = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // pins
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(switchPin, OUTPUT);
  digitalWrite(switchPin, HIGH);

  // IMU init
  Wire.begin();
  delay(2000);
  if (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed.");
    while (1) { delay(500); }
  }
  Serial.println("MPU initialized.");

  // initialize SPI for MCP_CAN
  SPI.begin();

  // CAN init
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed");
    while (1) { }
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN initialized.");

  Serial.println("TimeMS,Yaw,Error,Velocity,Torque");


  delay(100);
  setupController();
  delay(100);

  // kick off first CAN request
  prevCANReqMs = millis() - logInterval;
}

void loop() {
  unsigned long now = millis();

  // —— 1) PID control @25 ms ——
  if (mpu.update() && now - prevControlMs >= controlInterval) {
    control_loop();
    prevControlMs = now;
  }

  // —— 2) Handle incoming CAN frames ——
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned char len = 0, buf[8];
    unsigned long canId = 0;
    CAN.readMsgBuf(&canId, &len, buf);

    if (canId == RESP_ID) {
      if (buf[0] == 0x43 && buf[1] == INDEX_LOW && buf[2] == INDEX_HIGH) {
        // velocity response
        velocity = (int32_t)((uint32_t)buf[4] |
                             ((uint32_t)buf[5] << 8) |
                             ((uint32_t)buf[6] << 16) |
                             ((uint32_t)buf[7] << 24));
        currentState = READ_TORQUE;
        delay(5);
        requestTorque();
      }
      else if (buf[0] == 0x4B && buf[1] == TORQ_LOW && buf[2] == TORQ_HIGH) {
        // torque response
        torque = (int16_t)(buf[4] | (buf[5] << 8));
        // print timestamp + vel/trq
        Serial.print(velocity); Serial.print(",");
        Serial.println(torque);
        currentState = READ_VELOCITY;
      }
      else if (buf[0] == 0x80) {
        // SDO abort
        uint16_t idx = (buf[2] << 8) | buf[1];
        uint32_t code = ((uint32_t)buf[4]) |
                        ((uint32_t)buf[5] << 8) |
                        ((uint32_t)buf[6] << 16) |
                        ((uint32_t)buf[7] << 24);
        Serial.print("SDO Abort idx=0x");
        Serial.print(idx, HEX);
        Serial.print(" code=0x");
        Serial.println(code, HEX);
      }
    }
  }

  // —— 3) Pace CAN requests @200 ms ——
  if (now - prevCANReqMs >= logInterval && currentState == READ_VELOCITY) {
    requestVelocity();
    prevCANReqMs = now;
  }
}

// ——— PID control loop ———
void control_loop() {
  float currentYaw = mpu.getYaw();
  float t = millis() / 1000.0;
  float dt = t - previousTime;
  if (dt <= 0) return;

  // error with wrap
  float error = desiredYaw - currentYaw;
  if (error > 180)  error -= 360;
  if (error < -180) error += 360;

  // integral
  integralError += error * dt;
  integralError = constrain(integralError, -maxIntegral, maxIntegral);

  // derivative (yaw rate)
  float yawRate = (currentYaw - previousYaw) / dt;
  if (yawRate > 180.0/dt)  yawRate -= 360.0/dt;
  if (yawRate < -180.0/dt) yawRate += 360.0/dt;

  // PID
  float torqueCmd = Kp*error + Ki*integralError - Kd*yawRate;
  torqueCmd = constrain(torqueCmd, -maxTorque, maxTorque);

  // direction & PWM
  bool dir = (torqueCmd >= 0);
  digitalWrite(dirPin, dir ? HIGH : LOW);
  int pwmVal = int(fabs(torqueCmd / maxTorque) * 255); 
  pwmVal = constrain(pwmVal, 0, 255);
  analogWrite(pwmPin, pwmVal);

  // print IMU data
  Serial.print(millis()); Serial.print(",");
  Serial.print(currentYaw, 2); Serial.print(",");
  Serial.print(error, 2); Serial.print(",");

  // update state
  previousYaw   = currentYaw;
  previousTime  = t;
}

// ——— CANopen requests ———
void requestVelocity() {
  byte req[8] = { 0x40, INDEX_LOW, INDEX_HIGH, SUBINDEX, 0,0,0,0 };
  CAN.sendMsgBuf(REQ_ID, 0, 8, req);
}
void requestTorque() {
  byte req[8] = { 0x40, TORQ_LOW, TORQ_HIGH, 0x00, 0,0,0,0 };
  CAN.sendMsgBuf(REQ_ID, 0, 8, req);
}

// ——— Drive setup sequence ———
void setupController() {
  Serial.println("Configuring drive...");
  sendSDO(0x6060,0x00,0x02);   // Velocity mode
  delay(50);
  sendSDO(0x6040,0x00,0x0006); // Shutdown
  delay(50);
  sendSDO(0x6040,0x00,0x0007); // Switch on
  delay(50);
  sendSDO(0x6040,0x00,0x000F); // Enable operation
  delay(50);
  sendSDO(0x60FF,0x00,200);    // Target velocity (example)
  delay(50);
  Serial.println("Drive operational.");
}

void sendSDO(uint16_t idx, uint8_t sub, uint32_t val) {
  byte len, data[8];
  if      (val <= 0xFF)   { data[0]=0x2F; len=4; }
  else if (val <= 0xFFFF) { data[0]=0x2B; len=5; }
  else                    { data[0]=0x23; len=8; }
  data[1]= idx &0xFF;
  data[2]= idx>>8;
  data[3]= sub;
  data[4]= val &0xFF;
  data[5]= (val>>8)&0xFF;
  data[6]= (val>>16)&0xFF;
  data[7]= (val>>24)&0xFF;
  CAN.sendMsgBuf(REQ_ID, 0, len, data);
}
