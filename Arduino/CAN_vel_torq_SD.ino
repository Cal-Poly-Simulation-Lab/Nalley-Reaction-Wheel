#include <SPI.h>
#include <SD.h>
#include "mcp_can.h"
#include "MPU9250.h"

#define CAN_CS 10
#define SD_CS 7

MPU9250 mpu;

const int pwmPin = 9;     // PWM output: torque magnitude
const int dirPin = 8;     // Digital output: direction
const int switchPin = 4;  // Control motor controller on/off

MCP_CAN CAN(CAN_CS);

const byte NODE_ID = 127;
const unsigned long REQ_ID = 0x600 + NODE_ID;
const unsigned long RESP_ID = 0x580 + NODE_ID;

// Velocity (606Ch:00)
const byte INDEX_LOW  = 0x6C;
const byte INDEX_HIGH = 0x60;
const byte SUBINDEX   = 0x00;

// Torque (6077h:00)
const byte TORQUE_INDEX_LOW  = 0x77;
const byte TORQUE_INDEX_HIGH = 0x60;

// Position (6064h:00)
const byte POS_INDEX_LOW  = 0x64;
const byte POS_INDEX_HIGH = 0x60;

unsigned long lastLogTime = 0;
const unsigned long logInterval = 200;  // Log every 200 ms

int32_t velocity = 0;
int16_t torque = 0;

enum ReadState { READ_VELOCITY, READ_TORQUE };
ReadState currentState = READ_VELOCITY;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(switchPin, OUTPUT);
  digitalWrite(switchPin, HIGH);

  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH);

  pinMode(CAN_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);
  digitalWrite(SD_CS, HIGH);

  SPI.begin();

  // Init SD
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed");
    while (1);
  }
  Serial.println("SD init OK");

  Serial.println("Creating log file...");
  File dataFile = SD.open("test.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("TimeMS,Velocity,Torque");
    dataFile.close();
    Serial.println("Log file created.");
  } else {
    Serial.println("Failed to create log file");
    while (1);
  }

  // Init CAN
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN init OK");

  delay(100);

  setupController();
  delay(100);

  requestVelocity();  // Start the cycle
}

void loop() {
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned long canId = 0;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&canId, &len, buf);

    if (canId == RESP_ID) {
      // Velocity response
      if (buf[0] == 0x43 && buf[1] == INDEX_LOW && buf[2] == INDEX_HIGH && buf[3] == SUBINDEX) {
        velocity = (int32_t)((uint32_t)buf[4] |
                            ((uint32_t)buf[5] << 8) |
                            ((uint32_t)buf[6] << 16) |
                            ((uint32_t)buf[7] << 24));
        currentState = READ_TORQUE;
        delay(5);
        requestTorque();
      }

      // Torque response
      else if (buf[0] == 0x4B && buf[1] == TORQUE_INDEX_LOW && buf[2] == TORQUE_INDEX_HIGH && buf[3] == 0x00) {
        torque = (int16_t)(buf[4] | (buf[5] << 8));

        unsigned long now = millis();
        Serial.print("Time: "); Serial.print(now);
        Serial.print(" ms | Vel: "); Serial.print(velocity);
        Serial.print(" | Trq: "); Serial.println(torque);

        File dataFile = SD.open("test.csv", FILE_WRITE);
        if (dataFile) {
          dataFile.print(now);
          dataFile.print(",");
          dataFile.print(velocity);
          dataFile.print(",");
          dataFile.println(torque);
          dataFile.close();
        }

        currentState = READ_VELOCITY;
        lastLogTime = millis();
      }

      // SDO Abort
      else if (buf[0] == 0x80) {
        uint16_t index = (buf[2] << 8) | buf[1];
        uint32_t code = ((uint32_t)buf[4]) |
                        ((uint32_t)buf[5] << 8) |
                        ((uint32_t)buf[6] << 16) |
                        ((uint32_t)buf[7] << 24);
        Serial.print("SDO Abort at index 0x");
        Serial.print(index, HEX);
        Serial.print(", code 0x");
        Serial.println(code, HEX);
      }
    }
  }

  // Control request pacing
  if (millis() - lastLogTime >= logInterval && currentState == READ_VELOCITY) {
    requestVelocity();
  }
}

void requestVelocity() {
  byte request[8] = {
    0x40, INDEX_LOW, INDEX_HIGH, SUBINDEX,
    0x00, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(REQ_ID, 0, 8, request);
}

void requestTorque() {
  byte request[8] = {
    0x40, TORQUE_INDEX_LOW, TORQUE_INDEX_HIGH, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(REQ_ID, 0, 8, request);
}

void requestPosition() {
  byte request[8] = {
    0x40, POS_INDEX_LOW, POS_INDEX_HIGH, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(REQ_ID, 0, 8, request);
}

void setupController() {
  Serial.println("Configuring drive...");

  sendSDO(0x6060, 0x00, 0x02);   // Velocity Mode
  delay(50);
  sendSDO(0x6040, 0x00, 0x0006); // Shutdown
  delay(50);
  sendSDO(0x6040, 0x00, 0x0007); // Switch On
  delay(50);
  sendSDO(0x6040, 0x00, 0x000F); // Enable Operation
  delay(50);
  sendSDO(0x60FF, 0x00, 200);    // Target velocity
  delay(50);

  Serial.println("Drive is in Operation Enabled");
}

void sendSDO(uint16_t index, uint8_t subindex, uint32_t value) {
  byte len;
  byte data[8];

  if (value <= 0xFF) {
    data[0] = 0x2F; len = 4;
  } else if (value <= 0xFFFF) {
    data[0] = 0x2B; len = 5;
  } else {
    data[0] = 0x23; len = 8;
  }

  data[1] = index & 0xFF;
  data[2] = (index >> 8) & 0xFF;
  data[3] = subindex;
  data[4] = value & 0xFF;
  data[5] = (value >> 8) & 0xFF;
  data[6] = (value >> 16) & 0xFF;
  data[7] = (value >> 24) & 0xFF;

  CAN.sendMsgBuf(REQ_ID, 0, len, data);
}
