#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>

#define I2C_SLAVE_ADDRESS 11
#define CAN0_INT 2
const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);

// CAN IDs
const uint32_t CAN_ID_MOVE = 0x01E10112;
const uint32_t CAN_ID_RESET = 0x01E10105;
const uint32_t CAN_ID_ACTIVATE = 0x01E10103;
const uint32_t CAN_ID_ENABLE = 0x01E10111;

uint8_t resetData[8] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t moveData[8] = {0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t enableData[8] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};

float x = 0.0, y = 0.0, theta = 0.0, dT = 0.02, prevleft = 0, prevright = 0;
float linear_velocity = 0.0, angular_velocity = 0.0;
unsigned long lastTime = 0;
float WHEEL_BASE = 0.552;

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onRequest(requestEvent);  // Register request event handler
  Serial.begin(200000);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed.");
    while (1);
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
  delay(1000);

  sendCANMessage(CAN_ID_RESET, resetData, 8);
  delay(1000);

  sendCANMessage(CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  sendCANMessage(CAN_ID_ENABLE, enableData, 8);
  delay(1000);
}

void loop() {
  sendCANMessage(CAN_ID_MOVE, moveData, 8);
  checkOdometry();
}

void sendCANMessage(uint32_t id, uint8_t *data, uint8_t len) {
  if (CAN.sendMsgBuf(id, 1, len, data) != CAN_OK) {
    Serial.println("Error sending message");
  }
}

void checkOdometry() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long canId = 0;
    uint8_t len = 0;
    uint8_t buf[8];
    CAN.readMsgBuf(&canId, &len, buf);

    if (canId == 0x81E101B3) {  // Example CAN ID for odometry
      int32_t left_wheel_odometry = (int32_t)buf[0] | ((int32_t)buf[1] << 8) | ((int32_t)buf[2] << 16) | ((int32_t)buf[3] << 24);
      int32_t right_wheel_odometry = (int32_t)buf[4] | ((int32_t)buf[5] << 8) | ((int32_t)buf[6] << 16) | ((int32_t)buf[7] << 24);

      float left_odom_m = (left_wheel_odometry - prevleft) / 1000.0;
      float right_odom_m = (right_wheel_odometry - prevright) / 1000.0;
      prevleft = left_wheel_odometry;
      prevright = right_wheel_odometry;

      linear_velocity = (left_odom_m + right_odom_m) / (2.0 * dT);
      angular_velocity = (left_odom_m - right_odom_m) / (WHEEL_BASE * dT);

      theta += angular_velocity * dT;

      x += linear_velocity * cos(theta) * dT;
      y += linear_velocity * sin(theta) * dT;
    }
  }
}

// Function that executes whenever data is requested from the master
void requestEvent() {
  sendOdometryOverI2C();
}

void sendOdometryOverI2C() {
  Wire.write((byte*)&x, sizeof(x));
  Wire.write((byte*)&y, sizeof(y));
  Wire.write((byte*)&theta, sizeof(theta));
  Wire.write((byte*)&linear_velocity, sizeof(linear_velocity));
  Wire.write((byte*)&angular_velocity, sizeof(angular_velocity));
}
