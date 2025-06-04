#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/wdt.h>

// CAN IDs
const uint32_t CAN_ID_MOVE     = 0x01E10112;
const uint32_t CAN_ID_ACTIVATE = 0x01E10103;
const uint32_t CAN_ID_ENABLE   = 0x01E10111;
const uint32_t CAN_ID_HEARTBEAT = 0x01E101B0;

// Charger IDs
const uint32_t CAN_ID_ENABLE_GENERAL_SETTINGS = 0x06020103;
const uint32_t CAN_ID_CLEARSTATUS             = 0x06020104;
const uint32_t CAN_ID_CHARGER_SETTINGS        = 0x06020113;

const uint32_t CAN_ID_ODOMETRY                = 0x81E101B3;

uint8_t moveData[8] = {0};
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01};
uint8_t enableData[8]   = {0x02, 0x00, 0x00, 0x01};
uint8_t heartbeatData[8] = {0};
uint8_t enableSettingsData[8] = {0x06, 0x02, 0x01, 0x01};
uint8_t clearStatusData[8]    = {0xCC};
uint8_t chargerSettingsData[8] = {0x00, 0x00, 0x01, 0x00};

#define WHEEL_BASE 0.552
#define I2C_SLAVE_ADDRESS 11
#define SPI_CS_ROBOT  53
#define SPI_CS_CHARGER 9

MCP_CAN CAN(SPI_CS_ROBOT);
MCP_CAN CAN_CHARGER(SPI_CS_CHARGER);

// Odometry state
float x = 0.0, y = 0.0, theta = 0.0;
float linear_velocity = 0.0, angular_velocity = 0.0;
int32_t prev_left = 0, prev_right = 0;
unsigned long last_odo_time = 0;
unsigned long last_cmd_time = 0;
unsigned long last_heartbeat = 0;

// Velocity command buffer
float cmd_linear = 0.0, cmd_angular = 0.0;
volatile bool new_cmd_ready = false;

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(receiveCommand);
  Wire.onRequest(sendOdometry);
  
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);

  CAN_CHARGER.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  CAN_CHARGER.setMode(MCP_NORMAL);

  sendCAN(CAN, CAN_ID_ACTIVATE, activateData, 8);
  delay(500);
  sendCAN(CAN, CAN_ID_ENABLE, enableData, 8);
  delay(500);

  sendCAN(CAN_CHARGER, CAN_ID_ENABLE_GENERAL_SETTINGS, enableSettingsData, 8);
  delay(500);
  sendCAN(CAN_CHARGER, CAN_ID_CLEARSTATUS, clearStatusData, 8);
  delay(500);
  sendCAN(CAN_CHARGER, CAN_ID_CHARGER_SETTINGS, chargerSettingsData, 8);

  last_odo_time = millis();
  last_cmd_time = millis();
  last_heartbeat = millis();

  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();
  unsigned long now = millis();

  if (new_cmd_ready) {
    noInterrupts();
    float v = cmd_linear;
    float w = cmd_angular;
    new_cmd_ready = false;
    interrupts();
    convertVelocityToCAN(v, w, moveData);
    last_cmd_time = now;
  }

  if (now - last_cmd_time > 1000) {
    memset(moveData, 0, 8);
  }

  if (now - last_heartbeat > 500) {
    sendCAN(CAN_CHARGER, CAN_ID_HEARTBEAT, heartbeatData, 8);
    last_heartbeat = now;
  }

  sendCAN(CAN, CAN_ID_MOVE, moveData, 8);

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    uint32_t canId;
    uint8_t len;
    uint8_t buf[8];
    CAN.readMsgBuf(&canId, &len, buf);

    if (canId == CAN_ID_ODOMETRY) {
      int32_t left = *((int32_t*)&buf[0]);
      int32_t right = *((int32_t*)&buf[4]);
      updateOdometry(left, right);
    }
  }

  delay(10);  // yield to avoid hogging CPU
}

void receiveCommand(int n) {
  if (n != 9) return;

  Wire.read(); // Command byte
  union {
    byte raw[8];
    float vals[2];
  } data;

  for (int i = 0; i < 8; ++i) {
    data.raw[i] = Wire.read();
  }

  if (isfinite(data.vals[0]) && isfinite(data.vals[1])) {
    cmd_linear = data.vals[0];
    cmd_angular = data.vals[1];
    new_cmd_ready = true;
  }
}

void sendOdometry() {
  struct {
    float x, y, theta, v, w;
  } odo = {x, y, theta, linear_velocity, angular_velocity};

  Wire.write((uint8_t*)&odo, sizeof(odo));
}

void updateOdometry(int32_t left, int32_t right) {
  unsigned long now = millis();
  float dt = (now - last_odo_time) / 1000.0;
  if (dt <= 0.0) return;
  last_odo_time = now;

  float d_left = (left - prev_left) / 1000.0;
  float d_right = (right - prev_right) / 1000.0;

  prev_left = left;
  prev_right = right;

  float d_center = (d_left + d_right) / 2.0;
  float d_theta = (d_left - d_right) / WHEEL_BASE;

  linear_velocity = d_center / dt;
  angular_velocity = d_theta / dt;

  theta += d_theta;
  x += d_center * cos(theta);
  y += d_center * sin(theta);
}

void convertVelocityToCAN(float v, float w, uint8_t* data) {
  int16_t lv = v * 1000;
  int16_t av = w * 1000;
  data[0] = lv & 0xFF;
  data[1] = (lv >> 8) & 0xFF;
  data[4] = av & 0xFF;
  data[5] = (av >> 8) & 0xFF;
}

void sendCAN(MCP_CAN& can, uint32_t id, uint8_t* data, uint8_t len) {
  for (int i = 0; i < 3; ++i) {
    if (can.sendMsgBuf(id, 1, len, data) == CAN_OK) return;
    delay(1);
    wdt_reset();
  }
}
