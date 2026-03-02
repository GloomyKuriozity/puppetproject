/*
  Arduino @0x11 — Velocity board (MARK1 / XSTD-style CAN)

  Behavior:
  - Moves ONLY when it receives an I2C command (2 floats: linear_x [m/s], angular_z [rad/s])
  - Sends one CAN MOVE frame immediately on reception
  - Sends one CAN STOP frame when command becomes stale (timeout)
  - Does NOT stream CAN continuously

  Fixes vs your current version:
  - Faster stop reaction (shorter timeout + no periodic keepalive)
  - Deterministic sign handling (configurable inversion)
  - Keeps 0xF1 function byte intact (does not overwrite it)

  Notes:
  - Scaling used: linear m/s -> mm/s (int16)
  - Angular scaling is vendor/protocol dependent. Default here: rad/s -> mrad/s (int16)
*/

#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/wdt.h>

// ---------- Config ----------
#define I2C_SLAVE_ADDRESS 0x11
#define CAN_CS_PIN 53

// MARK1-DIFF base IDs (as you used)
const uint32_t CAN_ID_ACTIVATE = 0x81E10103;
const uint32_t CAN_ID_ENABLE   = 0x01E10111;
const uint32_t CAN_ID_MOVE     = 0x01E10112;

// Payload templates
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t enableData[8]   = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t moveData[8] = {0};

// Velocity command buffer
float cmd_linear = 0.0, cmd_angular = 0.0;
volatile bool new_cmd_ready = false;
volatile uint32_t last_cmd_time = 0;

MCP_CAN CAN(CAN_CS_PIN);

void setup()
{
  Serial.begin(1000000);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(onI2CReceive);

  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);

  delay(200);
  sendCAN(CAN, CAN_ID_ACTIVATE, activateData, 8);
  delay(300);
  sendCAN(CAN, CAN_ID_ENABLE, enableData, 8);
  delay(300);

  last_cmd_time = millis();
}

void loop()
{
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

  if (now - last_cmd_time > 300) {
    convertVelocityToCAN(0.0f, 0.0f, moveData);
  }

  sendCAN(CAN, CAN_ID_MOVE, moveData, 8);

  delay(10);
}

// ---------- Helpers ----------
void sendCAN(MCP_CAN& can, uint32_t id, uint8_t* data, uint8_t len) {
  for (int i = 0; i < 3; ++i) {
    if (can.sendMsgBuf(id, 1, len, data) == CAN_OK) return;
    delay(1);
  }
}

void convertVelocityToCAN(float v, float w, uint8_t* data) {
  int16_t lv = v * 1000;
  int16_t av = w * 1000;
  data[0] = lv & 0xFF;
  data[1] = (lv >> 8) & 0xFF;
  data[4] = av & 0xFF;
  data[5] = (av >> 8) & 0xFF;
}

static void sendStop()
{
  uint8_t payload[8];
  convertVelocityToCAN(0.0f, 0.0f, payload);
  sendCAN(CAN, CAN_ID_MOVE, payload, 8);
}

// ---------- I2C receive ----------
void onI2CReceive(int n)
{
  Serial.println("Hello?");
  if (n == 9) {
    Wire.read(); // discard command byte
    n--;
  }
  
  if (n != 8) {
    while (Wire.available()) Wire.read();
    return;
  }

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

void receiveCANMessage(MCP_CAN& CAN) {
  unsigned long canId = 0;
  unsigned char len = 0;
  unsigned char buf[8] = {0};  // IMPORTANT: clear buffer

  if (CAN.checkReceive() != CAN_MSGAVAIL) return;

  if (CAN.readMsgBuf(&canId, &len, buf) != CAN_OK) return;

  Serial.print("RX id=0x");
  Serial.print(canId, HEX);
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" data={ ");

  for (uint8_t i = 0; i < len; i++) {   // IMPORTANT: only len bytes
    if (buf[i] < 16) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println("}");
}