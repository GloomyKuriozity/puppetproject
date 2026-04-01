#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <math.h>

#define I2C_SLAVE_ADDRESS 0x22
#define CAN_CS_PIN 53

static const float WHEEL_BASE_M = 0.550f;

volatile float battery_voltage = -5.0f;
volatile float x = 0.0f, y = 0.0f, theta = 0.0f;
volatile float linear_velocity = 0.0f, angular_velocity = 0.0f;

volatile uint32_t last_battery_update_ms = 0;

const uint32_t CAN_ID_ODOM = 0x81E101B3;
const uint32_t CAN_ID_STATUS = 0x81E101B1;

uint32_t prevTime = 0;
uint32_t lastPrintMs = 0;

int32_t prev_left_mm = 0, prev_right_mm = 0;

bool have_prev = false;

MCP_CAN CAN(CAN_CS_PIN);

struct TelemetryPayload {
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;
  float battery_voltage;
};

TelemetryPayload telemetry = {0};

void setup()
{
  //Serial.begin(9600);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(100000);
  Wire.onRequest(onI2CRequest);

  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);

  prevTime = millis();
}

void loop()
{
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long canId=0; uint8_t len=0; uint8_t buf[8];
    if (CAN.readMsgBuf(&canId, &len, buf) != CAN_OK) continue;

    uint32_t now = millis();
    if (now - lastPrintMs >= 50) {
      //dumpRX(canId, len, buf);
      lastPrintMs = now;
    }

    if (len == 8 && ((canId & 0x1FFFFFFFul) == (CAN_ID_ODOM & 0x1FFFFFFFul))) {
      updateOdometry(read_i32_le(&buf[0]), read_i32_le(&buf[4]));
    }

    if (len >= 4 && ((canId & 0x1FFFFFFFul) == (CAN_ID_STATUS & 0x1FFFFFFFul))) {
      updateBatteryVoltage(buf);
    }
  }
  updateTelemetrySnapshot();
  //dumpTelemetry();
}

void updateBatteryVoltage(const uint8_t *buf)
{
  const uint16_t decivolts = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
  battery_voltage = decivolts * 0.1f;   // protocol unit = 0.1 V
  last_battery_update_ms = millis();
}

static inline int32_t read_i32_le(const uint8_t *p)
{
  return (int32_t)p[0] |
         ((int32_t)p[1] << 8) |
         ((int32_t)p[2] << 16) |
         ((int32_t)p[3] << 24);
}

void updateTelemetrySnapshot()
{
  noInterrupts();
  telemetry.x = x;
  telemetry.y = y;
  telemetry.theta = theta;
  telemetry.linear_velocity = linear_velocity;
  telemetry.angular_velocity = angular_velocity;
  telemetry.battery_voltage = battery_voltage;
  interrupts();
}

void dumpTelemetry()
{
  Serial.print(" x = "); Serial.print(x);
  Serial.print(" y = "); Serial.print(y);
  Serial.print(" theta = "); Serial.print(theta);
  Serial.print(" linear_velocity = "); Serial.print(linear_velocity);
  Serial.print(" angular_velocity = "); Serial.print(angular_velocity);
  Serial.print(" battery_voltage = "); Serial.print(battery_voltage);
  Serial.println();
}


void dumpRX(unsigned long id, uint8_t len, uint8_t* buf)
{
  Serial.print(" RX id=0x"); Serial.print(id, HEX);
  Serial.print(" len="); Serial.print(len);
  Serial.print(" data=");
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 16) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void onI2CRequest()
{
  Wire.write((const uint8_t*)&telemetry, sizeof(telemetry));
}

void sendCAN(MCP_CAN& can, uint32_t id, uint8_t* data, uint8_t len) {
  for (int i = 0; i < 3; ++i) {
    if (can.sendMsgBuf(id, 1, len, data) == CAN_OK) return;
    delay(1);
  }
}

byte sendCAN_dbg(MCP_CAN& can, uint32_t id, uint8_t* data, uint8_t len)
{
  byte st = CAN_FAILTX;
  st = can.sendMsgBuf(id, 1, len, data);

  Serial.print(" TX id=0x"); Serial.print(id, HEX);
  Serial.print(" len="); Serial.print(len);
  Serial.print(" st="); Serial.print(st);
  Serial.print(" data=");
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 16) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  return st;
}

void updateOdometry(int32_t left_mm, int32_t right_mm)
{
  const uint32_t now = millis();
  float dt = (now - prevTime) * 0.001f;
  prevTime = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  if (!have_prev) {
    prev_left_mm = left_mm;
    prev_right_mm = right_mm;
    have_prev = true;
    return;
  }

  const int32_t dL_mm = left_mm - prev_left_mm;
  const int32_t dR_mm = right_mm - prev_right_mm;
  prev_left_mm = left_mm;
  prev_right_mm = right_mm;

  const float dL = dL_mm * 0.001f; // m
  const float dR = dR_mm * 0.001f; // m

  const float ds = 0.5f * (dL + dR);
  const float dtheta = (dR - dL) / WHEEL_BASE_M;

  const float theta_mid = theta + 0.5f * dtheta;
  x += ds * cosf(theta_mid);
  y += ds * sinf(theta_mid);
  theta += dtheta;

  if (theta > (float)M_PI) theta -= 2.0f * (float)M_PI;
  if (theta < -(float)M_PI) theta += 2.0f * (float)M_PI;

  linear_velocity = ds / dt;
  angular_velocity = dtheta / dt;
}
