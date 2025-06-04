#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/wdt.h>

// CAN IDs
const uint32_t CAN_ID_MOVE =                        0x01E10112;
const uint32_t CAN_ID_RESET =                       0x01E10105;
const uint32_t CAN_ID_ACTIVATE =                    0x01E10103;
const uint32_t CAN_ID_ENABLE =                      0x01E10111;

// CAN IDs for Charger
const uint32_t CAN_ID_DEVICE_RESTART =              0x06020101;
const uint32_t CAN_ID_ENABLE_GENERAL_SETTINGS =     0x06020103;
const uint32_t CAN_ID_CLEARSTATUS =                 0x06020104;
const uint32_t CAN_ID_CHARGER_SETTINGS =            0x06020113;
const uint32_t CAN_ID_HEARTBEAT =                   0x01E101B0;

// CAN data
uint8_t resetData[8] =                    {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t enableData[8] =                   {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t activateData[8] =                 {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t moveData[8] =                     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// CAN Data for Charger
uint8_t clearstatusData[8] =              {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t enablegeneralsettingsData[8] =    {0x06, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t chargersettingsData[8] =          {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t heartbeatData[8] =                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Constants
#define I2C_SLAVE_ADDRESS 11
#define CAN0_INT 2
#define SPI_CS_PIN  53
#define SPI_CS_CHARGER 9
#define WHEEL_BASE  0.552

// Variables
bool isChargerConnected = false;
volatile bool new_cmd_ready = false;
volatile bool isReceiving = false, hasPendingRequest = false;
float pending_linear_x = 0.0, pending_angular_z = 0.0;
float x = 0.0, y = 0.0, theta = 0.0;
float prevleft = 0, prevright = 0;
float linear_velocity = 0.0, angular_velocity = 0.0;
float linear_x = 0.0, angular_z = 0.0;
unsigned long now = 0;
unsigned long prevTime = 0;
unsigned long lastCommandTime = 0;
unsigned long readStart = 0;
static unsigned long lastOdoCheck = 0;
static unsigned long lastCmdSendTime = 0;
static unsigned long lastHeartbeat = 0;
static unsigned long lastOdoUpdate = 0;
uint16_t missedCommandCount = 0;

// CAN definition
MCP_CAN CAN(SPI_CS_PIN); // CAN interface for robot exchange
MCP_CAN CAN_CHARGER(SPI_CS_CHARGER);  // CAN interface for charger


void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvents);
  Wire.onRequest(requestEvent);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

  while (CAN_CHARGER.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    delay(1000);
  }
  CAN_CHARGER.setMode(MCP_NORMAL);

  // Initialize robot
  sendCANMessage(CAN, CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  sendCANMessage(CAN, CAN_ID_ENABLE, enableData, 8);
  delay(1000);

  // Initialize charger
  sendCANMessage(CAN_CHARGER, CAN_ID_ENABLE_GENERAL_SETTINGS, enablegeneralsettingsData, 8);
  delay(1000);
  sendCANMessage(CAN_CHARGER, CAN_ID_CLEARSTATUS, clearstatusData, 8);
  delay(1000);
  sendCANMessage(CAN_CHARGER, CAN_ID_CHARGER_SETTINGS, chargersettingsData, 8);
  delay(1000);

  // Initialize variables and clock
  unsigned long startupTime = millis();
  prevTime = startupTime;
  readStart = startupTime;
  now = startupTime;
  lastOdoUpdate = startupTime;
  lastOdoCheck = startupTime;
  lastCmdSendTime = startupTime;
  lastCommandTime = startupTime;
  lastHeartbeat = startupTime;
  pinMode(13, OUTPUT);
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();

  if (isExpired(lastOdoCheck, 15)) {
    checkOdometry();
    lastOdoCheck = millis();
  }

  if (isExpired(lastOdoUpdate, 1000)) {
    linear_velocity = 0.0;
    angular_velocity = 0.0;
  }

  if (new_cmd_ready) {
    lastCommandTime = millis();  // Refresh command timestamp when new command is received

    noInterrupts();  // Disable interrupts while accessing shared variables
    linear_x = pending_linear_x;
    angular_z = pending_angular_z;
    new_cmd_ready = false;
    interrupts();

    convertSpeedToCANData(linear_x, angular_z, moveData);
  }

  now = millis();

  if (CAN_CHARGER.checkReceive() == CAN_MSGAVAIL) {
    uint32_t canId;
    uint8_t len;
    uint8_t buf[8];
    CAN_CHARGER.readMsgBuf(&canId, &len, buf);

    if (canId == 0x860201B3) {
      isChargerConnected = (buf[0] == 0x01);  // 0x01 means charger is touching
    }
  }

  if (isExpired(lastHeartbeat, 500)) {
    sendCANMessage(CAN_CHARGER, CAN_ID_HEARTBEAT, heartbeatData, 8);
    lastHeartbeat = now;
  }

  if (isExpired(lastCmdSendTime, 50)) {
    sendCANMessage(CAN, CAN_ID_MOVE, moveData, 8);
    lastCmdSendTime = now;
  }

  if (isExpired(lastCommandTime, 1000)) {
    memset(moveData, 0, sizeof(moveData));
    linear_x = 0.0;
    angular_z = 0.0;
    convertSpeedToCANData(0.0, 0.0, moveData);  // Make sure CAN gets updated
    digitalWrite(13, LOW);  // Show idle status
    missedCommandCount++; // to be used in diagnostic
  }
}


bool isExpired(unsigned long lastTime, unsigned long interval) {
  return (long)(millis() - lastTime) >= (long)interval;
}


void sendCANMessage(MCP_CAN &CANBus, uint32_t id, uint8_t *data, uint8_t len) {
  for (int attempts = 0; attempts < 3; ++attempts) {
    if (CANBus.sendMsgBuf(id, 1, len, data) == CAN_OK) {
      return;
    }
    delay(1);         // small delay
    wdt_reset();      // avoid freeze
  }

  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if ((long)(now - lastBlink) >= 1000) {
    digitalWrite(13, ledState ? HIGH : LOW);
    ledState = !ledState;
    lastBlink = now;
  }

}


void checkOdometry() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long canId = 0;
    uint8_t len = 0, buf[8];
    CAN.readMsgBuf(&canId, &len, buf);

    if (canId == 0x81E101B3) {

      int32_t left_wheel_odometry = (int32_t)buf[0] | ((int32_t)buf[1] << 8) | ((int32_t)buf[2] << 16) | ((int32_t)buf[3] << 24);
      int32_t right_wheel_odometry = (int32_t)buf[4] | ((int32_t)buf[5] << 8) | ((int32_t)buf[6] << 16) | ((int32_t)buf[7] << 24);

      float left_odom_m = (left_wheel_odometry - prevleft) / 1000.0;
      float right_odom_m = (right_wheel_odometry - prevright) / 1000.0;

      prevleft = left_wheel_odometry;
      prevright = right_wheel_odometry;

      unsigned long currentTime = millis();
      float deltaTime = (currentTime - prevTime) / 1000.0;
      if (prevTime == 0) {
        prevTime = currentTime;
        return; // Skip the first iteration
      }
      prevTime = currentTime;

      linear_velocity = (left_odom_m + right_odom_m) / (2.0 * deltaTime);
      angular_velocity = (left_odom_m - right_odom_m) / (WHEEL_BASE * deltaTime);


      theta += angular_velocity * deltaTime;
      x += linear_velocity * cos(theta) * deltaTime;
      y += linear_velocity * sin(theta) * deltaTime;

      lastOdoUpdate = millis();
    }
  }
}



void receiveEvents(int numBytes) {
  isReceiving = true;

  if (numBytes == 9) {
    union {
      byte byteArray[8];
      float floatArray[2];
    } data;

    Wire.read(); // Ignore the first byte (command byte)

    readStart = millis();
    for (int i = 0; i < 8; i++) {
      while (!Wire.available()) {
        if (millis() - readStart > 10) {
          // Timeout reached: abort and mark data invalid
          isReceiving = false;
          return;
        }
      }
      data.byteArray[i] = Wire.read();
    }

    if (!isfinite(data.floatArray[0]) || !isfinite(data.floatArray[1])) {
      isReceiving = false;
      return;
    }

    pending_linear_x = data.floatArray[0];
    pending_angular_z = data.floatArray[1];
    new_cmd_ready = true;

  } else if (numBytes == 1) {
    Wire.read(); // Clear the command byte
  }

  isReceiving = false;
  wdt_reset();  // Ensure watchdog resets during I2C ISR
}



void requestEvent() {
  if (isReceiving) return;
  hasPendingRequest = true;

  wdt_reset();  // Reset watchdog in case this takes time

  // Sanity check on odometry data before sending
  if (!isfinite(x) || !isfinite(y) || !isfinite(theta) ||
      !isfinite(linear_velocity) || !isfinite(angular_velocity) ||
      abs(x) > 1000 || abs(y) > 1000 || abs(theta) > 50 ||
      abs(linear_velocity) > 10 || abs(angular_velocity) > 10) {
    // You can log this if needed
    //Serial.println("⚠️ Invalid odometry detected — skipping I2C transmission.");
    hasPendingRequest = false;
    return;
  }

  // Prepare odometry data
  struct {
    float x, y, theta, linear_velocity, angular_velocity;
  } odometry_data = {x, y, theta, linear_velocity, angular_velocity};

  // Send odometry data and sonar status over I2C
  Wire.write((byte*)&odometry_data, sizeof(odometry_data));

  hasPendingRequest = false;
}

void convertSpeedToCANData(float speed_linear, float speed_angular, uint8_t *data) {
  int16_t fixedPointSpeedLinear = speed_linear * 1000;
  int16_t fixedPointSpeedAngular = speed_angular * 1000;
  data[0] = fixedPointSpeedLinear & 0xFF;
  data[1] = (fixedPointSpeedLinear >> 8) & 0xFF;
  data[4] = fixedPointSpeedAngular & 0xFF;
  data[5] = (fixedPointSpeedAngular >> 8) & 0xFF;
}
