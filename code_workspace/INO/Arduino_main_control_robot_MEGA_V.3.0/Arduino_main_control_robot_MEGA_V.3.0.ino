#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>

// **CAN IDs for Robot**
const uint32_t CAN_ID_MOVE = 0x01E10112;
const uint32_t CAN_ID_RESTART = 0x01E10101;
const uint32_t CAN_ID_ACTIVATE = 0x01E10103;
const uint32_t CAN_ID_ENABLE = 0x01E10111;

// **CAN IDs for Charger**
const uint32_t CAN_ID_DEVICE_RESTART = 0x06020101;
const uint32_t CAN_ID_ENABLE_GENERAL_SETTINGS = 0x06020103;
const uint32_t CAN_ID_CLEARSTATUS = 0x06020104;
const uint32_t CAN_ID_CHARGER_SETTINGS = 0x06020113;

// **CAN Data Buffers**
uint8_t enableData[8] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t clearstatusData[8] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t enablegeneralsettingsData[8] = {0x06, 0x02, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t chargersettingsData[8] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

// **Robot Movement Variables**
float linear_x = 0.0, angular_z = 0.0;
uint8_t moveData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
volatile bool isReceiving = false;
static unsigned long lastSendTime = 0;

// **Robot-Charger Status Variables**
bool isChargerConnected = false;

// **Heartbeat for Charger Activation**
uint8_t heartbeatData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// **I2C & CAN Configuration**
#define I2C_SLAVE_ADDRESS 11
#define SPI_CS1 53  // Robot CAN (Hardware SPI)
#define SPI_CS2 9   // Charger CAN (Software SPI)

// **Initialize CAN Modules**
MCP_CAN CAN1(SPI_CS1);  // Robot CAN (Hardware SPI)
MCP_CAN CAN2(SPI_CS2);  // Charger CAN (Software SPI)

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvents);

  while (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(1000); }
  while (CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(1000); }

  CAN1.setMode(MCP_NORMAL);
  CAN2.setMode(MCP_NORMAL);

  // **Initialize Charger System**
  sendCANMessage(CAN2, CAN_ID_ENABLE_GENERAL_SETTINGS, enablegeneralsettingsData, 8);
  delay(1000);
  sendCANMessage(CAN2, CAN_ID_CLEARSTATUS, clearstatusData, 8);
  delay(1000);
  sendCANMessage(CAN2, CAN_ID_CHARGER_SETTINGS, chargersettingsData, 8);
  delay(1000);

  // **Initialize Robot System**
  sendCANMessage(CAN1, CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  sendCANMessage(CAN1, CAN_ID_ENABLE, enableData, 8);
  delay(1000);
}

void loop() {
  uint32_t id;
  uint8_t len;
  uint8_t buf[8];

  static unsigned long lastHeartbeat = 0;
  unsigned long currentMillis = millis();

  // **Send Robot Heartbeat to Charger every 500ms**
  if (currentMillis - lastHeartbeat >= 500) {  
    sendCANMessage(CAN2, 0x01E101B0, heartbeatData, 8);
    lastHeartbeat = currentMillis;
  }

  // **Process Charger Messages**
  if (CAN2.checkReceive() == CAN_MSGAVAIL) {
    CAN2.readMsgBuf(&id, &len, buf);
    
    if (id == 0x860201B3) { // Charger Status Message
      isChargerConnected = (buf[0] == 0x01); // If charger detects contact
    }

    //sendCANMessage(CAN1, id, buf, len); // Forward charger status to Robot
  }

  // **Process Robot Messages**
  if (CAN1.checkReceive() == CAN_MSGAVAIL) {
    //Nothing to do for now
  }

  if (!isReceiving){
    // Periodically send movement commands via CAN
    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= 100) {
      sendCANMessage(CAN1, CAN_ID_MOVE, moveData, 8);
      lastSendTime = currentMillis;
    }
  }
}

// **I2C Receive Event for Speed Control**
void receiveEvents(int numBytes) {
  isReceiving = true;
  if (numBytes == 9) {
    union {
      byte byteArray[8];
      float floatArray[2];
    } data;

    Wire.read(); // Ignore the first byte (command byte)
    for (int i = 0; i < 8; i++) {
      if (Wire.available()) {
        data.byteArray[i] = Wire.read();
      }
    }
    linear_x = data.floatArray[0];
    angular_z = data.floatArray[1];
    Serial.print(linear_x);
    Serial.print(" ");
    Serial.println(angular_z);
    convertSpeedToCANData(linear_x, angular_z, moveData);

  } else if (numBytes == 1) {
    Wire.read();
  }
  isReceiving = false;
}


// **Send CAN Messages**
void sendCANMessage(MCP_CAN &CANBus, uint32_t id, uint8_t *data, uint8_t len) {
  if (CANBus.sendMsgBuf(id, 1, len, data) != CAN_OK) {
    // Handle error case if needed
  }
}

// **Convert Speed Data to CAN Format**
void convertSpeedToCANData(float speed_linear, float speed_angular, uint8_t *data) {
  int16_t fixedPointSpeedLinear = speed_linear * 1000;
  int16_t fixedPointSpeedAngular = speed_angular * 1000;
  data[0] = fixedPointSpeedLinear & 0xFF;
  data[1] = (fixedPointSpeedLinear >> 8) & 0xFF;
  data[4] = fixedPointSpeedAngular & 0xFF;
  data[5] = (fixedPointSpeedAngular >> 8) & 0xFF;
}
