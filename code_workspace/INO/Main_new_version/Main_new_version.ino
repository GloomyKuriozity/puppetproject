#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/wdt.h>  // Watchdog timer library

// CAN IDs
const uint32_t CAN_ID_MOVE = 0x01E10112;
const uint32_t CAN_ID_RESET = 0x01E10105;
const uint32_t CAN_ID_ACTIVATE = 0x01E10103;
const uint32_t CAN_ID_ENABLE = 0x01E10111;

// CAN data
uint8_t resetData[8] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t enableData[8] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t moveData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Constants
#define I2C_SLAVE_ADDRESS 11
#define CAN0_INT 2
#define SPI_CS_PIN  53
#define WHEEL_BASE  0.76
MCP_CAN CAN(SPI_CS_PIN);

// Variables
float x = 0.0, y = 0.0, theta = 0.0;
float prevleft = 0, prevright = 0;
unsigned long prevTime = 0;
float linear_velocity = 0.0, angular_velocity = 0.0;
float linear_x = 0.0, angular_z = 0.0;
volatile bool isReceiving = false;

void setup() {
  //Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvents);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    delay(1000);
    //Serial.println("CAN init failed, retrying...");
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

  sendCANMessage(CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  sendCANMessage(CAN_ID_ENABLE, enableData, 8);
  delay(1000);

  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();
  if (!isReceiving) {
    static unsigned long lastSendTime = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= 100) {
      sendCANMessage(CAN_ID_MOVE, moveData, 8);
      lastSendTime = currentMillis;
    }
  }
}

void sendCANMessage(uint32_t id, uint8_t *data, uint8_t len) {
  if (CAN.sendMsgBuf(id, 1, len, data) != CAN_OK) {
    delay(1);
    //Serial.println("Error sending CAN message");
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
    for (int i = 0; i < 8; i++) {
      if (Wire.available()) {
        data.byteArray[i] = Wire.read();
      }
    }

    linear_x = data.floatArray[0];
    angular_z = data.floatArray[1];
    convertSpeedToCANData(linear_x, angular_z, moveData);

  } else if (numBytes == 1) {
    Wire.read();
  }
  isReceiving = false;
}

void convertSpeedToCANData(float speed_linear, float speed_angular, uint8_t *data) {
  int16_t fixedPointSpeedLinear = speed_linear * 1000;
  int16_t fixedPointSpeedAngular = speed_angular * 1000;
  data[0] = fixedPointSpeedLinear & 0xFF;
  data[1] = (fixedPointSpeedLinear >> 8) & 0xFF;
  data[4] = fixedPointSpeedAngular & 0xFF;
  data[5] = (fixedPointSpeedAngular >> 8) & 0xFF;
}
