#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>

#define I2C_SLAVE_ADDRESS 11

const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);

// CAN IDs
const uint32_t CAN_ID_MOVE = 0x01E10112;    // CAN ID for movement control
const uint32_t CAN_ID_ACTIVATE = 0x01E10103;    // CAN ID for movement control
const uint32_t CAN_ID_ENABLE = 0x01E10111;    // CAN ID for movement control
const uint32_t CAN_ID_RESET = 0x01E10105;

uint8_t resetData[8] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t moveData[8] = {0xF4, 00, 00, 00, 00, 00, 00, 00};  // Data for moving
uint8_t enableData[8] = {0x02, 00, 00, 00, 00, 00, 00, 00};  // Data for moving
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 00, 00, 00, 00};  // Data for moving

float linear_x = 0.0;
float angular_z = 0.0;

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS); // Join I2C bus with address #8
  Wire.onReceive(receiveEvents);

  Serial.begin(9600);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed.");
    while (1);
  }

  CAN.setMode(MCP_NORMAL); // switch to NORMAL mode
  delay(1000);

  /*sendCANMessage(CAN_ID_RESET, resetData, 8);
  Serial.println("ROBOT RESET");
  delay(1000);*/
  
  sendCANMessage(CAN_ID_ACTIVATE, activateData, 8);
  Serial.println("ROBOT ACTIVATED");
  delay(1000);
  
  sendCANMessage(CAN_ID_ENABLE, enableData,8);
  Serial.println("ROBOT CONTROL ENABLE");
  delay(1000);
  
  
}

void loop() {
  sendCANMessage(CAN_ID_MOVE, moveData, 8);
  delay(100);
  // Send CAN message if new data is received
  if (linear_x != 0.0 || angular_z != 0.0) {
    sendCANMessage(CAN_ID_MOVE, moveData, 8);
    Serial.println("Sent!");
  }
}

void receiveEvents(int numBytes) {
  Serial.println("Hello!'
  if (numBytes == 8) { // Each float is 4 bytes, so we need 8 bytes for two floats
    union {
      byte byteArray[8];
      float floatArray[2];
    } data;

    // Read the 8 bytes from the I2C buffer
    for (int i = 0; i < 8; i++) {
      if (Wire.available()) {
        data.byteArray[i] = Wire.read();
      }
    }

    // Extract the float values
    linear_x = data.floatArray[0];
    angular_z = data.floatArray[1];

    convertSpeedToCANData(linear_x, angular_z, moveData);
  }
}

// Function to convert speed in m/s to CAN data format
void convertSpeedToCANData(float speed_linear, float speed_angular, uint8_t *data) {
  int16_t fixedPointSpeedLinear = speed_linear * 100;
  int16_t fixedPointSpeedAngular = speed_angular * 100;

  data[0] = fixedPointSpeedLinear & 0xFF;          // lower byte
  data[1] = (fixedPointSpeedLinear >> 8) & 0xFF;   // higher byte
  data[2] = 0;
  data[3] = 0;
  data[4] = fixedPointSpeedAngular & 0xFF;   // lower byte
  data[5] = (fixedPointSpeedAngular >> 8) & 0xFF; // higher byte
  data[6] = 0;
  data[7] = 0;
}

void sendCANMessage(uint32_t id, uint8_t *data, uint8_t len) {
  if (CAN.sendMsgBuf(id, 1, len, data) != CAN_OK) {
    Serial.println("Error sending message");
  }
}
