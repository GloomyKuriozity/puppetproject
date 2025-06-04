#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>

#define I2C_SLAVE_ADDRESS 11

const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);
#define CAN0_INT 2

// CAN IDs
const uint32_t CAN_ID_MOVE = 0x01E10112;    // CAN ID for movement control
const uint32_t CAN_ID_ACTIVATE = 0x01E10103;    // CAN ID for activation
const uint32_t CAN_ID_ENABLE = 0x01E10111;    // CAN ID for enabling

uint8_t moveData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Data for moving
uint8_t enableData[8] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};  // Data for enabling
uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};  // Data for activation

float linear_x = 0.0;
float angular_z = 0.0;

volatile bool isReceiving = false; // Flag to indicate if data is being received

void setup() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(receiveEvents);

  Serial.begin(115200);
  
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed.");
    while (1);
  }

  CAN.setMode(MCP_NORMAL); // switch to NORMAL mode
  pinMode(CAN0_INT, INPUT);
  delay(1000); 
  
  sendCANMessage(CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  
  sendCANMessage(CAN_ID_ENABLE, enableData, 8);
  delay(1000);
}

void loop() {
  if (!isReceiving) {
    // Continuously send moveData
    sendCANMessage(CAN_ID_MOVE, moveData, 8);
    delay(100); // Add a small delay to avoid overwhelming the CAN bus
  }
}

void receiveEvents(int numBytes) {
  isReceiving = true; // Set flag to indicate data reception
  Serial.print("Hello! Received ");
  Serial.print(numBytes);
  Serial.println(" bytes");

  if (numBytes == 9) { // Each float is 4 bytes, so we need 8 bytes for two floats plus the command byte
    union {
      byte byteArray[8];
      float floatArray[2];
    } data;

    // Read and ignore the first byte (command byte)
    Wire.read();
    
    // Read the 8 bytes from the I2C buffer
    for (int i = 0; i < 8; i++) {
      if (Wire.available()) {
        data.byteArray[i] = Wire.read();
      }
    }

    // Extract the float values
    linear_x = data.floatArray[0];
    angular_z = data.floatArray[1];

    /*Serial.print("Received linear_x: ");
    Serial.println(linear_x);
    Serial.print("Received angular_z: ");
    Serial.println(angular_z);*/

    convertSpeedToCANData(linear_x, angular_z, moveData);
  } else {
    // Handle error or other cases
    Serial.println("Error: Expected 9 bytes of data");
    
    /*// Print the received bytes for debugging
    while (Wire.available()) {
      byte b = Wire.read();
      Serial.print("Byte received: ");
      Serial.println(b);
    }*/
  }
  // Clear flag after processing the data
  isReceiving = false;
}

// Function to convert speed in m/s to CAN data format
void convertSpeedToCANData(float speed_linear, float speed_angular, uint8_t *data) {
  int16_t fixedPointSpeedLinear = speed_linear * 1000;
  int16_t fixedPointSpeedAngular = speed_angular * 1000;

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
