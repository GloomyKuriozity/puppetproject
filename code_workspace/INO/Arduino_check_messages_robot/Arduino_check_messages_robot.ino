#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <math.h>

#define I2C_SLAVE_ADDRESS 11 // Address that the I2C communication uses
#define SPI_CS_PIN 53        // Chip Select Pin for Robot/Arduino
#define SPI_CS0_PIN 10       // Chip Select Pin for Arduino/RPi5
#define CAN_INT 2            // CAN INT Robot/Arduino

MCP_CAN CAN(SPI_CS_PIN);

// CAN IDs
const uint32_t CAN_ID_ACTIVATE = 0x81E10103;   // CAN ID for activate device
const uint32_t CAN_ID_ENABLE = 0x01E10111;     // CAN ID for enabling device and setting control mode
const uint32_t CAN_ID_MOVE = 0x01E10112;       // CAN ID for sending speed
const uint32_t CAN_ID_CHARGE = 0x01E10113;     // CAN ID for charger
const uint32_t CAN_ID_CLEARSTATUS = 0x01E10104;

uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};   // Data for activating
uint8_t enableData[8] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};     // Data for enabling the device and setting CAN control mode
uint8_t moveData[8] = {0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};       // Data for moving
uint8_t chargeData[8] = {0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x00};
uint8_t clearstatusData[8] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Information about the robot
const float WHEEL_BASE = 552.0;                                    // Distance between wheels R/L in mm
const float WHEEL_D = 172.0;                                       // Diameter of wheels in mm
const float RES = 4096.0;                                          // Ticks per revolution _ encoder resolution

// Speeds, position and orientation tokens
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float linear_x = 0.0;
float angular_z = 0.0;

// Time
unsigned long lastTime = 0;

void setup() {
  // Initialize Serial for debug
  Serial.begin(9600);
  
  // Initialize CAN
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN bus initialization failed. Retry...");
  } 
  Serial.println("CAN bus initialized successfully.");
  CAN.setMode(MCP_NORMAL); // Switch to NORMAL mode
  delay(1000); 

  // Initialize robot
  sendCANMessage(CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  sendCANMessage(CAN_ID_ENABLE, enableData, 8);
  delay(1000); 
  sendCANMessage(CAN_ID_CLEARSTATUS, clearstatusData, 8);
  delay(1000);
  sendCANMessage(CAN_ID_CHARGE, chargeData, 8);
  delay(1000);
}

void loop() {
  /*
  sendCANMessage(CAN_ID_ACTIVATE, activateData, 8);
  delay(1000);
  sendCANMessage(CAN_ID_ENABLE, enableData, 8);
  delay(1000); 
  sendCANMessage(CAN_ID_MOVE, moveData, 8);*/
  receiveCANMessage();
}

void sendCANMessage(uint32_t id, uint8_t *data, uint8_t len) {
  if (CAN.sendMsgBuf(id, 1, len, data) != CAN_OK) {
    Serial.println("Error sending message");
  }
}

void receiveCANMessage() {
  unsigned char len = 0;
  unsigned long canId;
  unsigned char buf[8];

  if (CAN.checkReceive() == CAN_MSGAVAIL) {  
    if (CAN.readMsgBuf(&canId, &len, buf) == CAN_OK) {
      Serial.print("Message for ID ");
      Serial.print(canId,HEX);
      Serial.print(" : { ");
      for(int i = 0; i<8; i++){
        Serial.print(buf[i],HEX);
        Serial.print(" ");
      }
    }
    Serial.println(" }");
  }
}
