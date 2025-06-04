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

uint8_t activateData[8] = {0x01, 0xE1, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};   // Data for activating
uint8_t enableData[8] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};     // Data for enabling the device and setting CAN control mode
uint8_t moveData[8] = {0xF1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};       // Data for moving

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
  Serial.begin(115200);

  // Initialize I2C for communication with RPi5
  Wire.begin(I2C_SLAVE_ADDRESS); // Join I2C bus with address #8
  Wire.onReceive(receiveEvents);
  
  // Initialize SPI for communication with Raspberry Pi
  SPI.begin();
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, LOW); // Set CS pin high to deselect the Arduino/Robot
  pinMode(SPI_CS0_PIN, OUTPUT);
  digitalWrite(SPI_CS0_PIN, HIGH); // Set CS0 pin high to deselect the Arduino/Raspberry Pi
  
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
}

void loop() {
  // Send CAN message if new velocity data is received
  if (linear_x != 0.0 || angular_z != 0.0) {
    sendCANMessage(CAN_ID_MOVE, moveData, 8);
  }
  
  //receiveCANMessage();
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
      checkOdometry(canId, buf);
    }
  }
}

void receiveEvents(int numBytes) {
  Serial.print("Received bytes: ");
  Serial.println(numBytes);

  if (numBytes == 9) {
    // Discard the first byte
    byte discard = Wire.read();
    Serial.print("Discarding first byte: ");
    Serial.println(discard, HEX);
    numBytes--;
  }

  if (numBytes == 8) { // Ensure we have exactly 8 bytes for two floats
    union {
      byte byteArray[8];
      float floatArray[2];
    } data;

    // Read the 8 bytes from the I2C buffer
    for (int i = 0; i < 8; i++) {
      if (Wire.available()) {
        data.byteArray[i] = Wire.read();
        Serial.print(data.byteArray[i], HEX);
        Serial.print(" ");
      }
    }
    Serial.println();

    // Extract the float values
    linear_x = data.floatArray[0];
    angular_z = data.floatArray[1];

    Serial.print("linear_x: ");
    Serial.println(linear_x);
    Serial.print("angular_z: ");
    Serial.println(angular_z);

    convertSpeedToCANData(linear_x, angular_z, moveData);
  } else {
    Serial.println("Error: Incorrect data length");
  }
}


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

void checkOdometry(unsigned long canId, unsigned char* buf) {
  int32_t left_wheel_odometry = 0;
  int32_t right_wheel_odometry = 0;

  if (canId == 0xB3) {
    left_wheel_odometry = (int32_t)buf[0] | ((int32_t)buf[1] << 8) | ((int32_t)buf[2] << 16) | ((int32_t)buf[3] << 24);
    right_wheel_odometry = (int32_t)buf[4] | ((int32_t)buf[5] << 8) | ((int32_t)buf[6] << 16) | ((int32_t)buf[7] << 24);

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    float left_odom_m = left_wheel_odometry / 1000.0;
    float right_odom_m = right_wheel_odometry / 1000.0;

    float distance = (left_odom_m + right_odom_m) / 2.0;
    float delta_theta = (right_odom_m - left_odom_m) / WHEEL_BASE;

    x += distance * cos(theta + delta_theta / 2.0);
    y += distance * sin(theta + delta_theta / 2.0);
    theta += delta_theta;

    // Publish or use the odometry data
    publishOdometry(x, y, theta, distance / deltaTime, delta_theta / deltaTime);
  }
}

void publishOdometry(float x, float y, float theta, float linear_velocity, float angular_velocity) {
  // Pack the data into a byte array
  byte data[20];
  memcpy(data, &x, sizeof(x));
  memcpy(data + 4, &y, sizeof(y));
  memcpy(data + 8, &theta, sizeof(theta));
  memcpy(data + 12, &linear_velocity, sizeof(linear_velocity));
  memcpy(data + 16, &angular_velocity, sizeof(angular_velocity));
  
  // Send the data via SPI
  digitalWrite(SPI_CS_PIN, HIGH); // Deselect the Robot
  digitalWrite(SPI_CS0_PIN, LOW); // Select the Raspberry Pi
  for (int i = 0; i < 20; i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(SPI_CS0_PIN, HIGH); // Deselect the Raspberry Pi
  digitalWrite(SPI_CS_PIN, LOW); // Select the Robot

  // For debugging purposes, print the data to Serial
  Serial.print("Position: (");
  Serial.print(x, 2);
  Serial.print(", ");
  Serial.print(y, 2);
  Serial.print(") Orientation: ");
  Serial.print(theta, 2);
  Serial.print(" Linear Velocity: ");
  Serial.print(linear_velocity, 2);
  Serial.print(" m/s Angular Velocity: ");
  Serial.print(angular_velocity, 2);
  Serial.println(" rad/s");
}
