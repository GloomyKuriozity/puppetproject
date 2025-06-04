#include <Wire.h>

#define L6360_ADDR 0x60  // Confirmed working I2C address
#define ENL_PLUS_PIN 7

uint8_t one, two;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(ENL_PLUS_PIN, OUTPUT);
  digitalWrite(ENL_PLUS_PIN, HIGH);  // Turn on L+ line to power sensor

  Serial.println("== IO-Link Wake-Up Sequence ==");

  writeRegister(0x02, 0x49);  // COM2
  delay(20);
  
  // WURQ
  writeRegister(0x10, 0x01);
  delayMicroseconds(600);
  writeRegister(0x10, 0x00);
  delay(100);  
  
  Serial.print("COM_CHANNEL_STATUS: 0x");
  Serial.println(readRegister(0x11), HEX);
  
  // PREOPERATE
  writeRegister(0x0B, 0x02);
  delay(10);
  
  // Status read loop
  for (int i = 0; i < 10; i++) {
    uint8_t state = readRegister(0x0C);
    uint8_t frame = readRegister(0x19);
    Serial.print("IOL_STATE_STATUS: 0x");
    Serial.print(state, HEX);
    Serial.print(" | FRAME_STATUS: 0b");
    Serial.println(frame, BIN);
    delay(100);
  }

}



void loop() {
}



void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(L6360_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(L6360_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(L6360_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}
