#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Wire.h>

OpenLog myLog;
SFE_UBLOX_GPS myGPS;

uint8_t i2cBuffer[1024];   // Buffer for SD card writes
size_t bufferPointer = 0; // Size of i2cBuffer pointer for SD card writes

void setup() {
  Serial.begin(115200);
  delay(10000);
  Wire.begin();
  myLog.begin();
  myLog.append("test.ubx");
  if (myGPS.begin() == false) {
    Serial.println(F("Warning: u-blox ZED-F9P not detected at default I2C address. Please check wiring. Halting."));
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }
  boolean setValueSuccess = true;
  setValueSuccess &= myGPS.setVal8(0x10720001, 0x01); // Enable UBX output protocol on I2C
  setValueSuccess &= myGPS.setVal8(0x209102a4, 0x01); // Enable UBX-RXM-RAWX output on I2C port
  setValueSuccess &= myGPS.setVal8(0x20910231, 0x01); // Enable UBX-RXM-SFRBX output on I2C port
  setValueSuccess &= myGPS.setVal16(0x30210001, 1000); // Set time between GNSS measurements (CFG-RATE-MEAS)
  if (setValueSuccess == false)
    Serial.println("Warning: u-blox ZED-F9P not configured.");
}

void loop() {
  uint16_t bytesAvailable = 0;
  Wire.beginTransmission(0x42);
  Wire.write(0xFD);
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 2);
  if (Wire.available()) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    bytesAvailable = ((uint16_t)msb << 8) | lsb;
    if (lsb == 0xFF)
      bytesAvailable = 0;
  }
  if (bytesAvailable & ((uint16_t)1 << 15))
    bytesAvailable &= ~((uint16_t)1 << 15);
  while (bytesAvailable) {
    Wire.beginTransmission(0x42);
    Wire.write(0xFF);
    Wire.endTransmission(false);
    uint16_t bytesToRead = bytesAvailable;
    if (bytesToRead > 32)
      bytesToRead = 32;
    Serial.printf("\n"); // Serial Monitor newline
TRY_AGAIN:
    Wire.requestFrom(0x42, (uint8_t)bytesToRead);
    if (Wire.available()) {
      for (uint16_t x = 0; x < bytesToRead; x++) {
        uint8_t c = Wire.read();
        if (x == 0) {
          if ((c == 0x7F) || (c == 0xFF)) {
            delay(2);
            goto TRY_AGAIN;
          }
        }
        Serial.printf("%02X ", c); // Echo to Serial Monitor
        i2cBuffer[bufferPointer] = c; // Write bytes to i2cBuffer
        bufferPointer++;
        if (bufferPointer == 1024) { // Sync every 512 bytes
          digitalWrite(LED_BUILTIN, HIGH);
          for (uint16_t i = 0; i < bufferPointer; i++)
            myLog.write(i2cBuffer[i]);
          myLog.syncFile();
          digitalWrite(LED_BUILTIN, LOW);
          bufferPointer = 0;
        }
      }
      bytesAvailable -= bytesToRead;
    }
    delay(5);
  }
}
