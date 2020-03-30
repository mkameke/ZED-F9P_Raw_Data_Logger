/*
  Title:    ZED-F9P Raw Data Logger
  Date:     March 30, 2020
  Author:   Adam Garbo

  Description:
  - Intended to read data from I2C and stream it directly to storage as quickly as possible.
  - Code is based on SparkFun_Ublox_Arduino_Library's checkUbloxI2C() function

  Components:
  - SparkFun Edge2 (Artemis)
  - SparkFun GPS-RTK2 Board ZED-F9P

  Comments:
  - Functionality is currently limited to streaming UBX data to the Serial Monitor
*/

#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Wire.h>

OpenLog myLog;
SFE_UBLOX_GPS myGPS;

uint8_t i2cBuffer[2048];  // Buffer for SD card writes
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
  setValueSuccess &= myGPS.setVal8(0x209102a4, 0x01); // Enable UBX-RXM-RAWX output on I2C
  setValueSuccess &= myGPS.setVal8(0x20910231, 0x01); // Enable UBX-RXM-SFRBX output on I2C
  setValueSuccess &= myGPS.setVal16(0x30210001, 1000); // Set time between GNSS measurements (CFG-RATE-MEAS)
  if (setValueSuccess == false)
    Serial.println("Warning: u-blox ZED-F9P not configured.");
}

void loop() {
  uint16_t bytesAvailable = 0;
  Wire.beginTransmission(0x42);
  Wire.write(0xFD); // Number of available bytes can be read at addresses 0xFD (High Byte) and 0xFE (Low Byte)
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 2); // Request number of available bytes in the message stream
  if (Wire.available()) {
    uint8_t msb = Wire.read(); // High Byte (0xFD)
    uint8_t lsb = Wire.read(); // Low Byte (0xFE)
    bytesAvailable = ((uint16_t)msb << 8) | lsb; // Bitwise operators

    // See issue: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/37
    if (lsb == 0x7F) {
      //bytesAvailable = 0;
    }

    // See issue: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/38
    if (lsb == 0xFF) {
      bytesAvailable = 0;
    }
  }
  // See issue: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
  if (bytesAvailable & ((uint16_t)1 << 15)) {
    bytesAvailable &= ~((uint16_t)1 << 15);
  }

  while (bytesAvailable) {
    Wire.beginTransmission(0x42);
    Wire.write(0xFF); // Read data stream from register 0xFF. Delivers 0xFF if no data awaiting transmission.
    Wire.endTransmission(false);
    uint16_t bytesToRead = bytesAvailable;
    if (bytesToRead > 32) { // I2C buffer size: 32 bytes
      bytesToRead = 32;
    }
    //Serial.printf("\n"); // Serial Monitor newline
TRY_AGAIN:
    Wire.requestFrom(0x42, (uint8_t)bytesToRead);
    if (Wire.available()) {
      for (uint16_t x = 0; x < bytesToRead; x++) {
        uint8_t c = Wire.read(); // Read data stream
        if (x == 0) {
          // 0x7F indicates receiver is not ready to respond
          // 0xFF cannot be the first byte of a valid message
          if ((c == 0x7F) || (c == 0xFF)) {
            delay(2);
            goto TRY_AGAIN;
          }
        }
        //Serial.printf("%02X ", c); // Echo to Serial Monitor
        i2cBuffer[bufferPointer] = c; // Write bytes to i2cBuffer
        bufferPointer++;
      }
      bytesAvailable -= bytesToRead;
    }
    // Sync every >512 bytes
    if (bufferPointer > 512) {
      digitalWrite(LED_BUILTIN, HIGH);
      //myLog.write(i2cBuffer, bufferPointer); // Not working
      for (uint16_t i = 0; i < bufferPointer; i++)
        myLog.write(i2cBuffer[i]);
      myLog.syncFile();
      digitalWrite(LED_BUILTIN, LOW);
      bufferPointer = 0;
    }
    delay(5);
  }
  delay(5);
}
