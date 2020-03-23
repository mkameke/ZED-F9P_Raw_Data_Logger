/*
  Title:    ZED-F9P Raw Data Logger
  Date:     March 23, 2020
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

// Libraries
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Wire.h>

// Object instantiations
SFE_UBLOX_GPS myGPS;

// Defined constants
#define I2C_ADDRESS       0x42
#define I2C_BUFFER_LENGTH 32
#define DEBUG             true // Output debugging messages to Serial Monitor
#define DEBUG_UBX         true // Echo UBX data to Serial Monitor

// Global variable declarations
uint32_t  previousMillis  = 0;
uint32_t  i2cPollingWait  = 100;

void setup() {

  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(100000);

  if (myGPS.begin() == false) {
    Serial.println(F("Warning: u-blox ZED-F9P not detected at default I2C address. Please check wiring. Halting."));
    while (1);
  }
  boolean setValueSuccess = true;
  setValueSuccess &= myGPS.setVal8(0x10720001, 1); // Enable UBX output protocol on I2C
  setValueSuccess &= myGPS.setVal8(0x209102a4, 1); // Enable UBX-RXM-RAWX output on I2C port
  setValueSuccess &= myGPS.setVal8(0x20910231, 1); // Enable UBX-RXM-SFRBX output on I2C port
  setValueSuccess &= myGPS.setVal16(0x30210001, 1000); // Set time between GNSS measurements (CFG-RATE-MEAS)
  if (setValueSuccess == false) {
    Serial.println("Warning: u-blox ZED-F9P not configured.");
  }
}

// Read I2C bus and echo UBX data to Serial Monitor
void loop() {

  uint32_t currentMillis = millis();
  if (millis() - previousMillis >= i2cPollingWait) {
    previousMillis = currentMillis;

    uint16_t bytesAvailable = 0;
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0xFD); // Number of available bytes in the message stream can be read at addresses 0xFD (High Byte) and 0xFE (Low Byte).
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDRESS, 2); // Request number of available bytes in the message stream

    if (Wire.available()) {
      uint8_t msb = Wire.read(); // High Byte (0xFD)
      uint8_t lsb = Wire.read(); // Low Byte (0xFE)
      bytesAvailable = ((uint16_t)msb << 8) | lsb; // Bitwise operators

      // Possible u-blox bug. Device should never present an 0xFF.
      if (lsb == 0xFF) {
#if DEBUG
        Serial.println("Error: Low Byte = 0xFF");
#endif
        bytesAvailable = 0;
        previousMillis = millis(); // Delay to avoid I2C bus traffic
      }
    }

    if (bytesAvailable == 0) {
#if DEBUG
      Serial.printf("%d bytes available\n", bytesAvailable);
#endif
      previousMillis = millis(); // Delay to avoid I2C bus traffic
    }

    // Check for undocumented bit error that incorrectly interprets the first
    // bit of the two 'data available' bytes as 1, resuting in far too many bytes to check.
    // Related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15)) {
      bytesAvailable &= ~((uint16_t)1 << 15); // Clear the MSB
#if DEBUG
      Serial.printf("Error: Undocumented bit error. Bytes available: %d\n", bytesAvailable);
#endif
    }
#if DEBUG
    if (bytesAvailable > 100) {
      Serial.printf("Large packet of %d bytes received\n", bytesAvailable);
    }
    else {
      Serial.printf("Reading %d bytes\n", bytesAvailable);
    }
#endif

    while (bytesAvailable) {
      Wire.beginTransmission(I2C_ADDRESS);
      Wire.write(0xFF); // Register 0xFF allows the data stream to be read. This register will deliver the value 0xFF if no data is awaiting transmission.
      Wire.endTransmission(false);

      // Limit to available I2C buffer (i.e. 32 bytes)
      uint16_t bytesToRead = bytesAvailable;
      if (bytesToRead > I2C_BUFFER_LENGTH) {
        bytesToRead = I2C_BUFFER_LENGTH;
      }

TRY_AGAIN:

      Wire.requestFrom(I2C_ADDRESS, (uint8_t)bytesToRead);

      if (Wire.available()) {
        for (uint16_t x = 0; x < bytesToRead; x++) {
          uint8_t dataStream = Wire.read(); // Read data stream

          // If the first read is 0x7F the module is not ready to respond. Stop, wait, and try again.
          if (x == 0) {
            if (dataStream == 0x7F) {
#if DEBUG
              Serial.printf("Error: Module not ready with data\n");
#endif
              delay(5); // SparkFun notes a response time of 1.48 ms based on logic analyzation
              goto TRY_AGAIN;
            }
          }
#if DEBUG_UBX
          // Echo UBX data to Serial Monitor
          Serial.printf("%02X ", dataStream); // Process this valid character
#endif
        }
      }
      bytesAvailable -= bytesToRead;
    }
  }
}
