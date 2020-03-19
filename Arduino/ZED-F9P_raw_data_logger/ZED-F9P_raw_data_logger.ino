/*
  Title:    ZED-F9P Raw Data Logger
  Date:     March 19, 2020
  Author:   Adam Garbo

  Description:
  - Based on the u-blox ZED-F9P, this data logger records UBX-RAWX and UBX-SFRBX directly to 
  the Adafruit M0 Adalogger.

  Components:
  - Adafruit Feather M0 Adalogger
  - Adafruit DS3231 Precision RTC FeatherWing
  - SparkFun GPS-RTK2 Board ZED-F9P

  Comments:
  - This code is based heavily on Paul Clark's F9P_RAWX_Logger:
  https://github.com/PaulZC/ZED-F9P_FeatherWing_USB
  - The code has been streamlined and functionality to output raw UBX data to the Serial Monitor.
  - This code is currently being developed further to handle sync losses without needing to create new log files.
*/

// Libraries
#include <ArduinoLowPower.h>                // https://github.com/arduino-libraries/ArduinoLowPower
#include <DS3232RTC.h>                      // https://github.com/JChristensen/DS3232RTC
#include <SdFat.h>                          // https://github.com/greiman/SdFat
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <SPI.h>                            // https://www.arduino.cc/en/Reference/SPI
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire

// Debugging definitons
#define DEBUG                 true  // Debugging output to Serial Monitor
#define DEBUG_I2C             true  // I2C debugging output to Serial Monitor
#define DEBUG_NMEA            false // NMEA debugging output to Serial Monitor
#define DEBUG_SERIAL_BUFFER   false // Displays a message each time SerialBuffer.available reaches a new maximum
#define DEBUG_UBX             true  // UBX debugging output to Serial Monitor

// Pin definitions
#define LED_PIN       8
#define RTC_INT_PIN   5
#define SW_PIN        A0  // Press to halt logging and close the log file

// Instantiate objects
DS3232RTC           myRTC(false); // Tell constructor not to initialize the I2C bus
RingBufferN<16384>  SerialBuffer; // Define SerialBuffer as a RingBuffer of size 16k bytes
SdFat               sd;
SdFile              file;
SFE_UBLOX_GPS       myGPS;

// User-declared global variables and constants
const uint32_t  alarmInterval   = 60;   // Duration of log file before new file is created
const float     lowBattery      = 3.5;  // Low battery voltage threshold

// Declare global variables and constants
const size_t    sdPacket            = 512;    // SD packet size
const uint16_t  dwell               = 1000;   // Time to wait (in milliseconds) for residual data before closing the log file
const uint8_t   chipSelect          = 4;      // SD card chip select
volatile bool   alarmFlag           = false;  // RTC alarm interrupt service routine (ISR) flag
bool            stopPressed         = false;  // Flag to indicate if stop switch was pressed to halt logging
char            dirName[9]          = {0};    // Log file directory name buffer (YYYYMMDD)
char            fileName[22]        = {0};    // Log file name format limited to 8.3 characters: YYYYMMDD/HHMMSS.ubx
char            dataString[4]       = {0};
float           voltage             = 0.0;    // Battery voltage
uint8_t         maxValFix           = 5;      // Maximum number of valid GNSS fixes to be collected
uint8_t         serBuffer[sdPacket];          // Buffer
uint8_t         valFix              = 0;      // GNSS valid fix counter
int16_t         maxBufferAvailable  = 0;      // SerialBuffer debugging
uint16_t        numBytes            = 0;      //
uint32_t        bytesWritten        = 0;      //
size_t          bufferPointer       = 0;      // Buffer pointer
time_t          t, alarmTime;
tmElements_t    tm;

// Global Variables for UBX/NMEA Parsing
const uint8_t ubxSyncChar1          = 0xB5;
const uint8_t ubxSyncChar2          = 0x62;
uint8_t       ubxClass              = 0;
uint8_t       ubxId                 = 0;
uint16_t      ubxLength             = 0;
uint8_t       ubxChecksumA          = 0;
uint8_t       ubxChecksumB          = 0;
uint8_t       ubxExpectedChecksumA  = 0;
uint8_t       ubxExpectedChecksumB  = 0;
uint8_t       nmeaAddress1          = '0'; // e.g. G
uint8_t       nmeaAddress2          = '0'; // e.g. P
uint8_t       nmeaAddress3          = '0'; // e.g. G
uint8_t       nmeaAddress4          = '0'; // e.g. G
uint8_t       nmeaAddress5          = '0'; // e.g. A
uint8_t       nmeaChecksum          = 0;
uint8_t       nmeaChecksum1         = '0';
uint8_t       nmeaChecksum2         = '0';
uint8_t       nmeaExpectedChecksum1 = '0';
uint8_t       nmeaExpectedChecksum2 = '0';
const uint8_t nmeaMaxLength         = 100; // Max NMEA message length used to detect if sync is lost

// Enumerated switch statements
enum LoopSwitch {
  START,
  START_UBX,
  OPEN_FILE,
  WRITE_FILE,
  NEW_FILE,
  CLOSE_FILE,
  SLEEP,
  WAKE
};

enum ParseSwitch {
  PARSE_UBX_SYNC_CHAR_1,
  PARSE_UBX_SYNC_CHAR_2,
  PARSE_UBX_CLASS,
  PARSE_UBX_ID,
  PARSE_UBX_LENGTH_LSB,
  PARSE_UBX_LENGTH_MSB,
  PARSE_UBX_PAYLOAD,
  PARSE_UBX_CHECKSUM_A,
  PARSE_UBX_CHECKSUM_B,
  PARSE_NMEA_START_CHAR,
  PARSE_NMEA_CHECKSUM_1,
  PARSE_NMEA_CHECKSUM_2,
  PARSE_NMEA_END_1,
  PARSE_NMEA_END_2,
  SYNC_LOST
};

// Default switch steps
LoopSwitch  loopStep  = START;
ParseSwitch parseStep = PARSE_UBX_SYNC_CHAR_1;

// Satellite systems (GNSS) signal configuration
// Enable GPS/GLO and disable GAL/BDS/QZSS
uint8_t configureGnss() {
  myGPS.newCfgValset8(0x1031001f, 0x01, VAL_LAYER_RAM); // CFG-SIGNAL-GPS_ENA
  myGPS.addCfgValset8(0x10310001, 0x01);                // CFG-SIGNAL-GPS_L1CA_ENA
  myGPS.addCfgValset8(0x10310003, 0x01);                // CFG-SIGNAL-GPS_L2C_ENA
  myGPS.addCfgValset8(0x10310021, 0x00);                // CFG-SIGNAL-GAL_ENA
  myGPS.addCfgValset8(0x10310007, 0x00);                // CFG-SIGNAL-GAL_E1_ENA
  myGPS.addCfgValset8(0x1031000a, 0x00);                // CFG-SIGNAL-GAL_E5B_ENA
  myGPS.addCfgValset8(0x10310022, 0x00);                // CFG-SIGNAL-BDS_ENA
  myGPS.addCfgValset8(0x1031000d, 0x00);                // CFG-SIGNAL-BDS_B1_ENA
  myGPS.addCfgValset8(0x1031000e, 0x00);                // CFG-SIGNAL-BDS_B2_ENA
  myGPS.addCfgValset8(0x10310024, 0x00);                // CFG-SIGNAL-QZSS_ENA
  myGPS.addCfgValset8(0x10310012, 0x00);                // CFG-SIGNAL-QZSS_L1CA_ENA
  myGPS.addCfgValset8(0x10310015, 0x00);                // CFG-SIGNAL-QZSS_L2C_ENA
  myGPS.addCfgValset8(0x10310025, 0x01);                // CFG-SIGNAL-GLO_ENA
  myGPS.addCfgValset8(0x10310018, 0x01);                // CFG-SIGNAL-GLO_L1_ENA
  return myGPS.sendCfgValset8(0x1031001a, 0x01);        // CFG-SIGNAL-GLO_L2_ENA
}

// Enable UBX RAWX and SFRBX messages on UART1
uint8_t enableUbx() {
  myGPS.newCfgValset8(0x209102a5, 0x01, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  return myGPS.sendCfgValset8(0x20910232, 0x01);        // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
}

// Disable UBX RAWX and SFRBX messages on UART1
uint8_t disableUbx() {
  myGPS.newCfgValset8(0x209102a5, 0x00, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  return myGPS.sendCfgValset8(0x20910232, 0x00);        // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
}

// Enable NMEA messages on UART1
uint8_t enableNmea() {
  myGPS.newCfgValset8(0x209100bb, 0x01, VAL_LAYER_RAM); // CFG-MSGOUT-NMEA_ID_GGA_UART1
  myGPS.addCfgValset8(0x209100ca, 0x01);                // CFG-MSGOUT-NMEA_ID_GLL_UART1
  myGPS.addCfgValset8(0x209100c0, 0x01);                // CFG-MSGOUT-NMEA_ID_GSA_UART1
  myGPS.addCfgValset8(0x209100c5, 0x01);                // CFG-MSGOUT-NMEA_ID_GSV_UART1
  myGPS.addCfgValset8(0x209100b1, 0x01);                // CFG-MSGOUT-NMEA_ID_VTG_UART1
  myGPS.addCfgValset8(0x209100ac, 0x01);                // CFG-MSGOUT-NMEA_ID_RMC_UART1
  return myGPS.sendCfgValset8(0x209100b1, 0x07);        // CFG-INFMSG-NMEA_UART1
}

// Disable NMEA messages on UART1
uint8_t disableNmea() {
  myGPS.newCfgValset8(0x209100bb, 0x00, VAL_LAYER_RAM); // CFG-MSGOUT-NMEA_ID_GGA_UART1
  myGPS.addCfgValset8(0x209100ca, 0x00);                // CFG-MSGOUT-NMEA_ID_GLL_UART1
  myGPS.addCfgValset8(0x209100c0, 0x00);                // CFG-MSGOUT-NMEA_ID_GSA_UART1
  myGPS.addCfgValset8(0x209100c5, 0x00);                // CFG-MSGOUT-NMEA_ID_GSV_UART1
  myGPS.addCfgValset8(0x209100b1, 0x00);                // CFG-MSGOUT-NMEA_ID_VTG_UART1
  myGPS.addCfgValset8(0x209100ac, 0x00);                // CFG-MSGOUT-NMEA_ID_RMC_UART1
  return myGPS.sendCfgValset8(0x209100b1, 0x00);        // CFG-INFMSG-NMEA_UART1
}


//*****************************************************************************
//
// Setup
//
//*****************************************************************************
void setup() {

  // Pin configuration
  pinMode(RTC_INT_PIN, INPUT_PULLUP); // Configure interrupt on SQW/INT pin
  pinMode(SW_PIN, INPUT_PULLUP);      // Configure input for stop switch
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Initialize the RTC
  myRTC.begin();                                // Initialize the I2C bus
  myRTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);  // Initialize alarms to known values
  myRTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  myRTC.alarm(ALARM_1);                         // Clear alarm flags
  myRTC.alarm(ALARM_2);
  myRTC.alarmInterrupt(ALARM_1, false);         // Clear alarm interrupt flags
  myRTC.alarmInterrupt(ALARM_2, false);
  myRTC.squareWave(SQWAVE_NONE);                // Configure SQW/INT pin for interrupt operation (disable square wave output)

  // Attach a wakeup interrupt that triggers on the falling edge
  LowPower.attachInterruptWakeup(RTC_INT_PIN, alarmIsr, FALLING);

  // Start Serial at 115200 baud
  Serial.begin(115200);
  //while (!Serial);    // Wait for user to open Serial Monitor
  delay(5000);       // Allow 10 seconds for user to open Serial Monitor

  // Print current date and time
  printDateTime(myRTC.get());

  Serial.println();
  Serial.println("u-blox ZED-F9P Raw Datalogger");
  Serial.println("-----------------------------");

  // I2C Configuration
  Wire.begin();
  Wire.setClock(400000);  // Increase clock frequency for I2C communications to 400 kHz

  // Initialize the SD card
  Serial.println("Initializing SD card...");
  if (sd.begin(chipSelect, SD_SCK_MHZ(4))) {
    Serial.println("SD card initialized.");
  }
  else {
    Serial.println("Unable to initialize SD card. Halting.");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    while (1); // Halt the program
  }

  // u-blox ZED-F9P Configuration
  Serial.println("Initializing u-blox ZED-F9P...");
  //Connect to the u-blox ZED-F9P using the Wire (I2C) interface
  if (myGPS.begin() == true) {
    Serial.println("u-blox ZED-F9P detected");
#if DEBUG_I2C
    myGPS.enableDebugging(); // Enable I2C debug messages over Serial
#endif
  }
  else {
    Serial.println("u-blox ZED-F9P not detected at default I2C address. Please check wiring. Halting.");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    while (1); // Halt the program
  }

  // Receiver configuration messages:
  // Message Acknowledged:      UBX-ACK-ACK (0x05 0x01)
  // Message Not-Acknowledged:  UBX-ACK-NAK (0x05 0x00)
  // Payload:                   UBX-CFG-VALSET (0x06 0x8A)
  bool setValueSuccess = true;
  setValueSuccess &= myGPS.setVal8(0x10720002, 0);        // Disable NMEA as an output protocol on I2C      (CFG-I2COUTPROT-NMEA)
  setValueSuccess &= myGPS.setVal32(0x40520001, 230400);  // Set UART1 baud rate to 230400                  (CFG-UART1-BAUDRATE)
  setValueSuccess &= myGPS.setVal8(0x20110021, 2);        // Set the dynamic platform model to stationary   (CFG-NAVSPG-DYNMODEL)
  setValueSuccess &= myGPS.setVal16(0x30210001, 1000);    // Set time between GNSS measurements to 1000 ms  (CFG-RATE-MEAS)
  setValueSuccess &= configureGnss();                     // Configure GNSS signals
  setValueSuccess &= disableUbx();                        // Disable RAWX messages on UART1
  setValueSuccess &= disableNmea();                       // Disable NMEA messages on UART1

  if (setValueSuccess == true) {
    Serial.println("u-blox ZED-F9P initialized.");
  }
  else {
    Serial.println("Unable to initialize u-blox ZED-F9P. Halting.");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    while (1); // Halt the program
  }

  // Start Serial1 at 230400 baud.
  Serial1.begin(230400);

  // Wait for GNSS fix.
  Serial.println("Waiting for GNSS fix...");
}

//*****************************************************************************
//
// Loop
//
//*****************************************************************************
void loop() {

  switch (loopStep) {

    case START: {

#if DEBUG
        char gnssDateTime[20]; // GNSS date time buffer
        snprintf(gnssDateTime, sizeof(gnssDateTime), "%04u-%02d-%02d %02d:%02d:%02d",
                 myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(),
                 myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());

        int32_t latitude = myGPS.getLatitude();
        int32_t longitude = myGPS.getLongitude();
        float pdop = myGPS.getPDOP() / 100.0;
        uint8_t fix = myGPS.getFixType();
        uint8_t satellites = myGPS.getSIV();

        Serial.print("RTC: "); printDateTime(myRTC.get());
        Serial.print(" GNSS: "); Serial.print(gnssDateTime);
        Serial.print(" LAT: "); Serial.print(latitude);
        Serial.print(" LON: "); Serial.print(longitude);
        Serial.print(" SAT: "); Serial.print(satellites);
        Serial.print(" FIX: "); Serial.print(fix);
        Serial.print(" PDOP: "); Serial.println(pdop, 2);
#endif

        // Do we have a GNSS fix?
        if (myGPS.getFixType() > 0) {
          // Increment valFix and cap at maxValFix
          // Do not decrement valFix as logging will continue even if fix is lost
          valFix += 1;
          if (valFix > maxValFix) {
            valFix = maxValFix;
          }
        }

        // Have enough valid GNSS fixes been collected?
        if (valFix == maxValFix) {
          valFix = 0; // Reset GNSS fix counter

          // Set the RTC using GNSS date and time
          tm.Second = myGPS.getSecond();
          tm.Minute = myGPS.getMinute();
          tm.Hour   = myGPS.getHour();
          tm.Day    = myGPS.getDay();
          tm.Month  = myGPS.getMonth();
          tm.Year   = (myGPS.getYear() - 1970); // tmElements_t.Year is the offset from 1970
          t = makeTime(tm);                     // Change tm structure into time_t (seconds since epoch)
          myRTC.set(t);                         // Set RTC time

          Serial.print("RTC set: ");
          printDateTime(t);
          Serial.println();

          // Set alarm 1
          myRTC.read(tm);                                   // Read date and time
          //myRTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 0);   // Set alarm to minute rollover
          //myRTC.setAlarm(ALM1_MATCH_MINUTES, 0, (tm.Minute + alarmInterval) % 60, 0, 0);  // Calculate next alarm
          myRTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0);   // Set alarm to hour rollover
          //myRTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, 0, 0);     // Set alarm to day rollover
          myRTC.alarm(ALARM_1);                             // Ensure alarm 1 interrupt flag is cleared
          myRTC.alarmInterrupt(ALARM_1, true);              // Enable interrupt output for alarm 1

          // Flush RX buffer to clear any old data
          while (Serial1.available()) {
            Serial1.read();
          }

          // Once Serial1 is idle with an empy buffer, start TC3 interrupts to copy all new data into SerialBuffer
          // Set timer interval to: 10 * 10 / 230400 = 0.000434 seconds (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 230400 baud)
          startTimerInterval(0.000434);

          // Read battery voltage
          voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);
          Serial.print("voltage: "); Serial.println(voltage, 2);

          // Prevent logging if voltage < lowBattery
          if (voltage < lowBattery) {
            Serial.println("Warning: Low battery!");
            //break;
          }

          loopStep = START_UBX;
        } // End if (valFix == maxValFix)
      } // End case START:
      break;

    case START_UBX: {
#if DEBUG
        Serial.println("case START_UBX:");
#endif
        //enableNmea();
        enableUbx();          // Enable UBX RAWX and SFRBX messages
        bufferPointer = 0;    // Initialize bufferPointer

        loopStep = OPEN_FILE; // Open log file
      } // End case START_UBX:
      break;

    case OPEN_FILE: {

#if DEBUG
        Serial.println("Case: OPEN_FILE");
#endif
        // Read current date and time
        myRTC.read(tm);

        // Create a new folder
        snprintf(dirName, sizeof(dirName), "%04u%02d%02d", (tm.Year + 1970), tm.Month, tm.Day);
        if (sd.mkdir(dirName)) {
          Serial.print("Created folder: ");
          Serial.println(dirName);
        }
        else {
          Serial.println("Warning: Unable to create new folder!");
        }

        // Create log file
        snprintf(fileName, sizeof(fileName), "%04u%02d%02d/%02d%02d%02d.ubx",
                 (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);
        if (file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
          Serial.print("Logging to: ");
          Serial.println(fileName);
        }
        else {
          Serial.println("Warning: Unable to open new log file. Halting!");
          while (1); // Halt program
        }

        // Set the log file creation time
        if (!file.timestamp(T_CREATE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
          Serial.println("Warning: Unable to set file create timestamp!");
        }

        bytesWritten = 0;                   // Clear bytesWritten
        parseStep = PARSE_UBX_SYNC_CHAR_1;  // Set parseStep to expect B5 or $
        ubxLength = 0;                      // Set ubxLength to zero

        loopStep = WRITE_FILE; // Start logging data

      } // End case OPEN_FILE
      break;

    case WRITE_FILE: {

        // Move bytes into serBuffer and write to SD card once sdPacket threshold is reached
        int bufferAvailable = SerialBuffer.available();

        if (bufferAvailable > 0) {
#if DEBUG_SERIAL_BUFFER
          if (bufferAvailable > maxBufferAvailable) {
            maxBufferAvailable = bufferAvailable;
            Serial.print("Max bufferAvailable: ");
            Serial.println(maxBufferAvailable);
          }
#endif
          // Read bytes on UART1
          uint8_t c = SerialBuffer.read_char();

          // Write bytes to serBuffer
          serBuffer[bufferPointer] = c;

#if DEBUG_UBX
          // Echo UBX hex data to Serial Monitor
          sprintf(dataString, "%02X ", c);
          Serial.print(dataString);
#endif

#if DEBUG_NMEA
          // Echo NMEA data to Serial Monitor
          Serial.print(char(c));
#endif

          bufferPointer++; // Increment counter
          // If serBuffer equals size of sdPacket (512 bytes), write data to SD card
          if (bufferPointer == sdPacket) {
            digitalWrite(LED_PIN, HIGH);
            bufferPointer = 0;
            numBytes = file.write(&serBuffer, sdPacket);
            file.sync(); // Sync the file system
            bytesWritten += sdPacket;

#if DEBUG
            if (numBytes != sdPacket) {
              Serial.print("Warning: SD write error! Write size was: ");
              Serial.print(sdPacket);
              Serial.print(", but ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
#endif
            digitalWrite(LED_PIN, LOW);
          }

          // UBX Frame Structure:
          // Sync Char 1: 1-byte  0xB5
          // Sync Char 2: 1-byte  0x62
          // Class:       1-byte  Group of related messages
          // ID byte:     1-byte  Defines message to follow
          // Length:      2-byte  Payload only length. Little-Endian unsigned 16-bit integer
          // Payload:     Variable number of bytes
          // CK_A:        1-byte  16-bit checksum
          // CK_B:        1-byte
          // Example:     B5 62 02 15 0010 4E621058395C5C40000012000101C6BC 06 00

          // NMEA Message Format:
          // Start character:   Always '$'
          // Address field:     Talker field <XX> and Sentence Formatter <XXX>
          // Data field(s):     Delimited by ',' and can vary in length
          // Checksum field:    Starts with '*' and is two characters
          // End sequence:      Always <CR><LF>
          // Example:           $ GPZDA,141644.00,22,03,2002,00,00 *67 <CR><LF>

          // Process data bytes according to parseStep switch statement
          // Only allow a new file to be opened when a complete packet has been processed and
          // parseStep has returned to "PARSE_UBX_SYNC_CHAR_1" or when a data error is detected (SYNC_LOST)
          switch (parseStep) {

            case PARSE_UBX_SYNC_CHAR_1: {
                // Check for UBX Sync Char 1 (0xB5)
                if (c == ubxSyncChar1) {
                  parseStep = PARSE_UBX_SYNC_CHAR_2; // Look for Sync Char 2 (0x62)
                }
                // Check for NMEA Start Character '$'
                else if (c == '$') {
                  parseStep = PARSE_NMEA_START_CHAR; // Continue until an asterix is received
                  ubxLength = 0;      // Reset ubxLength then use it to track which character has arrived
                  nmeaChecksum = 0;   // Reset nmeaChecksum and update it as each character arrives
                  nmeaAddress1 = '0'; // Reset first five NMEA chars to invalid values
                  nmeaAddress2 = '0';
                  nmeaAddress3 = '0';
                  nmeaAddress4 = '0';
                  nmeaAddress5 = '0';
                }
                else {
                  Serial.println("Warning: Expected UBX Sync Char 1 (0xB5) or NMEA Start Character '$' but did not receive one!");
                  parseStep = SYNC_LOST;
                }
              } // End case PARSE_UBX_SYNC_CHAR_1:
              break;

            case PARSE_UBX_SYNC_CHAR_2: {
                // Check for UBX Sync Char 2 (0x62)
                if (c == ubxSyncChar2) {
                  ubxExpectedChecksumA = 0; // Reset the expected checksum
                  ubxExpectedChecksumB = 0;
                  parseStep = PARSE_UBX_CLASS; // Look for Class byte
                }
                else {
                  Serial.println("Warning: Expected Sync Char 0x62 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
              } // End case PARSE_UBX_SYNC_CHAR_2:
              break;

            // Class and ID information
            // RXM_RAWX:    Class 0x02  ID 0x15
            // RXM_SFRBF:   Class 0x02  ID 0x13
            case PARSE_UBX_CLASS: {
                ubxClass = c;
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_ID; // Look for ID byte

                // Class syntax checking
                if (ubxClass != 0x02) {
                  Serial.println("Warning: Expected Class 0x02 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
              } // End case PARSE_UBX_CLASS:
              break;

            case PARSE_UBX_ID: {
                ubxId = c;
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_LENGTH_LSB; // Look for UBX length LSB

                // UBX Class and ID syntax checking
                if ((ubxClass == 0x02) && ((ubxId != 0x15) && (ubxId != 0x13))) {
                  Serial.println("Warning: Expected ID of 0x15 or 0x13 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
              } // End case PARSE_UBX_ID:
              break;

            case PARSE_UBX_LENGTH_LSB: {
                ubxLength = c; // Store the length LSB
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_LENGTH_MSB; // Look for length MSB
              } // End case PARSE_UBX_LENGTH_LSB:
              break;

            case PARSE_UBX_LENGTH_MSB: {
                ubxLength = ubxLength + (c * 256); // Add length MSB
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_PAYLOAD; // Look for payload bytes (length: ubxLength)
              } // End case PARSE_UBX_LENGTH_MSB:
              break;

            case PARSE_UBX_PAYLOAD: {
                ubxLength = ubxLength - 1; // Decrement length by 1
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                if (ubxLength == 0) {
                  ubxExpectedChecksumA = ubxExpectedChecksumA & 0xff; // Limit checksums to 8-bits
                  ubxExpectedChecksumB = ubxExpectedChecksumB & 0xff;
                  parseStep = PARSE_UBX_CHECKSUM_A; // If we have received length payload bytes, look for checksum bytes
                }
              } // End case PARSE_UBX_PAYLOAD:
              break;

            case PARSE_UBX_CHECKSUM_A: {
                ubxChecksumA = c;
                parseStep = PARSE_UBX_CHECKSUM_B;
              } // End case PARSE_UBX_SYNC_CHAR_1:
              break;

            case PARSE_UBX_CHECKSUM_B: {
#if DEBUG_UBX
                Serial.println(); // Serial Monitor line break between UBX messages
#endif
                ubxChecksumB = c;
                parseStep = PARSE_UBX_SYNC_CHAR_1; // All bytes received. Look for new Sync Char 1 unless there is a checksum error
                if ((ubxExpectedChecksumA != ubxChecksumA) || (ubxExpectedChecksumB != ubxChecksumB)) {
                  Serial.println("Warning: UBX checksum error!");
                  parseStep = SYNC_LOST;
                }
              } // End case PARSE_UBX_CHECKSUM_B:
              break;

            // Parse NMEA messages
            case PARSE_NMEA_START_CHAR: {
                ubxLength++; // Increase message length count
                // If length is greater than nmeaMaxLength, likely SYNC_LOST
                if (ubxLength > nmeaMaxLength) {
                  Serial.println("Warning: Excessive NMEA message length!");
                  parseStep = SYNC_LOST;
                  break;
                }
                // If this is one of the first five characters, store it
                // May be useful for on-the-fly message parsing or DEBUG
                if (ubxLength <= 5) {
                  if (ubxLength == 1) {
                    nmeaAddress1 = c;
                  }
                  else if (ubxLength == 2) {
                    nmeaAddress2 = c;
                  }
                  else if (ubxLength == 3) {
                    nmeaAddress3 = c;
                  }
                  else if (ubxLength == 4) {
                    nmeaAddress4 = c;
                  }
                  else { // ubxLength == 5
                    nmeaAddress5 = c;

#if DEBUG
                    Serial.print("NMEA message type is: ");
                    Serial.print(char(nmeaAddress1));
                    Serial.print(char(nmeaAddress2));
                    Serial.print(char(nmeaAddress3));
                    Serial.print(char(nmeaAddress4));
                    Serial.println(char(nmeaAddress5));
#endif
                  }
                }
                // Check for asterisk '*'
                if (c == '*') {
                  // If asterix received do not exOR it into the checksum
                  // Instead calculate what the expected checksum should be (nmeaChecksum in ASCII hex)
                  nmeaExpectedChecksum1 = ((nmeaChecksum & 0xf0) >> 4) + '0'; // Convert MS nibble to ASCII hex
                  if (nmeaExpectedChecksum1 >= ':') {
                    nmeaExpectedChecksum1 += 7;  // Follows 9 so add 7 to convert to A-F
                  }
                  nmeaExpectedChecksum2 = (nmeaChecksum & 0x0f) + '0'; // Convert LS nibble to ASCII hex
                  if (nmeaExpectedChecksum2 >= ':') {
                    nmeaExpectedChecksum2 += 7;  // Follows 9 so add 7 to convert to A-F
                  }
                  parseStep = PARSE_NMEA_CHECKSUM_1; // Look for first checksum character
                  break; // Do not include '*' in checksum
                }
                // Update the checksum
                // The checksum is the exclusive-OR of all characters between the $ and the *
                nmeaChecksum = nmeaChecksum ^ c;
              } // End case PARSE_NMEA_START_CHAR:
              break;

            case PARSE_NMEA_CHECKSUM_1: {
                // Store first NMEA checksum character
                nmeaChecksum1 = c;
                parseStep = PARSE_NMEA_CHECKSUM_2;
              } // End case PARSE_NMEA_CHECKSUM_1:
              break;

            case PARSE_NMEA_CHECKSUM_2: {
                // Store second NMEA checksum character
                nmeaChecksum2 = c;
                // Check if checksum is correct
                if ((nmeaChecksum1 != nmeaExpectedChecksum1) || (nmeaChecksum2 != nmeaExpectedChecksum2)) {
                  // If checksum does not match, SYNC_LOST
                  Serial.println("Warning: NMEA checksum error!");
                  parseStep = SYNC_LOST;
                }
                else {
                  // Checksum was valid so wait for the terminators
                  parseStep = PARSE_NMEA_END_1;
                }
              } // End case PARSE_NMEA_CHECKSUM_2:
              break;

            case PARSE_NMEA_END_1: {
                // Check for <CR>
                if (c != '\r') {
                  Serial.println("Warning: NMEA CR not found!");
                  parseStep = SYNC_LOST;
                }
                else {
                  parseStep = PARSE_NMEA_END_2;
                }
              } // End case PARSE_NMEA_END_1:
              break;

            case PARSE_NMEA_END_2: {
                // Check for <LF>
                if (c != '\n') {
                  Serial.println("Warning: NMEA LF not found!");
                  parseStep = SYNC_LOST;
                }
                else {
                  // <LF> was received. Return to looking for '0xB5' or '$'
                  parseStep = PARSE_UBX_SYNC_CHAR_1;
                }
              } // End case PARSE_NMEA_END_2:
              break;
          }
        }
        else {
          // Read battery voltage
          voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);
        }

        // Check for conditions that would halt logging
        if (alarmFlag == true) {
          if (myRTC.alarm(ALARM_1)) {
            alarmFlag = true;
          }
          alarmFlag = false;
        }

        // Check if stop button was pressed
        if (digitalRead(SW_PIN) == LOW) {
          loopStep = CLOSE_FILE; // Close the file
          break;
        }
        // Check for low voltage
        else if (voltage < lowBattery) {
          loopStep = CLOSE_FILE; // Close the file
          break;
        }
        // Check if RTC alarm was triggered
        else if ((alarmFlag == true) && (parseStep == PARSE_UBX_SYNC_CHAR_1)) {
          Serial.println("ALARM_1 Triggered");
          loopStep = NEW_FILE; // Close current log file and create new file
          break;
        }
        // Check if sync was lost
        else if (parseStep == SYNC_LOST) {
          Serial.println("Sync lost");
          loopStep = CLOSE_FILE; // If sync is lost, disable RAWX messages, close and open a new log file
        }
      } // End case WRITE_FILE:
      break;

    // Close current log file and open a new one without stopping RAWX messages
    case NEW_FILE: {

#if DEBUG
        Serial.println("Case: NEW_FILE");
#endif
        // Write remaining data in serBuffer to log file
        if (bufferPointer > 0) {
          numBytes = file.write(&serBuffer, bufferPointer); // Write data
          file.sync(); // Sync the file system
          bytesWritten += bufferPointer;
          bufferPointer = 0; // Reset bufferPointer
#if DEBUG
          if (numBytes != sdPacket) {
            Serial.print("Warning: ");
            Serial.print(numBytes);
            Serial.print(" bytes of ");
            Serial.print(sdPacket);
            Serial.println(" byte packet were written.");
          }
#endif
        }

        // Read current date and time
        myRTC.read(tm);

        // Set the log file's last write/modification date and time
        if (!file.timestamp(T_WRITE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
          Serial.println("Warning: Could not set file write timestamp!");
        }
        // Set log file's last access date and time
        if (!file.timestamp(T_ACCESS, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
          Serial.println("Warning: Could not set file access timestamp!");
        }

        // Close the log file
        file.close();
        Serial.println("File closed!");

#if DEBUG
        Serial.print("File size is: ");
        Serial.println(file.fileSize());
        Serial.print("File size should be: ");
        Serial.println(bytesWritten);
#endif

        // Set RTC alarm
        alarmFlag = false; // Clear alarm flag
        myRTC.read(tm);
        printDateTime(myRTC.get()); Serial.println();

        myRTC.setAlarm(ALM1_MATCH_MINUTES, 0, (tm.Minute + alarmInterval) % 60, 0, 0 ); // Calculate next alarm
        //myRTC.setAlarm(ALM1_MATCH_HOURS, 0, 0, (tm.Hour + alarmInterval) % 24, 0 ); // Calculate next alarm
        myRTC.alarm(ALARM_1); // Ensure alarm 1 interrupt flag is cleared

        loopStep = OPEN_FILE; // Loop to open a new file

      } // End case NEW_FILE:
      break;

    case CLOSE_FILE: {

#if DEBUG
        Serial.println("Case: CLOSE_FILE");
#endif
        // Disable UBX RAWX messages, save any residual data and close the log file
        disableUbx();
        //disableNmea();

        // Wait for any residual data
        uint16_t dwellCounter = 0;
        while (dwellCounter < dwell) {
          while (SerialBuffer.available()) {
            serBuffer[bufferPointer] = SerialBuffer.read_char(); // Place extra bytes in serBuffer
            bufferPointer++;
            // Write a full packet
            if (bufferPointer == sdPacket) {
              bufferPointer = 0;
              numBytes = file.write(&serBuffer, sdPacket);
              file.sync(); // Sync the file system
              bytesWritten += sdPacket;
#if DEBUG
              if (numBytes != sdPacket) {
                Serial.print("Warning: SD write error! Write size was: ");
                Serial.print(sdPacket);
                Serial.print(", but ");
                Serial.print(numBytes);
                Serial.println(" were written.");
              }
#endif
            }
          }
          dwellCounter++;
          delay(1);
        }

        // Write remaining data in serBuffer to log file
        if (bufferPointer > 0) {
          numBytes = file.write(&serBuffer, bufferPointer); // Write data
          file.sync(); // Sync the file system
          bytesWritten += bufferPointer;

#if DEBUG
          if (numBytes != sdPacket) {
            Serial.print("Warning: ");
            Serial.print(numBytes);
            Serial.print(" bytes of ");
            Serial.print(sdPacket);
            Serial.println(" byte packet were written.");
          }
          Serial.print("Final SD write: ");
          Serial.print(bufferPointer);
          Serial.println(" bytes");
          Serial.print(bytesWritten);
          Serial.println(" bytes written.");
#endif
          bufferPointer = 0; // Reset bufferPointer
        }

        // Read current date and time
        myRTC.read(tm);

        // Set the log file's last write/modification date and time
        if (!file.timestamp(T_WRITE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
          Serial.println("Warning: Could not set file write timestamp!");
        }
        // Set log file's last access date and time
        if (!file.timestamp(T_ACCESS, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
          Serial.println("Warning: Could not set file access timestamp!");
        }

        // Close the log file
        file.close();
        Serial.println("File closed!");

#if DEBUG
        Serial.print("File size is: ");
        Serial.println(file.fileSize());
        Serial.print("File size should be: ");
        Serial.println(bytesWritten);
#endif

        // Halt program if stop button was pressed
        if (digitalRead(SW_PIN) == LOW) {
          Serial.println("Waiting for reset...");
          while (1); // Halt the program
        }
        // If low voltage, go to sleep
        else if (voltage < lowBattery) {
          loopStep = SLEEP;
          break;
        }
        else if (parseStep == SYNC_LOST) {
          loopStep = START_UBX;
          break;
        }
      } // End case CLOSE_FILE:
      break;

    case SLEEP: {

#if DEBUG
        Serial.println("Case: SLEEP");
#endif
        //LowPower.deepSleep();

        loopStep = WAKE;
      } // End case SLEEP:
      break;

    case WAKE: {

#if DEBUG
        Serial.println("Case: WAKE");
#endif

        loopStep = START;
      } // End case WAKE:
      break;

  } // End switch (loopStep)

} // End loop()

// RTC alarm interrupt service routine (ISR).
void alarmIsr() {
  alarmFlag = true; // Set alarm flag
}

// Print current time and date.
void printDateTime(time_t t) {
  char dateBuffer[25];
  snprintf(dateBuffer, sizeof(dateBuffer), "%04u-%02d-%02d %02d:%02d:%02d",
           year(t), month(t), day(t), hour(t), minute(t), second(t));
  Serial.print(dateBuffer);
}

// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer.
// Defines SerialBuffer as a large RingBuffer that will store Serial1 received
// data using a timer interrupt. Avoids having to increase the size of the
// Serial1 receive buffer by editing RingBuffer.h
// Use DEBUG_SERIAL_BUFFER to determine size of buffer.
// Increase if bufferAvailable nears or reaches the buffer size.
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 16

// Set TC3 Interval (sec)
void setTimerInterval(float intervalS) {
  int compareValue = intervalS * CPU_HZ / TIMER_PRESCALER_DIV;
  if (compareValue > 65535) compareValue = 65535;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

// Start TC3 with a specified interval
void startTimerInterval(float intervalS) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // Wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // Wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // Wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // Wait for sync

  // Set prescaler to 16
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // Wait for sync

  setTimerInterval(intervalS);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_SetPriority(TC3_IRQn, 3); // Set the TC3 interrupt priority to 3 (lowest)
  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // Wait for sync
}

// TC3 Interrupt Handler
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // copy any available Serial1 data into SerialBuffer
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    int available1 = Serial1.available(); // Check if there is any data waiting in the Serial1 RX buffer
    while (available1 > 0) {
      SerialBuffer.store_char(Serial1.read()); // If there is, copy it into our RingBuffer
      available1--;
    }
  }
}
