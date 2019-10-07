/*
    Title:    ZED-F9P Raw Datalogger
    Date:     October 7, 2019
    Author:   Adam Garbo      
    
    Components:
    - Adafruit Feather M0 Adalogger
    - SparkFun GPS-RTK2 Board ZED-F9P
    - Adafruit Breadboard-friendly RGB Smart NeoPixel
    
    Comments:
      This code is based heaivly on Paul Clark's F9P_RAWX_Logger, available from: 
      https://github.com/PaulZC/F9P_RAWX_Logger
*/

// Libraries
#include <Adafruit_NeoPixel.h>              // https://github.com/adafruit/Adafruit_NeoPixel
#include <RTCZero.h>                        // https://github.com/arduino-libraries/RTCZero
#include <SdFat.h>                          // https://github.com/greiman/SdFat
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <SPI.h>                            // https://www.arduino.cc/en/Reference/SPI
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire

// Define constants
#define DEBUG                 // Comment to disable debug messages
//#define DEBUG_I2C             // Comment to disable I2C debug messages
#define DEBUG_SERIAL_BUFFER   // Comment to disable Serial buffer messages
#define NEO_PIXEL             // Uncomment this line to enable the NeoPixel(s)
#define LED_PIN         8     // Which pin on the Arduino is connected to the NeoPixels?
#define LED_COUNT       1     // How many NeoPixels are attached to the Arduino?
#define SW_PIN          A0    // Switch to halt logging and close the log file.

// Switch statement
#define INIT            0
#define START_RAWX      1
#define OPEN_FILE       2
#define WRITE_FILE      3
#define NEW_FILE        4
#define CLOSE_FILE      5
#define RESTART_FILE    6
uint8_t loopStep = INIT;

// Instantiate objects
Adafruit_NeoPixel   pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
RingBufferN<16384>  SerialBuffer; // Define SerialBuffer as a RingBuffer of size 16k bytes
RTCZero             rtc;
SdFat               sd;
SdFile              file;
SFE_UBLOX_GPS       myGPS;

// User-declared global variables and constants
const uint16_t  interval    = 6;   // Duration of log (in hours) before new log file is created
const float     lowBattery  = 3.5;

// For a measurement rate of 4Hz (250msec), 300msec is a sensible value. i.e. slightly more than one measurement interval
const uint16_t  dwell       = 300;  // Time to wait in msec for residual RAWX data before closing the log file

// Declare global variables and constants
volatile bool alarmFlag       = false;                  // RTC alarm interrupt service routine (ISR) flag
bool          stopPressed     = false;                  // Flag to indicate if stop switch was pressed to halt logging
char          dirName[9]      = "YYYYMMDD";             // Log file directory name buffer
char          fileName[22]    = "YYYYMMDD/HHMMSS.ubx";  // Log file name buffer
float         voltage         = 0.0;                    // Battery voltage
const size_t  sdPacket        = 512;                    // Packet size
const uint8_t chipSelect      = 5;                      // SD card chip select
uint8_t       serBuffer[sdPacket];                      // Buffer
uint8_t       valFix          = 0;                      // GNSS valid fix counter
uint8_t       maxValFix       = 10;                     // Maximum number of valid GNSS fixes to be collected
int16_t       maxSerialBufferAvailable = 0;             // SerialBuffer debugging
uint16_t      numBytes        = 0;                      //
uint32_t      bytesWritten    = 0;                      //
size_t        bufferPointer   = 0;                      // Buffer pointer

#ifdef NEO_PIXEL
// NeoPixel colors
const uint32_t  red         = pixels.Color(255, 0, 0);
const uint32_t  orange      = pixels.Color(255, 127, 0);
const uint32_t  yellow      = pixels.Color(255, 255, 0);
const uint32_t  green       = pixels.Color(0, 255, 0);
const uint32_t  cyan        = pixels.Color(0, 255, 255);
const uint32_t  blue        = pixels.Color(0, 0, 255);
const uint32_t  violet      = pixels.Color(127, 0, 255);
const uint32_t  magenta     = pixels.Color(255, 0, 255);
const uint32_t  white       = pixels.Color(255, 255, 255);
const uint32_t  black       = pixels.Color(0, 0, 0);
uint8_t         brightness  = 8; // NeoPixel brightness (0 - 255 for WB2812B)
uint32_t        getColour   = pixels.getPixelColor(0);

// Set the NeoPixel(s) colour
void setLedColour(uint32_t colour) {
  pixels.setPixelColor(0, colour);
  pixels.show();
}
#endif

// UBX and NMEA Parse State
#define looking_for_B5_dollar   0
#define looking_for_62          1
#define looking_for_class       2
#define looking_for_ID          3
#define looking_for_length_LSB  4
#define looking_for_length_MSB  5
#define processing_payload      6
#define looking_for_checksum_A  7
#define looking_for_checksum_B  8
#define sync_lost               9
#define looking_for_asterix     10
#define looking_for_csum1       11
#define looking_for_csum2       12
#define looking_for_term1       13
#define looking_for_term2       14
#define max_nmea_len            100 // Maximum NMEA message length. Used to detect if sync is lost while receiving an NMEA message.

//
int ubx_nmea_state          = looking_for_B5_dollar;
int ubx_length              = 0;
int ubx_class               = 0;
int ubx_ID                  = 0;
int ubx_checksum_A          = 0;
int ubx_checksum_B          = 0;
int ubx_expected_checksum_A = 0;
int ubx_expected_checksum_B = 0;
int nmea_char_1             = '0'; // e.g. G
int nmea_char_2             = '0'; // e.g. P
int nmea_char_3             = '0'; // e.g. G
int nmea_char_4             = '0'; // e.g. G
int nmea_char_5             = '0'; // e.g. A
int nmea_csum               = 0;
int nmea_csum1              = '0';
int nmea_csum2              = '0';
int nmea_expected_csum1     = '0';
int nmea_expected_csum2     = '0';

// Enable all messages logged to SD card
uint8_t enableRawx() {
  myGPS.newCfgValset8(0x209102a5, 0x01, VAL_LAYER_RAM);  // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  myGPS.addCfgValset8(0x20910232, 0x01);                 // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
  myGPS.addCfgValset8(0x10930006, 0x00);                 // CFG-NMEA-HIGHPREC
  return myGPS.sendCfgValset8(0x209100bb, 0x00);         // CFG-MSGOUT-NMEA_ID_GGA_UART1
}

// Disable all messages logged to SD card
uint8_t disableRawx() {
  myGPS.newCfgValset8(0x209102a5, 0x00, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  myGPS.addCfgValset8(0x20910232, 0x00);                // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
  myGPS.addCfgValset8(0x10930006, 0x00);                // CFG-NMEA-HIGHPREC
  return myGPS.sendCfgValset8(0x209100bb, 0x00);        // CFG-MSGOUT-NMEA_ID_GGA_UART1
}

// Enable NMEA messages on UART1
uint8_t enableNmea() {
  myGPS.newCfgValset8(0x209100ca, 0x00, VAL_LAYER_RAM);  // CFG-MSGOUT-NMEA_ID_GLL_UART1
  myGPS.addCfgValset8(0x209100c0, 0x00);                 // CFG-MSGOUT-NMEA_ID_GSA_UART1
  myGPS.addCfgValset8(0x209100c5, 0x00);                 // CFG-MSGOUT-NMEA_ID_GSV_UART1
  myGPS.addCfgValset8(0x209100b1, 0x00);                 // CFG-MSGOUT-NMEA_ID_VTG_UART1
  myGPS.addCfgValset8(0x20920007, 0x00);                 // CFG-INFMSG-NMEA_UART1
  myGPS.addCfgValset8(0x209100bb, 0x01);                 // CFG-MSGOUT-NMEA_ID_GGA_UART1
  return myGPS.sendCfgValset8(0x209100ac, 0x01);         // CFG-MSGOUT-NMEA_ID_RMC_UART1
}

// Disable NMEA messages on UART1
uint8_t disableNmea() {
  myGPS.newCfgValset8(0x209100ca, 0x00, VAL_LAYER_RAM);  // CFG-MSGOUT-NMEA_ID_GLL_UART1
  myGPS.addCfgValset8(0x209100c0, 0x00);                 // CFG-MSGOUT-NMEA_ID_GSA_UART1
  myGPS.addCfgValset8(0x209100c5, 0x00);                 // CFG-MSGOUT-NMEA_ID_GSV_UART1
  myGPS.addCfgValset8(0x209100b1, 0x00);                 // CFG-MSGOUT-NMEA_ID_VTG_UART1
  myGPS.addCfgValset8(0x20920007, 0x00);                 // CFG-INFMSG-NMEA_UART1
  myGPS.addCfgValset8(0x209100bb, 0x00);                 // CFG-MSGOUT-NMEA_ID_GGA_UART1
  return myGPS.sendCfgValset8(0x209100ac, 0x00);         // CFG-MSGOUT-NMEA_ID_RMC_UART1
}

// Set time between GNSS measurements


void setup() {

#ifdef NEO_PIXEL
  pixels.begin();                   // Initialise the NeoPixel(s)
  pixels.setBrightness(brightness); // Set the NeoPixel(s) brightness
  pixels.clear();                   // Set the NeoPixel(s) to 'off'
  setLedColour(orange);             // Set the NeoPixel(s) to orange
#endif

  // Initialize SW_PIN (A1) as an input for the stop switch
  pinMode(SW_PIN, INPUT_PULLUP);

  //while (!Serial);  // Wait for user to open Serial Monitor
  delay(10000);     // Allow 10 seconds for user to open Serial Monitor

  Serial.begin(115200);

  Serial.println("u-blox ZED-F9P RAWX Logger");
  Serial.println("--------------------------");

#ifdef NEO_PIXEL
  Serial.println("NeoPixel colours:");
  Serial.println("Red.............Program halted due to error");
  Serial.println("Orange..........Awaiting Serial Monitor");
  Serial.println("Yellow..........Acquiring GNSS fix");
  Serial.println("Green...........Logging data");
  Serial.println("Cyan............Collecting GNSS fixes");
  Serial.println("Blue............GNSS fix acquired");
  Serial.println("Violet..........GNSS initialized");
  Serial.println("Magenta.........SD card initialized");
  Serial.println("White...........");
  Serial.println("--------------------------");
#else
  Serial.println("Green: ");
  Serial.println("Red: ");
#endif

  Wire.begin();
  Wire.setClock(400000); // Increase clock frequency for I2C communications to 400kHz

  // Connect to the u-blox ZED-F9P using the Wire (I2C) interface
  Serial.println("Initializing u-blox ZED-F9P...");
  if (myGPS.begin() == true) {
    Serial.println("u-blox ZED-F9P detected");
  }
  else {
    Serial.println("u-blox ZED-F9P not detected at default I2C address. Please check wiring. Halting.");
#ifdef NEO_PIXEL
    setLedColour(red); // Set NeoPixel(s) to red
#endif
    while (1);
  }

#ifdef DEBUG_I2C
  myGPS.enableDebugging(); // Enable I2C debug messages over Serial
#endif

  // Responses to UBX commands shown in debugging output.
  // Acknowledged:    CLS:5 ID:1 Payload: 6 8A
  // Unacknowledged:  CLS:5 ID:0
  bool setValueSuccess = true;
  setValueSuccess &= myGPS.setVal8(0x10720002, 0);        // Disable NMEA as an output protocol on I2C    (CFG-I2COUTPROT-NMEA)
  setValueSuccess &= myGPS.setVal32(0x40520001, 230400);  // Set UART1 baud rate to 230400                (CFG-UART1-BAUDRATE)
  setValueSuccess &= disableRawx();                       // Disable RAWX messages on UART1
  setValueSuccess &= disableNmea();                       // Disable NMEA messages on UART1
  setValueSuccess &= myGPS.setVal8(0x20110021, 2);        // Set the dynamic platform model to stationary (CFG-NAVSPG-DYNMODEL)

  if (setValueSuccess == true) {
    Serial.println("Initialized u-blox ZED-F9P.");
#ifdef NEO_PIXEL
    setLedColour(violet); // Set the NeoPixel(s) to magenta
    delay(1000);
#endif
  }
  else {
    Serial.println("Unable to initialize u-blox ZED-F9P. Halting.");
#ifdef NEO_PIXEL
    setLedColour(red); // Set the NeoPixel(s) to red
#endif
    while (1); // Halt program
  }

  Serial1.begin(230400); // Start Serial1 at 230400 baud

  // Initialize the SD card
  Serial.println("Initializing SD card...");
  if (sd.begin(chipSelect, SD_SCK_MHZ(50))) {
    Serial.println("SD card initialized.");
#ifdef NEO_PIXEL
    setLedColour(magenta); // Set the NeoPixel(s) to magenta
    delay(1000);
#endif
  }
  else {
    Serial.println("Unable to initialize SD card. Halting.");
#ifdef NEO_PIXEL
    setLedColour(red); // Set the NeoPixel(s) to red
#endif
    while (1); // Halt the program
  }

  // Initialize the RTC
  rtc.begin();

  Serial.println("Waiting for GNSS fix...");
}

void loop() {

  switch (loopStep) {
    //
    case INIT: {
        delay(1000);

#ifdef DEBUG
        char gpsDateTime[20]; // GNSS date time buffer
        snprintf(gpsDateTime, sizeof(gpsDateTime), "%04u-%02d-%02d %02d:%02d:%02d",
                 myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(),
                 myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());

        float latitude      = myGPS.getLatitude() / 10000000.0;
        float longitude     = myGPS.getLongitude() / 10000000.0;
        float pdop          = myGPS.getPDOP() / 100.0;
        uint8_t fix         = myGPS.getFixType();
        uint8_t satellites  = myGPS.getSIV();

        Serial.print(gpsDateTime);
        Serial.print(" Latitude: "); Serial.print(latitude, 6);
        Serial.print(" Longitude: "); Serial.print(longitude, 6);
        Serial.print(" Satellites: "); Serial.print(satellites);
        Serial.print(" Fix: "); Serial.print(fix);
        Serial.print(" PDOP: "); Serial.println(pdop, 2);
#endif
        // Read battery voltage
        voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);

        // Do we have a GNSS fix?
        if (myGPS.getFixType() > 0) {
#ifdef NEO_PIXEL
          setLedColour(cyan); // Set NeoPixel(s) to cyan to indicate GNSS fix
#endif
          // Increment valFix and cap at maxValFix. Do not decrement valFix as logging will continue even if fix is lost.
          valFix += 1;
          if (valFix > maxValFix) {
            valFix = maxValFix;
          }
        }
        else {
#ifdef NEO_PIXEL
          setLedColour(yellow); // Set NeoPixel(s) to orange to indicate lack of GNSS fix
#endif
        }
        // Have enough valid GNSS fixes been collected?
        if (valFix == maxValFix) {

#ifdef NEO_PIXEL
          setLedColour(blue); // Set NeoPixel(s) to blue
#endif
          alarmFlag = false;                                                                // Clear alarm flag
          rtc.setTime(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());               // Set the time
          rtc.setDate(myGPS.getDay(), myGPS.getMonth(), (uint8_t)(myGPS.getYear() - 2000)); // Set the date
          rtc.setAlarmSeconds(0);                                                           // Set RTC Alarm Seconds to zero
          uint8_t nextAlarmMin = ((myGPS.getMinute() + interval) / interval) * interval;    // Calculate next alarm minutes
          nextAlarmMin = nextAlarmMin % 60;                                                 // Correct hour rollover
          Serial.print("rtc.getHours: "); Serial.println(rtc.getHours());
          uint8_t nextAlarmHour = (rtc.getHours() + interval) % 24;
          Serial.print("nextAlarmHour: "); Serial.println(nextAlarmHour);
          //rtc.setAlarmMinutes(nextAlarmMin);                                                // Set RTC Alarm Minutes
          rtc.setAlarmHours(nextAlarmHour);
          rtc.enableAlarm(rtc.MATCH_HHMMSS);                                                // Alarm Match on minutes and seconds
          rtc.attachInterrupt(alarmMatch);                                                  // Attach alarm interrupt

          // Prevent logging if voltage < lowBattery
          if (voltage < lowBattery) {
            Serial.println("Warning: Low battery!");
            break;
          }

          while (Serial1.available()) {
            Serial1.read(); // Flush RX buffer to clear any old data
          }

          // Once Serial1 is idle with an empy buffer, start TC3 interrupts to copy all new data into SerialBuffer
          // Set timer interval to: 10 * 10 / 230400 = 0.000434 seconds (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 230400 baud)
          startTimerInterval(0.000434);

          loopStep = START_RAWX; // Start RAWX messages
        }
      }
      break;
      
    // Start RAWX messages
    case START_RAWX: {
#ifdef DEBUG
        Serial.println("Case: START_RAWX");
#endif
        enableRawx();         // Enable RAWX and SFRBX messages
        bufferPointer = 0;    // Initialize bufferPointer
        loopStep = OPEN_FILE; // Create and/or open log file
      }
      break;

    // Open the log file
    case OPEN_FILE: {
#ifdef DEBUG
        Serial.println("Case: OPEN_FILE");
#endif
        // Create a new folder
        snprintf(dirName, sizeof(dirName), "%04u%02d%02d",
                 (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay());
        if (!sd.mkdir(dirName)) {
          Serial.println("Warning: Unable to create new folder!");
        }
        else {
          Serial.print("Created folder: ");
          Serial.println(dirName);
        }

        // Create log file. File name limited to 8.3 characters. Use format: YYYYMMDD/HHMMSS.ubx
        snprintf(fileName, sizeof(fileName), "%04u%02d%02d/%02d%02d%02d.ubx",
                 (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(),
                 rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
        if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
          Serial.println("Warning: Unable to open new log file. Halting!");
#ifdef NEO_PIXEL
          setLedColour(red); // Set the NeoPixel(s) to red
#endif
          while (1); // Halt program
        }
        else {
          Serial.print("Logging to: ");
          Serial.println(fileName);
        }

        // Set the log file creation time
        if (!file.timestamp(T_CREATE, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file create timestamp!");
        }
        bytesWritten    = 0;                      // Clear bytesWritten
        ubx_nmea_state  = looking_for_B5_dollar;  // Set ubx_nmea_state to expect B5 or $
        ubx_length      = 0;                      // Set ubx_length to zero
#ifdef NEO_PIXEL
        setLedColour(green); // Set NeoPixel(s) to green
        delay(1000);
        setLedColour(black);
#endif
        loopStep = WRITE_FILE; // Start logging data
      }
      break;

    // Move bytes into serBuffer and write to SD card once sdPacket threshold is reached
    case WRITE_FILE: {

        int bufAvail = SerialBuffer.available();
        if (bufAvail > 0) {
#ifdef DEBUG_SERIAL_BUFFER
          if (bufAvail > maxSerialBufferAvailable) {
            maxSerialBufferAvailable = bufAvail;
            Serial.print("Max bufAvail: ");
            Serial.println(maxSerialBufferAvailable);
          }
#endif
          uint8_t c = SerialBuffer.read_char();
          serBuffer[bufferPointer] = c;
          bufferPointer++;
          if (bufferPointer == sdPacket) {
#ifdef NEO_PIXEL
            setLedColour(green); // Set NeoPixel(s) to green
#endif
            bufferPointer = 0;
            numBytes = file.write(&serBuffer, sdPacket);
            //file.sync(); // Sync the file system
            bytesWritten += sdPacket;

#ifdef DEBUG
            if (numBytes != sdPacket) {
              Serial.print("Warning: SD write error! Write size was: ");
              Serial.print(sdPacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
#endif
#ifdef NEO_PIXEL
            setLedColour(black); // Set NeoPixel(s) to green
#endif
          }
          // Process data bytes according to ubx_nmea_state
          // UBX Frame Structure:
          // Sync Char 1: 1-byte  0xB5
          // Sync Char 2: 1-byte  0x62
          // Class:       1-byte  Group of related messages
          // ID byte:     1-byte  Defines message to follow
          // Length:      2-byte  Payload only length. Little-Endian unsigned 16-bit integer
          // Payload:     Variable number of bytes
          // CK_A:        1-byte  16-bit checksum
          // CK_B:        1-byte
          //
          // NMEA message format:
          // Starts character '$'
          // Next five characters indicate the message type (stored in nmea_char_1 to nmea_char_5)
          // Message fields are comma-separated
          // Followed by an '*'
          // Then a two character checksum (the logical exclusive-OR of all characters between the $ and the * as ASCII hex)
          // Ends with <CR><LF>
          // Only allow a new file to be opened when a complete packet has been processed and ubx_nmea_state has returned to "looking_for_B5_dollar"
          // Or when a data error is detected (sync_lost)

          switch (ubx_nmea_state) {

            case (looking_for_B5_dollar): {
                // Have we found Sync Char 1 (0xB5) if we were expecting one?
                if (c == 0xB5) {
                  ubx_nmea_state = looking_for_62; // Now look for Sync Char 2 (0x62)
                }
                // Have we found an NMEA '$' if we were expecting one?
                else if (c == '$') {
                  ubx_nmea_state = looking_for_asterix; // Continue until an asterix is received
                  ubx_length = 0;     // Reset ubx_length then use it to track which character has arrived
                  nmea_csum = 0;      // Reset nmea_csum and update it as each character arrives
                  nmea_char_1 = '0';  // Reset first five NMEA chars to invalid values
                  nmea_char_2 = '0';
                  nmea_char_3 = '0';
                  nmea_char_4 = '0';
                  nmea_char_5 = '0';
                }
                else {
                  Serial.println("Warning: Expected Sync Char 1 (0xB5) or NMEA $ but did not receive one!");
                  ubx_nmea_state = sync_lost;
                }
              }
              break;
            case (looking_for_62): {
                // Have we found Sync Char 2 (0x62) when we were expecting one?
                if (c == 0x62) {
                  ubx_expected_checksum_A = 0;        // Reset the expected checksum
                  ubx_expected_checksum_B = 0;
                  ubx_nmea_state = looking_for_class; // Look for Class byte
                }
                else {
                  Serial.println("Warning: Expected Sync Char 0x62 but did not receive one!");
                  ubx_nmea_state = sync_lost;
                }
              }
              break;

            // Class and ID information
            // RXM_RAWX:    Class 0x02  ID 0x15
            // RXM_SFRBF:   Class 0x02  ID 0x13
            case (looking_for_class): {
                ubx_class = c;
                ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update expected checksum
                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                ubx_nmea_state = looking_for_ID; // Look for ID byte
#ifdef DEBUG
                // Class syntax checking
                if (ubx_class != 0x02) {
                  Serial.println("Warning: Expected Class 0x02 but did not receive one!");
                  ubx_nmea_state = sync_lost;
                }
#endif
              }
              break;
            case (looking_for_ID): {
                ubx_ID = c;
                ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update expected checksum
                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                ubx_nmea_state = looking_for_length_LSB; // Look for length LSB
#ifdef DEBUG
                // ID syntax checking
                if ((ubx_class == 0x02) && ((ubx_ID != 0x15) && (ubx_ID != 0x13))) {
                  Serial.println("Warning: Expected ID of 0x15 or 0x13 but did not receive one!");
                  ubx_nmea_state = sync_lost;
                }
#endif
              }
              break;
            case (looking_for_length_LSB): {
                ubx_length = c; // Store the length LSB
                ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update expected checksum
                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                ubx_nmea_state = looking_for_length_MSB; // Look for length MSB
              }
              break;
            case (looking_for_length_MSB): {
                ubx_length = ubx_length + (c * 256); // Add length MSB
                ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update expected checksum
                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                ubx_nmea_state = processing_payload; // Look for payload bytes (length: ubx_length)
              }
              break;
            case (processing_payload): {
                ubx_length = ubx_length - 1; // Decrement length by 1
                ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update expected checksum
                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
                if (ubx_length == 0) {
                  ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff; // Limit checksums to 8-bits
                  ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff;
                  ubx_nmea_state = looking_for_checksum_A; // If we have received length payload bytes, look for checksum bytes
                }
              }
              break;
            case (looking_for_checksum_A): {
                ubx_checksum_A = c;
                ubx_nmea_state = looking_for_checksum_B;
              }
              break;
            case (looking_for_checksum_B): {
                ubx_checksum_B = c;
                ubx_nmea_state = looking_for_B5_dollar; // All bytes received. Look for new Sync Char 1 unless there is a checksum error
                if ((ubx_expected_checksum_A != ubx_checksum_A) or (ubx_expected_checksum_B != ubx_checksum_B)) {
                  Serial.println("Warning: UBX checksum error!");
                  ubx_nmea_state = sync_lost;
                }
              }
              break;
            // NMEA messages
            case (looking_for_asterix): {
                ubx_length++; // Increase message length count
                // If length is greater than max_nmea_len, likely sync_lost
                if (ubx_length > max_nmea_len) {
                  Serial.println("Warning: Excessive NMEA message length!");
                  ubx_nmea_state = sync_lost;
                  break;
                }
                // If this is one of the first five characters, store it
                // May be useful for on-the-fly message parsing or DEBUG
                if (ubx_length <= 5) {
                  if (ubx_length == 1) {
                    nmea_char_1 = c;
                  }
                  else if (ubx_length == 2) {
                    nmea_char_2 = c;
                  }
                  else if (ubx_length == 3) {
                    nmea_char_3 = c;
                  }
                  else if (ubx_length == 4) {
                    nmea_char_4 = c;
                  }
                  else { // ubx_length == 5
                    nmea_char_5 = c;
#ifdef DEBUG
                    Serial.print("NMEA message type is: ");
                    Serial.print(char(nmea_char_1));
                    Serial.print(char(nmea_char_2));
                    Serial.print(char(nmea_char_3));
                    Serial.print(char(nmea_char_4));
                    Serial.println(char(nmea_char_5));
#endif
                  }
                }
                // Check if an asterisk '*'
                if (c == '*') {
                  // Asterix received
                  // Do not exOR it into the checksum
                  // Instead calculate what the expected checksum should be (nmea_csum in ASCII hex)
                  nmea_expected_csum1 = ((nmea_csum & 0xf0) >> 4) + '0'; // Convert MS nibble to ASCII hex
                  if (nmea_expected_csum1 >= ':') {
                    nmea_expected_csum1 += 7;  // : follows 9 so add 7 to convert to A-F
                  }
                  nmea_expected_csum2 = (nmea_csum & 0x0f) + '0'; // Convert LS nibble to ASCII hex
                  if (nmea_expected_csum2 >= ':') {
                    nmea_expected_csum2 += 7;  // : follows 9 so add 7 to convert to A-F
                  }
                  // Look for the first csum character
                  ubx_nmea_state = looking_for_csum1;
                  break; // Do not include '*' in checksum
                }
                // Update the checksum
                // The checksum is the exclusive-OR of all characters between the $ and the *
                nmea_csum = nmea_csum ^ c;
              }
              break;
            case (looking_for_csum1): {
                // Store first NMEA checksum character
                nmea_csum1 = c;
                ubx_nmea_state = looking_for_csum2;
              }
              break;
            case (looking_for_csum2): {
                // Store second NMEA checksum character
                nmea_csum2 = c;
                // Check if the checksum is correct
                if ((nmea_csum1 != nmea_expected_csum1) or (nmea_csum2 != nmea_expected_csum2)) {
                  // If checksum does not match, sync_lost
                  Serial.println("Warning: NMEA checksum error!");
                  ubx_nmea_state = sync_lost;
                }
                else {
                  // Checksum was valid so wait for the terminators
                  ubx_nmea_state = looking_for_term1;
                }
              }
              break;
            case (looking_for_term1): {
                // Check if this is CR
                if (c != '\r') {
                  Serial.println("Warning: NMEA CR not found!");
                  ubx_nmea_state = sync_lost;
                }
                else {
                  ubx_nmea_state = looking_for_term2;
                }
              }
              break;
            case (looking_for_term2): {
                // Check if this is LF
                if (c != '\n') {
                  Serial.println("Warning: NMEA LF not found!");
                  ubx_nmea_state = sync_lost;
                }
                else {
                  // LF was received so go back to looking for B5 or a $
                  ubx_nmea_state = looking_for_B5_dollar;
                }
              }
              break;
          }
        }
        else {
          // Read battery voltage
          voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);
        }
        // Check if stop button was pressed, if battery voltage is low or if RTC alarm was triggered
        if (digitalRead(SW_PIN) == LOW) {
          stopPressed = true;
        }
        if ((stopPressed == true) or (voltage < lowBattery)) {
          loopStep = CLOSE_FILE; // Now close the file
          break;
        }
        else if ((alarmFlag == true) and (ubx_nmea_state == looking_for_B5_dollar)) {
          loopStep = NEW_FILE; // Close current log file and create new file
          break;
        }
        else if (ubx_nmea_state == sync_lost) {
          loopStep = RESTART_FILE; // Sync lost. Halt RAWX messages, create new log file and restart RAWX messages
        }
      }
      break;

    // Close the current log file and open a new one without halting RAWX messages
    case NEW_FILE: {
        Serial.println("Case: NEW_FILE");
        // If data remains in serBuffer, write it to file
        if (bufferPointer > 0) {
          numBytes = file.write(&serBuffer, bufferPointer); // Write data
          file.sync(); // Sync the file system
          bytesWritten += bufferPointer;
#ifdef DEBUG
          if (numBytes != bufferPointer) {
            Serial.print("Warning: SD write error! Write size was: ");
            Serial.print(bufferPointer);
            Serial.print(". ");
            Serial.print(numBytes);
            Serial.println(" were written.");
          }
#endif
          bufferPointer = 0; // Reset bufferPointer
        }

        // Set the log file's last write/modification date and time
        if (!file.timestamp(T_WRITE, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Could not set file write timestamp!");
        }
        // Set log file's last access date and time
        if (!file.timestamp(T_ACCESS, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Could not set file access timestamp!");
        }

        file.close(); // Close the log file
        Serial.println("File closed!");

#ifdef DEBUG
        Serial.print("File size is: ");
        Serial.println(file.fileSize());
        Serial.print("File size should be: ");
        Serial.println(bytesWritten);
#endif
        // An RTC alarm was detected, so set the RTC alarm time to the next interval and loop back to OPEN_FILE.
        // We only receive an RTC alarm on a minute mark, so it doesn't matter that the RTC seconds will have moved on at this point.
        alarmFlag = false;                    // Clear the RTC alarm flag
        uint8_t rtc_mins = rtc.getMinutes();  // Read the RTC minutes
        rtc_mins = rtc_mins + interval;       // Add the interval to the RTC minutes
        rtc_mins = rtc_mins % 60;             // Correct for hour rollover
        //rtc.setAlarmMinutes(rtc_mins);        // Set next alarm time (minutes only - hours are ignored)
        bytesWritten = 0;                     // Clear bytesWritten

        Serial.print("rtc.getHours: "); Serial.println(rtc.getHours());
        uint8_t nextAlarmHour = (rtc.getHours() + interval) % 24;
        Serial.print("nextAlarmHour: "); Serial.println(nextAlarmHour);
        //rtc.setAlarmMinutes(nextAlarmMin);                                                // Set RTC Alarm Minutes
        rtc.setAlarmHours(nextAlarmHour);

        loopStep = OPEN_FILE; // Loop to open a new file
      }
      break;

    // Disable RAWX messages, save any residual data and close the log file.
    case CLOSE_FILE: {
        disableRawx(); // Disable RAWX messages
        int waitcount = 0;
        // Wait for residual data
        while (waitcount < dwell) {
          while (SerialBuffer.available()) {
            serBuffer[bufferPointer] = SerialBuffer.read_char(); // Place extra bytes in serBuffer
            bufferPointer++;
            if (bufferPointer == sdPacket) { // Write a full packet
              bufferPointer = 0;
              numBytes = file.write(&serBuffer, sdPacket);
              //file.sync(); // Sync the file system
              bytesWritten += sdPacket;
#ifdef DEBUG
              if (numBytes != sdPacket) {
                Serial.print("Warning: SD write error! Write size was: ");
                Serial.print(sdPacket);
                Serial.print(". ");
                Serial.print(numBytes);
                Serial.println(" bytes were written.");
              }
#endif
            }
          }
          waitcount++;
          delay(1);
        }
        // If there is any data left in serBuffer, write it to file
        if (bufferPointer > 0) {
          numBytes = file.write(&serBuffer, bufferPointer); // Write remaining data
          file.sync(); // Sync the file system
          bytesWritten += bufferPointer;

#ifdef DEBUG
          if (numBytes != bufferPointer) {
            Serial.print("Warning: SD write error! Write size was: ");
            Serial.print(bufferPointer);
            Serial.print(". ");
            Serial.print(numBytes);
            Serial.println(" bytes were written.");
          }
          Serial.print("Final SD write: ");
          Serial.print(bufferPointer);
          Serial.println(" bytes");
          Serial.print(bytesWritten);
          Serial.println(" bytes written.");
#endif
          bufferPointer = 0; // Reset bufferPointer
        }

        // Get the RTC time and date
        uint8_t RTCseconds = rtc.getSeconds();
        uint8_t RTCminutes = rtc.getMinutes();
        uint8_t RTChours = rtc.getHours();
        uint8_t RTCday = rtc.getDay();
        uint8_t RTCmonth = rtc.getMonth();
        uint8_t RTCyear = rtc.getYear();

        // Set log file write time
        if (!file.timestamp(T_WRITE, (RTCyear + 2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
          Serial.println("Warning! Could not set file write timestamp!");
        }

        // Set log file access time
        if (!file.timestamp(T_ACCESS, (RTCyear + 2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
          Serial.println("Warning! Could not set file access timestamp!");
        }

        file.close(); // close the file

#ifdef DEBUG
        Serial.print("File size is: ");
        Serial.println(file.fileSize());
        Serial.print("File size should be: ");
        Serial.println(bytesWritten);
#endif
        Serial.println("File closed!");
        // Either the battery is low or the user pressed the stop button:
        if (stopPressed == true) {
          // Stop switch was pressed so just wait for a reset
          Serial.println("Waiting for reset.");
#ifdef NEO_PIXEL
          setLedColour(red); // Set NeoPixel(s) to red
#endif
          while (1); // Wait for reset
        }
        else {
          // Low battery was detected so wait for the battery to recover
          Serial.println("Battery must be low - waiting for it to recover...");

          // Check the battery voltage. Make sure it has been OK for at least 5 seconds before continuing
          int high_for = 0;
          while (high_for < 500) {
            // read battery voltage
            voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);
            if (voltage < lowBattery) {
              high_for = 0; // If battery voltage is low, reset the count
            }
            else {
              high_for++; // Increase the count
            }
            delay(10); // Wait 10msec
          }
          // Loop round again and restart rawx messages before opening a new file
          loopStep = START_RAWX;
        }
      }
      break;

    // If RAWX data sync is lost, disable RAWX messages, save residual data, close log file, create a new file and restart RAWX messages. Do not update the next RTC alarm.
    case RESTART_FILE: {
        disableRawx(); // Disable RAWX messages
        int waitcount = 0;
        // Wait for residual data
        while (waitcount < dwell) {
          while (SerialBuffer.available()) {
            serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
            bufferPointer++;
            if (bufferPointer == sdPacket) { // Write a full packet
              bufferPointer = 0;
              numBytes = file.write(&serBuffer, sdPacket);
              //file.sync(); // Sync the file system

              bytesWritten += sdPacket;
#ifdef DEBUG
              if (numBytes != sdPacket) {
                Serial.print("Warning: SD write error! Write size was: ");
                Serial.print(sdPacket);
                Serial.print(". ");
                Serial.print(numBytes);
                Serial.println(" were written.");
              }
#endif
            }
          }
          waitcount++;
          delay(1);
        }
        // If there is any data left in serBuffer, write it to file
        if (bufferPointer > 0) {

          numBytes = file.write(&serBuffer, bufferPointer); // Write remaining data
          file.sync(); // Sync the file system
          bytesWritten += bufferPointer;

#ifdef DEBUG
          if (numBytes != bufferPointer) {
            Serial.print("SD write error! Write size was ");
            Serial.print(bufferPointer);
            Serial.print(". ");
            Serial.print(numBytes);
            Serial.println(" were written.");
          }
          Serial.print("Final SD Write: ");
          Serial.print(bufferPointer);
          Serial.println(" Bytes");
          Serial.print(bytesWritten);
          Serial.println(" Bytes written");
#endif
          bufferPointer = 0; // reset bufferPointer
        }

        // Get the RTC time and date
        uint8_t RTCseconds = rtc.getSeconds();
        uint8_t RTCminutes = rtc.getMinutes();
        uint8_t RTChours = rtc.getHours();
        uint8_t RTCday = rtc.getDay();
        uint8_t RTCmonth = rtc.getMonth();
        uint8_t RTCyear = rtc.getYear();

        // Set log file write time
        if (!file.timestamp(T_WRITE, (RTCyear + 2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
          Serial.println("Warning! Could not set file write timestamp!");
        }

        // Set log file access time
        if (!file.timestamp(T_ACCESS, (RTCyear + 2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
          Serial.println("Warning! Could not set file access timestamp!");
        }

        // Close the file
        file.close();

#ifdef DEBUG
        Serial.print("File size is: ");
        Serial.println(file.fileSize());
        Serial.print("File size should be: ");
        Serial.println(bytesWritten);
#endif
        Serial.println("File closed!");
        loopStep = START_RAWX; // loop round again and restart rawx messages before opening a new file
      }
      break;
  }
}

// RTC alarm interrupt service routine (ISR)
void alarmMatch() {
  alarmFlag = true; // Set alarm flag
}

// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer
// Defines SerialBuffer as a large RingBuffer that will store Serial1 received data using a timer interrupt
// Avoids having to increase the size of the Serial1 receive buffer by editing RingBuffer.h
// Use DEBUG_SERIAL_BUFFER to determine size of buffer. Increase if bufAvail nears or reaches the buffer size
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
