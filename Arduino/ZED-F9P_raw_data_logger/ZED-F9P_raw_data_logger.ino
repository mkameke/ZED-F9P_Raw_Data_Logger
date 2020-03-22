/*
  Title:    ZED-F9P Raw Data Logger
  Date:     March 22, 2020
  Author:   Adam Garbo

  Description:
  - Logs RXM-RAWX, RXM-SFRBX and TIM-TM2 data from u-blox ZED_F9P GNSS to SD card
  - Also logs NAV_PVT messages (which provide the carrSoln status) and NAV-STATUS messages (which indicate a time fix for Survey_In mode)
  - Also logs high precision NMEA GNGGA position solution messages which can be extracted by RTKLIB

  Components:
  - Adafruit Feather M0 Adalogger
  - Adafruit DS3231 Precision RTC FeatherWing
  - SparkFun GPS-RTK2 Board ZED-F9P

  Comments:
  This code is based heaivly on Paul Clark's F9P_RAWX_Logger:
  https://github.com/PaulZC/ZED-F9P_FeatherWing_USB
*/

// Libraries
#include <ArduinoLowPower.h>                // https://github.com/arduino-libraries/ArduinoLowPower
#include <RTCZero.h>                        // https://github.com/arduino-libraries/RTCZero
#include <SdFat.h>                          // https://github.com/greiman/SdFat
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <SPI.h>                            // https://www.arduino.cc/en/Reference/SPI
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire

// Debugging definitons
#define DEBUG               true // Output debug messages to Serial Monitor
#define DEBUG_I2C           false // Output I2C debug messages to Serial Monitor
#define DEBUG_NAV           false // Output NAV_PVT debug messages to Serial Monitor
#define DEBUG_SERIAL_BUFFER false // Outputs a message each time SerialBuffer.available reaches a new maximum
#define DEBUG_UBX           false // Output UBX debug messages to Serial Monitor

// LEDs
#define LED_ON  true  // Enable LEDs
#define LED_PIN 8     // Indicates that the GNSS has established a fix

// Pin assignments
#define GPS_INT_PIN A3  // Controls ON/OFF operation of ZED-F9P
#define MODE_PIN    6   // Connect MODE_PIN to GND to select base mode. Leave open for rover mode.
#define SURVEY_PIN  A3  // Connect to GND to select SURVEY_IN mode when in BASE mode
#define SW_PIN      10  // Connect a normally-open push-to-close switch between SW_PIN and GND to halt logging and close the log file

// Select alarm to set
#define ALARM_MIN false // Enable rolling-minutes alarm
#define ALARM_HR  false // Enable rolling-hours alarm
#define ALARM_DAY true  // Enable rolling-day alarm

// Object instantiations
RTCZero       rtc;
SdFat         sd;
SdFile        file;
SFE_UBLOX_GPS gnss;

// User-declared global variables and constants
const byte    alarmMinutes    = 10;     // Creates a new log file every alarmMinutes
const byte    alarmHours      = 4;      // Creates a new log file every alarmHours
const int     maxValFix       = 10;     // Minimum number of valid GNSS fixes to collect before beginning to log data
const int     dwell           = 1100;   // How long to wait in msec for residual UBX data before closing log file (e.g. 1Hz = 1000 ms, so 1100 ms is slightly more than one measurement interval)
const float   lowVoltage      = 3.5;    // Low battery voltage threshold

// Global flag variable delcarations
volatile bool alarmFlag       = false;  // RTC alarm interrupt service routine flag
volatile bool sleepFlag       = false;  // Flag to indicate to Watchdog Timer if in deep sleep mode
volatile byte watchdogCounter = 0;      // Watchdog interrupt service routine counter
bool          stopFlag        = false;  // Flag to indicate if stop switch was pressed to stop logging
bool          surveyFlag      = false;  // Flag to indicate if the code is in SURVEY_IN mode
bool          modeFlag        = true;   // Flag to indicate if the code is in base or rover mode. true = BASE mode, false = ROVER mode
bool          ledState        = LOW;    // Flag to toggle LED in blinkLed() function
bool          voltageFlag     = false;  // Flag to indiciate low battery voltage

// Global variables and constant declarations
const byte    chipSelect      = 4;      // SD card chip select
const size_t  sdPacket        = 512;    // SD card write packet size
char          fileName[24]    = {0};    // Log file name. Limited to 8.3 characters. Format: YYYYMMDD/HHMMSS.ubx
char          dirName[9]      = {0};    // Log file directory name. Format: YYYYMMDD
int           numBytes        = 0;      // Not sure
int           maxSerialBuffer = 0;      // Maximum size of available SerialBuffer
int           valFix          = 0;      // GNSS valid fix counter
long          bytesWritten    = 0;      // SD card write byte counter
float         voltage         = 0.0;    // Battery voltage
uint8_t       serBuffer[sdPacket];      // Buffer for SD card writes
unsigned long previousMillis  = 0;      // Global millis() timer variable
size_t        bufferPointer   = 0;      // Size of serBuffer pointer for SD card writes

// Enumerated switch statements
enum LoopSwitch {
  INIT,
  START_UBX,
  OPEN_FILE,
  WRITE_FILE,
  NEW_FILE,
  CLOSE_FILE,
  RESTART_FILE,
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

// Default switch cases
LoopSwitch loopStep = INIT;
ParseSwitch parseStep = PARSE_UBX_SYNC_CHAR_1;

// Global variables for UBX/NMEA parsing
int ubxLength             = 0;
int ubxClass              = 0;
int ubxId                 = 0;
int ubxChecksumA          = 0;
int ubxChecksumB          = 0;
int ubxExpectedChecksumA  = 0;
int ubxExpectedChecksumB  = 0;
int nmeaAddress1          = '0'; // e.g. G
int nmeaAddress2          = '0'; // e.g. P
int nmeaAddress3          = '0'; // e.g. G
int nmeaAddress4          = '0'; // e.g. G
int nmeaAddress5          = '0'; // e.g. A
int nmeaChecksum          = 0;
int nmeaChecksum1         = '0';
int nmeaChecksum2         = '0';
int nmea_expected_csum1   = '0';
int nmea_expected_csum2   = '0';
const int maxNmeaLength   = 100; // Maximum length for an NMEA message used to detect lost sync

// Definitions for u-blox F9P UBX-format (binary) messages

// Configure u-blox ZED-F9P receiver for ON/OFF operation (UBX-CFG-PM2)
static uint8_t ubxCfgPm2_payload[] = {
  0x01, 0x06, 0x00, 0x00, 0x6E, 0x00, 0x42, 0x01, 0xE8, 0x03, 0x00,
  0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86,
  0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00
};

ubxPacket ubxCfgPm2 = {
  0x06, 0x3b, 44, 0, 0, ubxCfgPm2_payload, 0, 0, false
};

// Satellite systems (GNSS) signal configuration. Enable GPS/GLO and disable GAL/BDS/QZSS. Warning: Will cause receiver reset!
uint8_t configureGnss() {
  gnss.newCfgValset8(0x1031001f, 0x01, VAL_LAYER_RAM); // CFG-SIGNAL-GPS_ENA
  gnss.addCfgValset8(0x10310001, 0x01);                // CFG-SIGNAL-GPS_L1CA_ENA
  gnss.addCfgValset8(0x10310003, 0x01);                // CFG-SIGNAL-GPS_L2C_ENA
  gnss.addCfgValset8(0x10310021, 0x00);                // CFG-SIGNAL-GAL_ENA
  gnss.addCfgValset8(0x10310007, 0x00);                // CFG-SIGNAL-GAL_E1_ENA
  gnss.addCfgValset8(0x1031000a, 0x00);                // CFG-SIGNAL-GAL_E5B_ENA
  gnss.addCfgValset8(0x10310022, 0x00);                // CFG-SIGNAL-BDS_ENA
  gnss.addCfgValset8(0x1031000d, 0x00);                // CFG-SIGNAL-BDS_B1_ENA
  gnss.addCfgValset8(0x1031000e, 0x00);                // CFG-SIGNAL-BDS_B2_ENA
  gnss.addCfgValset8(0x10310024, 0x00);                // CFG-SIGNAL-QZSS_ENA
  gnss.addCfgValset8(0x10310012, 0x00);                // CFG-SIGNAL-QZSS_L1CA_ENA
  gnss.addCfgValset8(0x10310015, 0x00);                // CFG-SIGNAL-QZSS_L2C_ENA
  gnss.addCfgValset8(0x10310025, 0x01);                // CFG-SIGNAL-GLO_ENA
  gnss.addCfgValset8(0x10310018, 0x01);                // CFG-SIGNAL-GLO_L1_ENA
  return gnss.sendCfgValset8(0x1031001a, 0x01);        // CFG-SIGNAL-GLO_L2_ENA
}

// Disable NMEA output on the I2C port
uint8_t disableI2cNmea() {
  return gnss.setVal8(0x10720002, 0x00, VAL_LAYER_RAM);
}

// Set UART1 to 230400 baud
uint8_t setUart1Baud() {
  return gnss.setVal32(0x40520001, 0x00038400, VAL_LAYER_RAM); // CFG-UART1-BAUDRATE - 0x00038400 = 230400 decimal
}

// Set UART2 to 230400 baud
uint8_t setUart2Baud230400() {
  return gnss.setVal32(0x40530001, 0x00038400, VAL_LAYER_RAM);  // CFG-UART2-BAUDRATE - 0x00038400 = 230400 decimal
}

// Set UART2 to 115200 baud
uint8_t setUart2Baud115200() {
  return gnss.setVal32(0x40530001, 0x0001c200, VAL_LAYER_RAM);  // CFG-UART2-BAUDRATE - 0x0001c200 = 230400 decimal
}

// Disables all messages logged to the SD card
uint8_t disableUbx() {
  gnss.newCfgValset8(0x209102a5, 0x00, VAL_LAYER_RAM);  // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  gnss.addCfgValset8(0x20910232, 0x00);                 // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
  gnss.addCfgValset8(0x20910179, 0x00);                 // CFG-MSGOUT-UBX_TIM_TM2_UART1
  gnss.addCfgValset8(0x2091002a, 0x00);                 // CFG-MSGOUT-UBX_NAV_POSLLH_UART1
  gnss.addCfgValset8(0x20910007, 0x00);                 // CFG-MSGOUT-UBX_NAV_PVT_UART1
  gnss.addCfgValset8(0x2091001b, 0x00);                 // CFG-MSGOUT-UBX_NAV_STATUS_UART1
  gnss.addCfgValset8(0x20930031, 0x01);                 // CFG-NMEA-MAINTALKERID            Sets the main talker ID to GP
  gnss.addCfgValset8(0x10930006, 0x00);                 // CFG-NMEA-HIGHPREC                Disables NMEA high-precision mode
  return gnss.sendCfgValset8(0x209100bb, 0x00);         // CFG-MSGOUT-NMEA_ID_GGA_UART1     Disables the GGA message
}


// Enable UBX RAWX and SFRBX messages on UART1
uint8_t enableUbx() {
  gnss.newCfgValset8(0x209102a5, 0x01, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  return gnss.sendCfgValset8(0x20910232, 0x01);        // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
}

/*
  // Enables all messages to be logged to the SD card
  uint8_t enableUbx() {
  gnss.newCfgValset8(0x209102a5, 0x01, VAL_LAYER_RAM);  // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  gnss.addCfgValset8(0x20910232, 0x01);                 // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
  gnss.addCfgValset8(0x20910179, 0x01);                 // CFG-MSGOUT-UBX_TIM_TM2_UART1
  gnss.addCfgValset8(0x2091002a, 0x01);                 // CFG-MSGOUT-UBX_NAV_POSLLH_UART1
  gnss.addCfgValset8(0x20910007, 0x01);                 // CFG-MSGOUT-UBX_NAV_PVT_UART1
  gnss.addCfgValset8(0x2091001b, 0x01);                 // CFG-MSGOUT-UBX_NAV_STATUS_UART1
  gnss.addCfgValset8(0x20930031, 0x03);                 // CFG-NMEA-MAINTALKERID            Sets the main talker ID to GN
  gnss.addCfgValset8(0x10930006, 0x01);                 // CFG-NMEA-HIGHPREC                Enables NMEA high-precision mode
  return gnss.sendCfgValset8(0x209100bb, 0x00);         // CFG-MSGOUT-NMEA_ID_GGA_UART1     Enables the GGA mesage
  }
*/

// Enable NMEA messages on UART1
uint8_t enableNmea() {
  gnss.newCfgValset8(0x209100bb, 0x01, VAL_LAYER_RAM);  // CFG-MSGOUT-NMEA_ID_GGA_UART1
  gnss.addCfgValset8(0x209100ca, 0x01);                 // CFG-MSGOUT-NMEA_ID_GLL_UART1
  gnss.addCfgValset8(0x209100c0, 0x01);                 // CFG-MSGOUT-NMEA_ID_GSA_UART1
  gnss.addCfgValset8(0x209100c5, 0x01);                 // CFG-MSGOUT-NMEA_ID_GSV_UART1
  gnss.addCfgValset8(0x209100b1, 0x01);                 // CFG-MSGOUT-NMEA_ID_VTG_UART1
  gnss.addCfgValset8(0x209100ac, 0x01);                 // CFG-MSGOUT-NMEA_ID_RMC_UART1
  return gnss.sendCfgValset8(0x209100b1, 0x07);         // CFG-INFMSG-NMEA_UART1
}

// Disable NMEA messages on UART1
uint8_t disableNmea() {
  gnss.newCfgValset8(0x209100bb, 0x00, VAL_LAYER_RAM);  // CFG-MSGOUT-NMEA_ID_GGA_UART1
  gnss.addCfgValset8(0x209100ca, 0x00);                 // CFG-MSGOUT-NMEA_ID_GLL_UART1
  gnss.addCfgValset8(0x209100c0, 0x00);                 // CFG-MSGOUT-NMEA_ID_GSA_UART1
  gnss.addCfgValset8(0x209100c5, 0x00);                 // CFG-MSGOUT-NMEA_ID_GSV_UART1
  gnss.addCfgValset8(0x209100b1, 0x00);                 // CFG-MSGOUT-NMEA_ID_VTG_UART1
  gnss.addCfgValset8(0x209100ac, 0x00);                 // CFG-MSGOUT-NMEA_ID_RMC_UART1
  return gnss.sendCfgValset8(0x209100b1, 0x00);         // CFG-INFMSG-NMEA_UART1
}

// Set the Main NMEA Talker ID to "GP"
uint8_t setTalkerId() {
  return gnss.setVal8(0x20930031, 0x01, VAL_LAYER_RAM); // CFG-NMEA-MAINTALKERID
}

// Set time between GNSS measurements (CFG-RATE-MEAS)
uint8_t setRate20Hz() {
  return gnss.setVal16(0x30210001, 0x0032, VAL_LAYER_RAM);
}
uint8_t setRate10Hz() {
  return gnss.setVal16(0x30210001, 0x0064, VAL_LAYER_RAM);
}
uint8_t setRate5Hz() {
  return gnss.setVal16(0x30210001, 0x00c8, VAL_LAYER_RAM);
}
uint8_t setRate4Hz() {
  return gnss.setVal16(0x30210001, 0x00fa, VAL_LAYER_RAM);
}
uint8_t setRate2Hz() {
  return gnss.setVal16(0x30210001, 0x01f4, VAL_LAYER_RAM);
}
uint8_t setRate1Hz() {
  return gnss.setVal16(0x30210001, 0x03e8, VAL_LAYER_RAM);
}

// Set the navigation dynamic model
// UBX-CFG-VALSET message with a key ID of 0x20110021 (CFG-NAVSPG-DYNMODEL)
uint8_t setNavPortable() {
  return gnss.setVal8(0x20110021, 0x00, VAL_LAYER_RAM);
};
uint8_t setNavStationary() {
  return gnss.setVal8(0x20110021, 0x02, VAL_LAYER_RAM);
};
uint8_t setNavPedestrian() {
  return gnss.setVal8(0x20110021, 0x03, VAL_LAYER_RAM);
};
uint8_t setNavAutomotive() {
  return gnss.setVal8(0x20110021, 0x04, VAL_LAYER_RAM);
};
uint8_t setNavSea() {
  return gnss.setVal8(0x20110021, 0x05, VAL_LAYER_RAM);
};
uint8_t setNavAir1g() {
  return gnss.setVal8(0x20110021, 0x06, VAL_LAYER_RAM);
};
uint8_t setNavAir2g() {
  return gnss.setVal8(0x20110021, 0x07, VAL_LAYER_RAM);
};
uint8_t setNavAir4g() {
  return gnss.setVal8(0x20110021, 0x08, VAL_LAYER_RAM);
};
uint8_t setNavWrist() {
  return gnss.setVal8(0x20110021, 0x09, VAL_LAYER_RAM);
};

// Set Survey_In mode
uint8_t setSurveyIn() {
  gnss.newCfgValset8(0x20030001, 0x01, VAL_LAYER_RAM);  // CFG-TMODE-MODE
  gnss.addCfgValset32(0x40030011, 0x0000c350);          // CFG-TMODE-SVIN_ACC_LIMIT   0x0000c350 = 50000 decimal = 5 min
  return gnss.sendCfgValset32(0x40030010, 0x0000003c);  // CFG-TMODE-SVIN_MIN_DUR     0x0000003c = 60 decimal = 1 min
}

// Disable Survey_In mode
uint8_t disableSurveyIn() {
  return gnss.setVal8(0x20030001, 0x00, VAL_LAYER_RAM); // CFG-TMODE-MODE
}

// Enable RTCM message output on UART2
// Set the value byte to 0x01 to send an RTCM message at RATE_MEAS; set the value to 0x04 to send an RTCM message at 1/4 RATE_MEAS
// (i.e. assumes you will be logging RAWX data at 4 Hz. Adjust accordingly)
uint8_t enableRtcm() {
  gnss.newCfgValset8(0x209102bf, 0x04, VAL_LAYER_RAM);  // CFG-MSGOUT-RTCM_3X_TYPE1005_UART2
  gnss.addCfgValset8(0x209102ce, 0x04);                 // CFG-MSGOUT-RTCM_3X_TYPE1077_UART2
  gnss.addCfgValset8(0x209102d3, 0x04);                 // CFG-MSGOUT-RTCM_3X_TYPE1087_UART2
  gnss.addCfgValset8(0x209102d8, 0x04);                 // CFG-MSGOUT-RTCM_3X_TYPE1127_UART2
  gnss.addCfgValset8(0x2091031a, 0x04);                 // CFG-MSGOUT-RTCM_3X_TYPE1097_UART2
  return gnss.sendCfgValset8(0x20910305, 0x28);         // CFG-MSGOUT-RTCM_3X_TYPE1230_UART2
}

// Disable RTCM message output on UART2
uint8_t disableRtcm() {
  gnss.newCfgValset8(0x209102bf, 0x00, VAL_LAYER_RAM);  // CFG-MSGOUT-RTCM_3X_TYPE1005_UART2
  gnss.addCfgValset8(0x209102ce, 0x00);                 // CFG-MSGOUT-RTCM_3X_TYPE1077_UART2
  gnss.addCfgValset8(0x209102d3, 0x00);                 // CFG-MSGOUT-RTCM_3X_TYPE1087_UART2
  gnss.addCfgValset8(0x209102d8, 0x00);                 // CFG-MSGOUT-RTCM_3X_TYPE1127_UART2
  gnss.addCfgValset8(0x2091031a, 0x00);                 // CFG-MSGOUT-RTCM_3X_TYPE1097_UART2
  return gnss.sendCfgValset8(0x20910305, 0x00);         // CFG-MSGOUT-RTCM_3X_TYPE1230_UART2
}

// Set TimeGrid for TP1 to GPS (instead of UTC) so TIM_TM2 messages are aligned with GPS time
uint8_t setTimeGrid() {
  return gnss.setVal8(0x2005000c, 0x01, VAL_LAYER_RAM); // CFG-TP-TIMEGRID_TP1  0x01 = GPS
}

// Enable NMEA messages on UART2 for test purposes
// UBX-CFG-VALSET message with key ID of 0x10760002 (CFG-UART2OUTPROT-NMEA) and value of 1:
uint8_t setUART2nmea() {
  return gnss.setVal8(0x10760002, 0x01, VAL_LAYER_RAM);
}

// 'Disable' timepulse TP1 by setting LEN_LOCK_TP1 to zero
// (This doesn't actually disable the timepulse, it just sets its length to zero!)
uint8_t disableTP1() {
  return gnss.setVal32(0x40050005, 0x00, VAL_LAYER_RAM); // CFG-TP-LEN_LOCK_TP1
}

// Define SerialBuffer as a large RingBuffer which we will use to store the Serial1 receive data
// Actual Serial1 receive data will be copied into SerialBuffer by a timer interrupt
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
// That way, we do not need to increase the size of the Serial1 receive buffer (by editing RingBuffer.h)
// You can use DEBUG_SERIAL_BUFFER to determine how big the buffer should be. Increase it if you see bufAvail get close to or reach the buffer size.
RingBufferN<16384> SerialBuffer; // Define SerialBuffer as a RingBuffer of size 16k bytes

// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer
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

  // Enable the compare interruptl
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

// Setup
void setup() {

  // Pin configuration
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);    // Input for the Base/Rover mode select switch
  pinMode(GPS_INT_PIN, OUTPUT);       // Control ZED-F9P ON/OFF operation
  pinMode(SW_PIN, INPUT_PULLUP);      // Input for the stop switch
  pinMode(SURVEY_PIN, INPUT_PULLUP);  // Input for the SURVEY_IN switch

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(MODE_PIN, LOW); // Manually set to base mode

  // Blink LEDs on reset
  for (byte i = 0; i < 20; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }

  // Configure the Watchdog Timer to perform a system reset if loop() blocks for more than 8-16 seconds
  configureWatchdog();

  // Start Serial at 115200 baud
  Serial.begin(115200);
  //while (!Serial);    // Wait for user to open Serial Monitor
  delay(5000);       // Delay to allow user to open Serial Monitor

  Serial.println("u-blox ZED-F9P Raw Datalogger");
  Serial.println("-----------------------------");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // Increase clock frequency for I2C communications to 400 kHz

  // Initialize the RTC
  rtc.begin();

  // Initialize the SD card
  if (sd.begin(chipSelect, SD_SCK_MHZ(4))) {
    Serial.println("SD card initialized.");
  }
  else {
    Serial.println("Warning: Unable to initialize SD card. Halting!");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    while (1); // Halt the program
  }

  // Initialize the u-blox ZED-F9P
  if (gnss.begin() == true) {
    Serial.println("u-blox ZED-F9P initialized.");
    //gnss.sendCommand(ubxCfgPm2);  // Configure UBX-CFG-PM2 for ON/OFF operation
    //gnss.saveConfiguration();     // Save current settings to flash and BBR
#if DEBUG_I2C
    gnss.enableDebugging();       // Enable I2C debug output to Serial Monitor
#endif
  }
  else {
    Serial.println("Warning: u-blox ZED-F9P not detected at default I2C address. Please check wiring. Halting!");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    while (1); // Halt the program
  }

  // Configure u-blox ZED-F9P
  // Messages:
  // Acknowledged:      "Received: CLS:5 ID:1 Payload: 6 8A" (UBX-ACK-ACK (0x05 0x01)
  // Not-Acknowledged:  UBX-ACK-NAK (0x05 0x00)
  // Payload:           UBX-CFG-VALSET (0x06 0x8A)

  // These sendCommands will timeout as the commandAck checking in processUBXpacket expects the packet to be in packetCfg, not our custom packet!
  // Turn on DEBUG to see if the commands are acknowledged (Received: CLS:ACK ID:1 Len: 0x2 Payload: 6 8A) or not acknowledged (CLS:5 ID:0)
  boolean setValueSuccess = true;

  setValueSuccess &= configureGnss();       // Configure GNSS constellations
  setValueSuccess &= disableI2cNmea();      // Disable NMEA messages on the I2C port leaving it clear for UBX messages
  setValueSuccess &= setUart1Baud();        // Change the UART1 baud rate to 230400
  setValueSuccess &= disableUbx();          // Disable UBX messages and NMEA high precision mode on UART1
  setValueSuccess &= disableNmea();         // Disable NMEA messages on UART1
  setValueSuccess &= setTalkerId();         // Set NMEA TALKERID to GP
  setValueSuccess &= setRate1Hz();          // Set Navigation/Measurement Rate to 1Hz
  setValueSuccess &= setUart2Baud115200();  // Set UART2 baud rate
  setValueSuccess &= disableSurveyIn();     // Disable Survey_In mode
  setValueSuccess &= disableRtcm();         // Disable RTCM output on UART2
  setValueSuccess &= setTimeGrid();         // Set the TP1 TimeGrid to GPS so TIM_TM2 messages are aligned with GPS time

  if (setValueSuccess == true) {
    Serial.println("u-blox ZED-F9P configured.");
  }
  else {
    Serial.println("Warning: Unable to configure u-blox ZED-F9P. Halting!");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    while (1); // Halt the program
  }

  // Check the MODE_PIN and set the navigation dynamic model
  if (digitalRead(MODE_PIN) == LOW) {
    Serial.println("BASE mode selected");
    setNavStationary(); // Set Static Navigation Mode (use this for the Base Logger)
  }
  else {
    modeFlag = false; // Clear modeFlag flag
    Serial.println("ROVER mode selected");
    // Select rover mode
    //setNavPortable();   // Set Portable Navigation Mode
    //setNavPedestrian(); // Set Pedestrian Navigation Mode
    //setNavAutomotive(); // Set Automotive Navigation Mode
    //setNavSea();        // Set Sea Navigation Mode
    //setNavAir1g();      // Set Airborne <1G Navigation Mode
    //setNavAir2g();      // Set Airborne <2G Navigation Mode
    //setNavAir4g();      // Set Airborne <4G Navigation Mode
    //setNavWrist();      // Set Wrist Navigation Mode
  }

#if !LED_ON
  disableTP1(); // Disable the timepulse to prevent the LED from flashing
#endif

  // Start Serial1 at 230400 baud
  Serial1.begin(230400);

#if DEBUG
  // Print RTC's current date and time
  Serial.print("Current RTC date and time: "); printDateTime();

  // Wait for GNSS fix
  Serial.println("Awaiting GNSS fix...");
#endif
}

// Loop
void loop() {

  switch (loopStep) {
    case INIT: {

        // Pet the dog
        resetWatchdog();

#if DEBUG
        char gnssDateTime[24]; // GNSS date and time buffer
        snprintf(gnssDateTime, sizeof(gnssDateTime), "%04u-%02d-%02d %02d:%02d:%02d",
                 gnss.getYear(), gnss.getMonth(), gnss.getDay(),
                 gnss.getHour(), gnss.getMinute(), gnss.getSecond());

        int32_t latitude = gnss.getLatitude();
        int32_t longitude = gnss.getLongitude();
        uint16_t pdop = gnss.getPDOP();
        uint8_t fix = gnss.getFixType();
        uint8_t satellites = gnss.getSIV();

        Serial.print(gnssDateTime);
        Serial.print(" Latitude: "); Serial.print(latitude);
        Serial.print(" Longitude: "); Serial.print(longitude);
        Serial.print(" Satellites: "); Serial.print(satellites);
        Serial.print(" Fix: "); Serial.print(fix);
        Serial.print(" PDOP: "); Serial.println(pdop);
#endif

        // Read battery voltage
        voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);
#if DEBUG
        //Serial.print("Battery: "); Serial.print(voltage, 2); Serial.println("V");
#endif

        // Has a GNSS fix been acquired?
        if (gnss.getFixType() > 0) {
#if LED_ON
          digitalWrite(LED_PIN, HIGH); // Turn LED ON to indicate GNSS fix
#endif
          // Increment valFix and cap at maxValFix
          // don't do anything fancy in terms of decrementing valFix as we want to keep logging even if the fix is lost
          valFix += 1;
          if (valFix > maxValFix) valFix = maxValFix;
        }
        else {
#if LED_ON
          digitalWrite(LED_PIN, LOW); // Turn LED OFF to indicate loss of GNSS fix
#endif
        }

        // Have enough valid GNSS fixes been collected?
        if (valFix == maxValFix) {

          // Set the RTC's date and time
          rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond());    // Set the time
          rtc.setDate(gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000); // Set the date
          Serial.print("RTC set: "); printDateTime();

          // Set the RTC's alarm
          alarmFlag = false; // Clear alarm flag
#if ALARM_MIN
          // Rolling-minutes alarm
          rtc.setAlarmTime(0, (rtc.getMinutes() + alarmMinutes) % 60, 0); // Set alarm time(hour, minute, second)
          rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());  // Set alarm date (day, month, year)
          rtc.enableAlarm(rtc.MATCH_MMSS);                                // Alarm match on minutes and seconds
#endif
#if ALARM_HR
          // Rolling-hour alarm
          rtc.setAlarmTime((rtc.getHours() + alarmHours) % 24, 0, 0);     // Set alarm time (hours, minutes, seconds)
          rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());  // Set alarm date (day, month, year)
          rtc.enableAlarm(rtc.MATCH_HHMMSS);                              // Alarm match on hours, minutes and seconds
#endif
#if ALARM_DAY
          // Rolling-day alarm
          rtc.setAlarmTime(0, 0, 0);                                      // Set alarm time (hours, minutes, seconds)
          rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());  // Set alarm date (day, month, year)
          rtc.enableAlarm(rtc.MATCH_HHMMSS);                              // Alarm match on hours, minutes and seconds
#endif
          rtc.attachInterrupt(alarmMatch);  // Attach alarm interrupt
          Serial.print("Next alarm: "); printAlarm();

          // Halt logging if battery voltage is below lowVoltage threshold
          if (voltage < lowVoltage) {
            Serial.println("Warning: Low battery!");
            loopStep = SLEEP;
            break;
          }
          /*
            // If we are in BASE mode, check the SURVEY_IN pin
            if (modeFlag == true) {
              if (digitalRead(SURVEY_PIN) == LOW) {
                // We are in BASE mode and the SURVEY_IN pin is low so send the extra UBX messages:
                Serial.println("SURVEY_IN mode selected");
                surveyFlag = true;  // Set the surveyFlag flag true
                enableRtcm();       // Enable RTCM messages on UART2
                delay(1100);
                setSurveyIn();      // Enable SURVEY_IN mode
                delay(1100);
              }
            }
          */
          // Flush RX buffer to clear any old data
          while (Serial1.available()) {
            Serial1.read();
          }

          // Now that Serial1 should be idle and the buffer empty, start TC3 interrupts to copy all new data into SerialBuffer
          // Set the timer interval to 10 * 10 / 230400 = 0.000434 secs (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 230400 baud)
          startTimerInterval(0.000434);

          // Start UBX data
          loopStep = START_UBX;
        }
      }
      break;

    // Start UBX messages
    case START_UBX: {

        Serial.println("Case: START_UBX");
        enableUbx(); // Start the UBX and NMEA messages
        bufferPointer = 0; // Initialise bufferPointer

        // Create a new log file
        loopStep = OPEN_FILE;
      }
      break;

    // Open the log file
    case OPEN_FILE: {
#if DEBUG
        Serial.println("Case: OPEN_FILE");
#endif
        // Pet the dog
        resetWatchdog();

        // Create a new folder
        snprintf(dirName, sizeof(dirName), "%04u%02d%02d", (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay());
        if (sd.mkdir(dirName)) {
          Serial.print("Created folder: ");
          Serial.println(dirName);
        }
        else {
          Serial.println("Warning: Unable to create new folder!");
        }

        // Create log file
        snprintf(fileName, sizeof(fileName), "%04u%02d%02d/%02d%02d%02d.ubx",
                 (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
        if (file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
          Serial.print("Logging to: ");
          Serial.println(fileName);
        }
        else {
          Serial.println("Warning: Unable to open new log file. Halting!");
          while (1); // Halt program
        }

        // Set the log file creation time
        if (!file.timestamp(T_CREATE, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file create timestamp!");
        }

        bytesWritten = 0;                   // Clear bytesWritten
        parseStep = PARSE_UBX_SYNC_CHAR_1;  // Set parseStep to expect 0xB5 or '$'
        ubxLength = 0;                      // Set ubxLength to zero

        // Begin logging data
        loopStep = WRITE_FILE;

      } // End case OPEN_FILE
      break;

    // Stuff bytes into serBuffer and write when we have reached sdPacket
    case WRITE_FILE: {

        int bufAvail = SerialBuffer.available();
        if (bufAvail > 0) {
#if DEBUG_SERIAL_BUFFER
          if (bufAvail > maxSerialBuffer) {
            maxSerialBuffer = bufAvail;
            Serial.print("Max bufAvail: ");
            Serial.println(maxSerialBuffer);
          }
#endif
          // Read bytes on UART1
          uint8_t c = SerialBuffer.read_char();

          // Write bytes to serBuffer
          serBuffer[bufferPointer] = c;

#if DEBUG_UBX
          // Output UBX hex data to Serial Monitor
          char dataString[2] = {0};
          sprintf(dataString, "%02X ", c);
          Serial.print(dataString);
#endif

          bufferPointer++;
          if (bufferPointer == sdPacket) {
            bufferPointer = 0;
            numBytes = file.write(&serBuffer, sdPacket);
            file.sync(); // Sync the file system
            bytesWritten += sdPacket;

            // Blink the LED to indicate SD write
            blinkLed(1, 50);
#if DEBUG
            if (numBytes != sdPacket) {
              Serial.print("Warning: SD write error. Only ");
              Serial.print(sdPacket); Serial.print("/");
              Serial.print(numBytes); Serial.println(" bytes were written.");
            }
#endif
            digitalWrite(LED_BUILTIN, LOW);
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
          // Only allow a new file to be opened when a complete packet has been processed and parseStep
          // has returned to "PARSE_UBX_SYNC_CHAR_1" or when a data error is detected (i.e. SYNC_LOST)
          switch (parseStep) {
            case (PARSE_UBX_SYNC_CHAR_1): {
                if (c == 0xB5) { // Have we found Sync Char 1 (0xB5) if we were expecting one?
                  parseStep = PARSE_UBX_SYNC_CHAR_2; // Now look for Sync Char 2 (0x62)
                }
                else if (c == '$') { // Have we found an NMEA '$' if we were expecting one?
                  parseStep = PARSE_NMEA_START_CHAR; // Now keep going until we receive an asterix
                  ubxLength = 0; // Reset ubxLength then use it to track which character has arrived
                  nmeaChecksum = 0; // Reset the nmeaChecksum. Update it as each character arrives
                  nmeaAddress1 = '0'; // Reset the first five NMEA chars to something invalid
                  nmeaAddress2 = '0';
                  nmeaAddress3 = '0';
                  nmeaAddress4 = '0';
                  nmeaAddress5 = '0';
                }
                else {
                  Serial.println("Warning: Was expecting Sync Char 0xB5 or an NMEA $ but did not receive one!");
                  parseStep = SYNC_LOST;
                }
              }
              break;
            case (PARSE_UBX_SYNC_CHAR_2): {
                if (c == 0x62) { // Have we found Sync Char 2 (0x62) when we were expecting one?
                  ubxExpectedChecksumA = 0; // Reset the expected checksum
                  ubxExpectedChecksumB = 0;
                  parseStep = PARSE_UBX_CLASS; // Now look for Class byte
                }
                else {
                  Serial.println("Warning: Was expecting Sync Char 0x62 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
              }
              break;
            // Message      Class   ID
            // RXM_RAWX     0x02    0x15
            // RXM_SFRBF    0x02    0x13
            // TIM_TM2      0x0d    0x03
            // NAV_POSLLH   0x01    0x02
            // NAV_PVT      0x01    0x07
            // NAV-STATUS   0x01    0x03
            case (PARSE_UBX_CLASS): {
                ubxClass = c;
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update the expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_ID; // Now look for ID byte
#if DEBUG
                // Class syntax checking
                if ((ubxClass != 0x02) and (ubxClass != 0x0d) and (ubxClass != 0x01)) {
                  Serial.println("Warning: Was expecting Class of 0x02 or 0x0d or 0x01 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
#endif
              }
              break;
            case (PARSE_UBX_ID): {
                ubxId = c;
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update the expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_LENGTH_LSB; // Now look for length LSB
#if DEBUG
                // ID syntax checking
                if ((ubxClass == 0x02) && ((ubxId != 0x15) && (ubxId != 0x13))) {
                  Serial.println("Warning: Was expecting ID of 0x15 or 0x13 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
                else if ((ubxClass == 0x0d) && (ubxId != 0x03)) {
                  Serial.println("Warning: Was expecting ID of 0x03 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
                else if ((ubxClass == 0x01) && ((ubxId != 0x02) && (ubxId != 0x07) && (ubxId != 0x03))) {
                  Serial.println("Warning: Was expecting ID of 0x02 or 0x07 or 0x03 but did not receive one!");
                  parseStep = SYNC_LOST;
                }
#endif
              }
              break;
            case (PARSE_UBX_LENGTH_LSB): {
                ubxLength = c; // Store the length LSB
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update the expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_LENGTH_MSB; // Now look for length MSB
              }
              break;
            case (PARSE_UBX_LENGTH_MSB): {
                ubxLength = ubxLength + (c * 256); // Add the length MSB
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update the expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                parseStep = PARSE_UBX_PAYLOAD; // Now look for payload bytes (length: ubxLength)
              }
              break;
            case (PARSE_UBX_PAYLOAD): {
                // If this is a NAV_PVT message, check the flags byte (byte offset 21) and report the carrSoln
                if ((ubxClass == 0x01) && (ubxId == 0x07)) { // Is this a NAV_PVT message (class 0x01 ID 0x07)?
                  if (ubxLength == 71) { // Is this byte offset 21? (ubxLength will be 92 for byte offset 0, so will be 71 for byte offset 21)
#if DEBUG_NAV
                    Serial.print("NAV_PVT carrSoln: ");
                    if ((c & 0xc0) == 0x00) {
                      Serial.println("None");
                    }
                    else if ((c & 0xc0) == 0x40) {
                      Serial.println("Floating");
                    }
                    else if ((c & 0xc0) == 0x80) {
                      Serial.println("Fixed");
                    }
#endif
                    // Have we got a fixed carrier solution?
                    if ((c & 0xc0) == 0x80) {
                    }
                  }
                }
                // If this is a NAV_STATUS message, check the gpsFix byte (byte offset 4) and flash the green LED if the fix is TIME
                if ((ubxClass == 0x01) && (ubxId == 0x03)) { // Is this a NAV_STATUS message (class 0x01 ID 0x03)?
                  if (ubxLength == 12) { // Is this byte offset 4? (ubxLength will be 16 for byte offset 0, so will be 12 for byte offset 4)
#if DEBUG_NAV
                    Serial.print("NAV_STATUS gpsFix: ");
                    if (c == 0x00) {
                      Serial.println("No fix");
                    }
                    else if (c == 0x01) {
                      Serial.println("Dead reckoning");
                    }
                    else if (c == 0x02) {
                      Serial.println("2D-fix");
                    }
                    else if (c == 0x03) {
                      Serial.println("3D-fix");
                    }
                    else if (c == 0x04) {
                      Serial.println("GPS + dead reckoning");
                    }
                    else if (c == 0x05) {
                      Serial.println("Time");
                    }
                    else {
                      Serial.println("Reserved");
                    }
#endif
                    // Have we got a TIME fix?
                    if (c == 0x05) {

                    }
                  }
                }
                ubxLength = ubxLength - 1; // Decrement length by one
                ubxExpectedChecksumA = ubxExpectedChecksumA + c; // Update the expected checksum
                ubxExpectedChecksumB = ubxExpectedChecksumB + ubxExpectedChecksumA;
                if (ubxLength == 0) {
                  ubxExpectedChecksumA = ubxExpectedChecksumA & 0xff; // Limit checksums to 8-bits
                  ubxExpectedChecksumB = ubxExpectedChecksumB & 0xff;
                  parseStep = PARSE_UBX_CHECKSUM_A; // If we have received length payload bytes, look for checksum bytes
                }
              }
              break;
            case (PARSE_UBX_CHECKSUM_A): {
                ubxChecksumA = c;
                parseStep = PARSE_UBX_CHECKSUM_B;
              }
              break;
            case (PARSE_UBX_CHECKSUM_B): {
                ubxChecksumB = c;
#if DEBUG_UBX
                Serial.println(); // Insert line break between UBX frames in Serial Monitor
#endif
                parseStep = PARSE_UBX_SYNC_CHAR_1; // All bytes received so go back to looking for a new Sync Char 1 unless there is a checksum error
                if ((ubxExpectedChecksumA != ubxChecksumA) or (ubxExpectedChecksumB != ubxChecksumB)) {
                  Serial.println("Warning: UBX checksum error!");
                  parseStep = SYNC_LOST;
                }
              }
              break;

            // NMEA sentence parsing
            case (PARSE_NMEA_START_CHAR): {
                ubxLength++; // Increase the message length count
                if (ubxLength > maxNmeaLength) { // If the length is greater than maxNmeaLength, something bad must have happened (SYNC_LOST)
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
                // Now check if this is an '*'
                if (c == '*') {
                  // Asterix received
                  // Don't exOR it into the checksum
                  // Instead calculate what the expected checksum should be (nmeaChecksum in ASCII hex)
                  nmea_expected_csum1 = ((nmeaChecksum & 0xf0) >> 4) + '0'; // Convert MS nibble to ASCII hex
                  if (nmea_expected_csum1 >= ':') {
                    nmea_expected_csum1 += 7;  // : follows 9 so add 7 to convert to A-F
                  }
                  nmea_expected_csum2 = (nmeaChecksum & 0x0f) + '0'; // Convert LS nibble to ASCII hex
                  if (nmea_expected_csum2 >= ':') {
                    nmea_expected_csum2 += 7;  // : follows 9 so add 7 to convert to A-F
                  }
                  // Next, look for the first csum character
                  parseStep = PARSE_NMEA_CHECKSUM_1;
                  break; // Don't include the * in the checksum
                }
                // Now update the checksum
                // The checksum is the exclusive-OR of all characters between the $ and the *
                nmeaChecksum = nmeaChecksum ^ c;
              }
              break;
            case (PARSE_NMEA_CHECKSUM_1): {
                // Store the first NMEA checksum character
                nmeaChecksum1 = c;
                parseStep = PARSE_NMEA_CHECKSUM_2;
              }
              break;
            case (PARSE_NMEA_CHECKSUM_2): {
                // Store the second NMEA checksum character
                nmeaChecksum2 = c;
                // Now check if the checksum is correct
                if ((nmeaChecksum1 != nmea_expected_csum1) or (nmeaChecksum2 != nmea_expected_csum2)) {
                  // The checksum does not match so SYNC_LOST
                  Serial.println("Warning: NMEA checksum error!");
                  parseStep = SYNC_LOST;
                }
                else {
                  // Checksum was valid so wait for the terminators
                  parseStep = PARSE_NMEA_END_1;
                }
              }
              break;
            case (PARSE_NMEA_END_1): {
                // Check if this is CR
                if (c != '\r') {
                  Serial.println("Warning: NMEA CR not found!");
                  parseStep = SYNC_LOST;
                }
                else {
                  parseStep = PARSE_NMEA_END_2;
                }
              }
              break;
            case (PARSE_NMEA_END_2): {
                // Check if this is LF
                if (c != '\n') {
                  Serial.println("Warning: NMEA LF not found!");
                  parseStep = SYNC_LOST;
                }
                else {
                  // LF was received so go back to looking for B5 or a $
                  parseStep = PARSE_UBX_SYNC_CHAR_1;
                }
              }
              break;
          }
        }
        else {
          // Read battery voltage
          voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0);
        }

        // Check for conditions that would halt logging
        /*
          // Check if stop button was pressed
          if (digitalRead(SW_PIN) == LOW) {
            stopFlag = true;
            loopStep = CLOSE_FILE; // Close the log file
            break;
          }
          // Check for low battery voltage
          else if (voltage < lowVoltage) {
            voltageFlag = true;
            loopStep = CLOSE_FILE; // Close the file
            break;
          }
        */
        // Check if RTC alarm was triggered
        if ((alarmFlag == true) && (parseStep == PARSE_UBX_SYNC_CHAR_1)) {
          Serial.print("RTC alarm: "); printDateTime();
          loopStep = NEW_FILE; // Close the file and open a new one
          break;
        }
        // Check if sync was lost
        else if (parseStep == SYNC_LOST) {
          Serial.println("Restarting file due to sync loss");
          loopStep = RESTART_FILE; // Sync has been lost so halt UBX messages and create a new file before restarting UBX
        }
      }
      break;

    // Close the current log file and open a new one without stopping UBX messages
    case NEW_FILE: {
#if DEBUG
        Serial.println("Case: NEW_FILE");
#endif
        // If there is any data left in serBuffer, write it to file
        if (bufferPointer > 0) {
          numBytes = file.write(&serBuffer, bufferPointer); // Write remaining data
          file.sync(); // Sync the file system
          bytesWritten += bufferPointer;

          // Blink the LED to indicate SD write
          blinkLed(1, 50);
#if DEBUG
          if (numBytes != sdPacket) {
            Serial.print("Warning: Incomplete SD write. ");
            Serial.print(numBytes); Serial.print("/");
            Serial.print(sdPacket); Serial.println(" bytes were written.");
          }
#endif
          bufferPointer = 0; // Reset bufferPointer
        }

        // Set the log file's last write/modification date and time
        if (!file.timestamp(T_WRITE, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file write timestamp!");
        }
        // Set log file's last access date and time
        if (!file.timestamp(T_ACCESS, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file access timestamp!");
        }

        // Close the file
        file.close();

#if DEBUG
        Serial.print("File closed!");
        uint32_t filesize = file.fileSize(); // Get the file size
        Serial.print(" File size: "); Serial.print(filesize);
        Serial.print(". Expected file size: "); Serial.println(bytesWritten);
#endif


        // Synchronize the RTC with GPS time
        if (gnss.getFixType() > 0) {
          // Set the RTC's date and time
          rtc.setTime(gnss.getHour(), gnss.getMinute(), gnss.getSecond());    // Set the time
          rtc.setDate(gnss.getDay(), gnss.getMonth(), gnss.getYear() - 2000); // Set the date
          Serial.print("RTC set: "); printDateTime();
        }

        // An RTC alarm was detected, so set the RTC alarm time to the next alarmInterval and loop back to OPEN_FILE.
        // Set the RTC's alarm
        alarmFlag = false; // Clear the RTC alarm flag
#if ALARM_MIN
        // Rolling-minutes alarm
        rtc.setAlarmTime(0, (rtc.getMinutes() + alarmMinutes) % 60, 0); // Set alarm time(hour, minute, second)
        rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());  // Set alarm date (day, month, year)
        rtc.enableAlarm(rtc.MATCH_MMSS);                                // Alarm match on minutes and seconds
#endif
#if ALARM_HR
        // Rolling-hour alarm
        rtc.setAlarmTime((rtc.getHours() + alarmHours) % 24, 0, 0);     // Set alarm time (hours, minutes, seconds)
        rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());  // Set alarm date (day, month, year)
        rtc.enableAlarm(rtc.MATCH_HHMMSS);                              // Alarm match on hours, minutes and seconds
#endif
#if ALARM_DAY
        // Rolling-day alarm
        rtc.setAlarmTime(0, 0, 0);                                      // Set alarm time (hours, minutes, seconds)
        rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());  // Set alarm date (day, month, year)
        rtc.enableAlarm(rtc.MATCH_HHMMSS);                              // Alarm match on hours, minutes and seconds
#endif
        rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
#if DEBUG
        Serial.print("Next alarm: "); printAlarm();
#endif

        loopStep = OPEN_FILE; // Loop to open a new file
        bytesWritten = 0;     // Clear bytesWritten
      }
      break;

    // Disable UBX messages, save any residual data and close the file
    case CLOSE_FILE: {
#if DEBUG
        Serial.println("Case: CLOSE_FILE");
#endif
        disableUbx(); // Disable UBX messages
        int waitcount = 0;
        // Wait for residual data
        while (waitcount < dwell) {
          while (SerialBuffer.available()) {
            serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
            bufferPointer++;
            // Write a full packet
            if (bufferPointer == sdPacket) {
              bufferPointer = 0;
              numBytes = file.write(&serBuffer, sdPacket);
              file.sync(); // Sync the file system
              bytesWritten += sdPacket;

              // Blink the LED to indicate SD write
              blinkLed(1, 50);

#if DEBUG
              if (numBytes != sdPacket) {
                Serial.print("Warning: SD write error. Only ");
                Serial.print(numBytes); Serial.print("/");
                Serial.print(sdPacket); Serial.println(" bytes were written.");
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

          // Blink the LED to indicate SD write
          blinkLed(1, 50);

#if DEBUG
          if (numBytes != sdPacket) {
            Serial.print("Warning: Incomplete SD write. ");
            Serial.print(numBytes); Serial.print("/");
            Serial.print(sdPacket); Serial.println(" bytes were written.");
          }
          Serial.print("Final SD write: "); Serial.print(bufferPointer);
          Serial.print(" bytes. Total bytes written: "); Serial.println(bytesWritten);
#endif
          // Reset bufferPointer
          bufferPointer = 0;
        }

        // Set the log file's last write/modification date and time
        if (!file.timestamp(T_WRITE, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file write timestamp!");
        }
        // Set log file's last access date and time
        if (!file.timestamp(T_ACCESS, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file access timestamp!");
        }

        // Close the file
        file.close();

#if DEBUG
        Serial.print("File closed!");
        uint32_t filesize = file.fileSize(); // Get the file size
        Serial.print(" File size: "); Serial.print(filesize);
        Serial.print(". Expected file size: "); Serial.println(bytesWritten);
#endif
        /*
          // Either the battery is low or the user pressed the stop button:
          if (stopFlag == true) {
            Serial.println("Waiting for reset...");
            while (1); // Wait for reset
          }
          else {
            // Low battery was detected so wait for the battery to recover
            Serial.println("Battery must be low - waiting for it to recover...");

            // Check the battery voltage. Make sure it has been OK for at least 5 seconds before continuing
            int voltageTimer = 0;
            while (voltageTimer < 500) {
              voltage = analogRead(A7) * (2.0 * 3.3 / 1023.0); // Read battery voltage
              if (voltage < lowVoltage) {
                voltageTimer = 0; // If battery voltage is low, reset the count
              }
              else {
                voltageTimer++; // Increase the count
              }
              delay(10); // Wait 10 msec
            }
          }
        */
        // Loop to restart UBX messages before opening a new file
        loopStep = START_UBX;
      }
      break;

    // If UBX sync is lost, disable UBX messages, save any residual data, close the file, create another and restart UBX messages. Do not update RTC alarm
    case RESTART_FILE: {

        Serial.println("RESTART_FILE");

        // Pet the dog
        resetWatchdog();

        // Disable UBX messages
        disableUbx();

        int waitcount = 0;
        // Wait for residual data
        while (waitcount < dwell) {
          while (SerialBuffer.available()) {
            serBuffer[bufferPointer] = SerialBuffer.read_char(); // Place extra bytes into serBuffer
            bufferPointer++;
            // Write a full packet
            if (bufferPointer == sdPacket) {
              bufferPointer = 0;
              numBytes = file.write(&serBuffer, sdPacket);
              file.sync(); // Sync the file system
              bytesWritten += sdPacket;

              // Blink the LED to indicate SD write
              blinkLed(1, 50);
#if DEBUG
              if (numBytes != sdPacket) {
                Serial.print("Warning: Incomplete SD write. ");
                Serial.print(numBytes); Serial.print("/");
                Serial.print(sdPacket); Serial.println(" bytes were written.");
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

          blinkLed(1, 50);  // Blink LED to indicate SD card write

#if DEBUG
          if (numBytes != sdPacket) {
            Serial.print("Warning: Incomplete SD write. ");
            Serial.print(numBytes); Serial.print("/");
            Serial.print(sdPacket); Serial.println(" bytes were written.");
          }
          Serial.print("Final SD write: "); Serial.print(bufferPointer);
          Serial.print(" bytes. Total bytes written: "); Serial.println(bytesWritten);
#endif
          // Reset bufferPointer
          bufferPointer = 0;
        }

        // Set the log file's last write/modification date and time
        if (!file.timestamp(T_WRITE, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file write timestamp!");
        }
        // Set log file's last access date and time
        if (!file.timestamp(T_ACCESS, rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds())) {
          Serial.println("Warning: Unable to set file access timestamp!");
        }

        // Close the file
        file.close();

#if DEBUG
        Serial.print("File closed!");
        uint32_t filesize = file.fileSize(); // Get the file size
        Serial.print(" File size: "); Serial.print(filesize);
        Serial.print(". Expected file size: "); Serial.println(bytesWritten);
#endif
        // Loop to restart UBX messages before opening a new file
        loopStep = START_UBX;
      } // case RESTART_FILE
      break;

    case SLEEP: {

#if DEBUG
        Serial.println("Case: SLEEP");
#endif

        // Pet the dog
        resetWatchdog();

        // Go to deep sleep
        LowPower.deepSleep();

        // Check if RTC alarm was triggered
        if (alarmFlag == true) {
          loopStep = WAKE;
        }
        else {
          loopStep = SLEEP;
        }

        // Blink LED to indicate Watchdog Timer trigger
        blinkLed(1, 10);

      } // case SLEEP
      break;

    case WAKE: {

#if DEBUG
        Serial.println("Case: WAKE");
#endif

        // Check battery voltage
        if (voltage > lowVoltage) {
          loopStep = INIT;
        }
        else {
          // Set alarm
          loopStep = SLEEP;
        }
      } // case WAKE
      break;
  }
}

// RTC alarm interrupt
void alarmMatch() {
  alarmFlag = true; // Set alarm flag
}

// Print the RTC's current time and date
void printDateTime() {
  char dateTimeBuffer[25];
  snprintf(dateTimeBuffer, sizeof(dateTimeBuffer), "%04u-%02d-%02d %02d:%02d:%02d",
           (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(),
           rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  Serial.println(dateTimeBuffer);
}

// Print the RTC's current time and date
void printAlarm() {
  char alarmBuffer[25];
  snprintf(alarmBuffer, sizeof(alarmBuffer), "%04u-%02d-%02d %02d:%02d:%02d",
           (rtc.getAlarmYear() + 2000), rtc.getAlarmMonth(), rtc.getAlarmDay(),
           rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  Serial.println(alarmBuffer);
}

// Blink LED
void blinkLed(byte flashes, unsigned long interval) {

  // Pet the dog
  resetWatchdog();

  byte i = 0;
  while (i < flashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
      }
#if LED_ON
      digitalWrite(LED_BUILTIN, ledState);
#endif
      i++;
    }
  }
}

// Configure the WDT to perform a system reset if loop() blocks for more than 8-16 seconds
void configureWatchdog() {

  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |          // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);            // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |        // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K | // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);          // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |     // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;         // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                 // Set the Early Warning Interrupt Time Offset to 8 seconds // REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                       // Enable the Early Warning interrupt                       // REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                      // Set the WDT reset timeout to 16 seconds                  // REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                       // Enable the WDT in normal mode                            // REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);               // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);  // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// Pet the Watchdog Timer
void resetWatchdog() {
  WDT->CLEAR.bit.CLEAR = 0xA5;        // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);   // Await synchronization of registers between clock domains
  watchdogCounter = 0;
}

// Disable the Watchdog Timer
void disableWatchdog() {
  WDT->CTRL.bit.ENABLE = 0;         // Disable Watchdog
  while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine
void WDT_Handler() {
  // Permit a limited number of Watchdog Timer interrupts before resetting the system
  if (watchdogCounter < 10) {
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
#if DEBUG
    WDT->CTRL.bit.ENABLE = 0;         // Disable Watchdog
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on LEDs to indicate Watchdog Timer reset has triggered
    digitalWrite(LED_PIN, HIGH);
#endif
    while (1); // Force Watchdog Timer to reset the system
  }

  // Increment the number of watchdog interrupts
  watchdogCounter++;
#if DEBUG
  Serial.print("watchdogCounter: "); Serial.println(watchdogCounter);
#endif
}
