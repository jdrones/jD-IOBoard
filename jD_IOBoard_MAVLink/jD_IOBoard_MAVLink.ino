/*

 Only works in UP-Work

 Copyright (c) 2012.  All rights reserved.
 An Open Source Arduino based jD_IOBoard driver for MAVLink
 
 Program  : jD_IOBoard
 Version  : V0.4a-FrSky, July 26 2013
 Author(s): Jani Hirvinen
 Coauthor(s):
   Sandro Beningo  (MAVLink routines)
   Mike Smith      (BetterStream and Fast Serial libraries)
   Up tst
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
*/
 
/*
//////////////////////////////////////////////////////////////////////////
//  Description: 
// 
//  This is an Arduino sketch on how to use jD-IOBoard LED Driver board
//  that listens MAVLink commands and changes patterns accordingly.
//
//  If you use, redistribute this please mention original source.
//
//  jD-IOBoard pinouts v1.0
//
//             S M M G       R T R
//         5 5 C O I N D D D X X S
//         V V K S S D 7 6 5 1 1 T
//         | | | | | | | | | | | |
//      +----------------------------+
//      |O O O O O O O O O O O O O   |
// O1 - |O O   | | |                O| _ DTS 
// O2 - |O O   3 2 1                O| - RX  F
// O3 - |O O   1 1 1                O| - TX  T
// O4 - |O O   D D D                O| - 5V  D
// O5 - |O O                        O| _ CTS I
// O6 - |O O O O O O O O   O O O O  O| - GND
//      +----------------------------+
//       |   | | | | | |   | | | |
//       C   G 5 A A A A   S S 5 G
//       O   N V 0 1 2 3   D C V N
//       M   D             A L   D
//
//  jD-IOBoard pinouts v1.1
//
//       1 G   S M M G       R T R
//       2 N 5 C O I N D D D X X S
//       V D V K S S D 6 5 2 1 1 T
//       | | | | | | | | | | | | |
//      +----------------------------+
//      |O O O O O O O O O O O O O   |
// O1 - |O O   | | |                O| _ DTS 
// O2 - |O O   3 2 1                O| - RX  F
// O3 - |O O   1 1 1                O| - TX  T
// O4 - |O O   D D D                O| - 5V  D
// O5 - |O O                        O| _ CTS I
// O6 - |O O O O O O O O   O O O O  O| - GND
//      +----------------------------+
//       |   | | | | | |   | | | |
//       C   G 5 A A A A   S S 5 G
//       O   N V 0 1 2 3   D C V N
//       M   D             A L   D

//
// More information, check http://www.jdrones.com/jDoc
//
/* **************************************************************************** */
 
/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

#define MAVLINK10     // Are we listening MAVLink 1.0 or 0.9   (0.9 is obsolete now)
#define HEARTBEAT     // HeartBeat signal
#define SERDB         // Output debug information to SoftwareSerial 
#define FRSER          // FrSky serial output, cannot be run same time with SERDB
//#define ONOFFSW       // Do we have OnOff switch connected in pins 
//#define membug
//#define HWSWITCH    // Hardware factory reset option

/* **********************************************/
/* ***************** INCLUDES *******************/

#define membug   // undefine for real firmware
//#define FORCEINIT  // You should never use this unless you know what you are doing 

#define hiWord(w) ((w) >> 8)
#define loWord(w) ((w) & 0xff)

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
//#include <Wire.h>

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <GCS_MAVLink.h>


#ifdef membug
#include <MemoryFree.h>
#endif

#include <SoftwareSerial.h>

// Configurations
#include "IOBoard.h"
#include "IOEEPROM.h"

#define CHKVER 40
//#define DUMPEEPROM            // Should not be activated in repository code, only for debug
//#define DUMPEEPROMTELEMETRY   // Should not be activated in repository code, only for debug
#define NEWPAT
#define HWRESET

/* *************************************************/
/* ***************** DEFINITIONS *******************/

#define VER "v0.4"

// These are not in real use, just for reference
//#define O1 8      // High power Output 1
//#define O2 9      // High power Output 2, PWM
//#define O3 10     // High power Output 3, PWM
//#define O4 4      // High power Output 4 
//#define O5 3      // High power Output 5, PWM
//#define O6 2      // High power Output 6

#define Circle_Dly 1000

//#define ledPin 13     // Heartbeat LED if any
#define LOOPTIME  50  // Main loop time for heartbeat
//#define BAUD 57600    // Serial speed

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port


/* Patterns and other variables */
static byte LeRiPatt = NOMAVLINK; // default pattern is full ON

static long p_preMillis;
static long p_curMillis;
static int p_delMillis = LOOPTIME;

static int curPwm;
static int prePwm;

int messageCounter;
static bool mavlink_active;
byte hbStatus;

byte voltAlarm;  // Alarm holder for internal voltage alarms, trigger 4 volts

float boardVoltage;
int i2cErrorCount;

byte ledState;
byte baseState;  // Bit mask for different basic output LEDs like so called Left/Right 

//byte debug = 1;  // Shoud not be activated on repository code, only for debug
byte deb2 = 1;

byte ANA;
byte bVER = 10;

// Objects and Serial definitions
FastSerialPort0(Serial);

SimpleTimer  mavlinkTimer;

#ifdef FRSER
SoftwareSerial frSerial(6,5,true);
#endif

#ifdef SERDB
SoftwareSerial dbSerial(12,11);        // For debug porposes we are using pins 11, 12
#define DPL if(debug) dbSerial.println
#define DPN if(debug) dbSerial.print
byte debug = 1;
#else
#define DPL if(debug) {}
#define DPN if(debug) {}
byte debug = 0;
#endif




/* **********************************************/
/* ***************** SETUP() *******************/

void setup() 
{
  
  // Before all, set debug level if we have any. Comment out if not
debug = 4;  
  
  
  // Initialize Serial port, speed
  Serial.begin(TELEMETRY_SPEED);

#ifdef SERDB
  // Our software serial is connected on pins D6 and D5
  dbSerial.begin(38400);                    // We don't want to too fast
  DPL("Debug port Open");
  DPL("No input from this serialport.  ");
  if(analogRead(A6) == 1023) DPL("Running IOBoard v1.1");
#endif
  if(analogRead(A6) == 1023) bVER=11;
  
//#ifdef FRSER
  frSerial.begin(9600);
//#endif

#ifdef HWRESET
  digitalWrite(13, HIGH);      // Let's put PULLUP high in pin 13 to avoid accidental erases
  if(digitalRead(13) == 0) {    
    DPL("Force erase pin LOW, Eracing EEPROM");
    DPN("Writing EEPROM...");
    writeFactorySettings();
    DPL(" done.");
  }
#endif    
  
  // Check that EEPROM has initial settings, if not write them
  if(readEEPROM(CHK1) + readEEPROM(CHK2) != CHKVER) {
#ifdef DUMPEEPROMTELEMETRY
    Serial.print(CHK1);
    Serial.print(",");
    Serial.print(CHK2);
    Serial.print(",");
    Serial.println(CHKVER);
#endif    
    // Write factory settings on EEPROM
    DPN("Writing EEPROM...");
    writeFactorySettings();
    DPL(" done.");
  }
 
#ifdef DUMPEEPROM
  // For debug needs, should never be activated on real-life
  for(int edump = 0; edump <= 24; edump ++) {
   DPN("EEPROM SLOT: ");
   DPN(edump);
   DPN(" VALUE: ");
   DPL(readEEPROM(edump));     
  }

  // For debug needs, should never be activated on real-life
  for(int edump = 60; edump <= 80; edump ++) {
   DPN("EEPROM SLOT: ");
   DPN(edump);
   DPN(" VALUE: ");
   DPL(readEEPROM(edump));     
  }
#endif

#ifdef DUMPEEPROMTELEMETRY
  // For debug needs, should never be activated on real-life
  for(int edump = 0; edump <= 140; edump ++) {
   Serial.print("EEPROM SLOT: ");
   Serial.print(edump);
   Serial.print(" VALUE: ");
   Serial.println(readEEPROM(edump), DEC);     
  }
#endif
    
  // Rear most important values from EEPROM to their variables  
  LEFT = readEEPROM(LEFT_IO_ADDR);
  RIGHT = readEEPROM(RIGHT_IO_ADDR);
  FRONT = readEEPROM(FRONT_IO_ADDR);
  REAR = readEEPROM(REAR_IO_ADDR);
  ledPin = readEEPROM(LEDPIN_IO_ADDR);
    
  // setup mavlink port
  mavlink_comm_0_port = &Serial;

  // Initializing output pins
  for(int looper = 0; looper <= 5; looper++) {
    pinMode(Out[looper],OUTPUT);
  }

  // Initial 
  for(int loopy = 0; loopy <= 5; loopy++) {
   SlowRoll(25); 
  }

  for(int loopy = 0; loopy <= 2 ; loopy++) {
    AllOn();
    delay(100);
    AllOff();
    delay(100);
  }

  // Activate Left/Right lights
  updateBase();

  // Jani's debug stuff  
#ifdef membug
  Serial.print("Freemem: ");
  Serial.println(freeMem());
  DPL(freeMem());
#endif

  // Startup MAVLink timers, 50ms runs
  // this affects pattern speeds too.
  mavlinkTimer.Set(&OnMavlinkTimer, 50);

  // House cleaning, enable timers
  mavlinkTimer.Enable();
  
  // Enable MAV rate request, yes always enable it for in case.   
  // if MAVLink flows correctly, this flag will be changed to DIS
  enable_mav_request = EN;  
  
  // for now we are always active, maybe in future there will be some
  // additional features like light conditions that changes it.
  isActive = EN;  
  DPL("End of setup");
    
} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

// The thing that goes around and around and around for ethernity...
// MainLoop()
void loop() 
{

  
#ifdef HEARTBEAT
  HeartBeat();   // Update heartbeat LED on pin = ledPin (usually D13)
#endif

  if(isActive) { // main loop
    p_curMillis = millis();
    if(p_curMillis - p_preMillis > p_delMillis) {
      // save the last time you blinked the LED 
      p_preMillis = p_curMillis;   

      // First we update pattern positions 
      patt_pos++;
      if(patt_pos == 16) {
        patt_pos = 0;
        if(debug == 4) dumpVars(); 
      }
    }

    // Update base lights if any
    updateBase();
  
    if(enable_mav_request == 1) { //Request rate control. 
     // DPL("IN ENA REQ");
      // During rate requsst, LEFT/RIGHT outputs are HIGH
      digitalWrite(LEFT, EN);
      digitalWrite(RIGHT, EN);

      for(int n = 0; n < 3; n++) {
        request_mavlink_rates();   //Three times to certify it will be readed
        delay(50);
      }
      enable_mav_request = 0;

      // 2 second delay, during delay we still update PWM output
      for(int loopy = 0; loopy <= 2000; loopy++) {
        delay(1);
        updatePWM();
      }
      waitingMAVBeats = 0;
      lastMAVBeat = millis();    // Preventing error from delay sensing
      //DPL("OUT ENA REQ");
    }  
    
    // Request rates again on every 10th check if mavlink is still dead.
//    if(!mavlink_active && messageCounter == 10) {
    if(!mavlink_active) {
      DPL("Enabling requests again");
      enable_mav_request = 1;
//      messageCounter = 0;
      LeRiPatt = 6;
    } 

    read_mavlink();
    mavlinkTimer.Run();

    updatePWM(); 
    update_FrSky();

  } //else AllOff();

}

/* *********************************************** */
/* ******** functions used in main loop() ******** */

// Function that is called every 120ms
void OnMavlinkTimer()
{
  if(millis() < (lastMAVBeat + 3000)) {
    // General condition checks starts from here
    //
      
    // Checks that we handle only if MAVLink is active
    if(mavlink_active) {
      if(iob_fix_type <= 2) LeRiPatt = ALLOK;
//      if(iob_fix_type <= 2) LeRiPatt = NOLOCK;
      if(iob_fix_type >= 3) LeRiPatt = ALLOK;
  
      // CPU board voltage alarm  
      if(voltAlarm) {
        LeRiPatt = LOWVOLTAGE;  
        DPL("ALARM, low voltage");
      }     
    }
        
    // If we are armed, run patterns on read output
    if(isArmed) RunPattern();
     else ClearPattern();
    
    // Update base LEDs  
    updateBase();

/*    DPN("MC:"); 
    DPL(messageCounter);
    if(messageCounter >= 20 && mavlink_active) {
      DPL("We lost MAVLink");
      mavlink_active = 0;
      messageCounter = 0;
      LeRiPatt = NOMAVLINK;
    }
  //  DPL(messageCounter);
*/ 
  // End of OnMavlinkTimer
  } else {
//    DPN("Beats:");
//   DPL(lastMAVBeat);
    waitingMAVBeats = 1;
    LeRiPatt = NOMAVLINK;
  }
}


void dumpVars() {
#ifdef SERDB  
 DPN("Sats:");
 DPN(iob_satellites_visible);
 DPN(" Fix:");
 DPN(iob_fix_type);
 DPN(" Modes:");
 DPN(iob_mode);
 DPN(" Armed:");
 DPN(isArmed);
 DPN(" Thr:");
 DPN(iob_throttle);
 DPN(" CPUVolt:");
 DPN(boardVoltage);
 DPN(" BatVolt:");
 DPN(iob_vbat_A);
 DPN(" Alt:");
 DPN(iob_alt);
 DPN(" Hdg:");
 DPN(iob_heading);
 DPN(" Spd:");
 DPN(iob_groundspeed);
 DPN(" Lat:");
 DPN(iob_lat);
 DPN(" Lon");
 DPN(iob_lon);
 DPL(" "); 
#endif
}
