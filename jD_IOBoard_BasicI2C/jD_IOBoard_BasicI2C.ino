/**********************************************************/
//
// Name    : jD-IOBoard
// Version : v1.0 20-02-12
// Author  : Jani Hirvinen, jani@j....com
// 
//
//  Copyright (c) 2012 Jani Hirvinen.  All rights reserved.
//  An Open Source Arduino LED Driver.
//
//  This program is free software: you can redistribute it and/or modify 
//  it under the terms of the GNU General Public License as published by 
//  the Free Software Foundation, either version 3 of the License, or 
//  (at your option) any later version. 
//
//  This program is distributed in the hope that it will be useful, 
//  but WITHOUT ANY WARRANTY; without even the implied warranty of 
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
//  GNU General Public License for more details. 
//
//  You should have received a copy of the GNU General Public License 
//  along with this program. If not, see <http://www.gnu.org/licenses/>. 
// 
// Changelog:
// 20-02-12  Initial file
//
//
//////////////////////////////////////////////////////////////////////////
//  Description: 
// 
//  This is example Arduino sketch on how to use jD-IOBoard.
//  LED Driver board that listens I2C commands and changes patterns accordingly.
//
//  If you use, redistribute this please mention original source.
//
//  jD-IOBoard pinouts
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
//
// Receive byte messages
// 0xff  = All on
// 0xfe  = All off
// 0xfd  = RI/LE/RE on
// 0xfc  = RI/LE on 
// 0xfb  = Flashes all 4 outputs in circle 5 rounds
// 0-9   = Front pattern
//
// Board listens i2c address of: 0x42
//
/******************************************************************************/


///////////////////////////
// Global includes
//#include "I2C.h"

#include <Wire.h>


///////////////////////////
// Global defines 

#define VER "v1.0"

#define addr 0x42

#define O1 8      // High power Output 1
#define O2 9      // High power Output 2
#define O3 10     // High power Output 3
#define O4 4      // High power Output 4 
#define O5 3      // High power Output 5
#define O6 2      // High power Output 6


#define LE  O1    // Left Arm
#define RI  O2    // Right Arm
#define FR  O3    // Front Arm
#define RE  O4    // Read Arm

#define EN  1     // Enable value
#define DI  0     // Disable value

#define Circle_Dly 1000

#define ledPin 13     // Heartbeat LED if any
#define LOOPTIME  50  // Main loop time for heartbeat
#define BAUD 57600    // Serial speed

///////////////////////////
// Global variables
int counter = 0;
int Out[] = { 8,9,10,4,3,2};
int i,x;

byte Stat_LE; 
byte Stat_RI;
byte Stat_FR;
byte Stat_RE;


// Flight LED patterns
byte flight_patt[10][16] = {
  { 0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0  },    // 0
  { 1,1,1,1,0,0,0,0 ,1,1,1,1,0,0,0,0  },    // 1
  { 1,1,1,1,1,0,0,0 ,0,0,0,0,0,1,0,0  },    // 2
  { 1,1,0,0,1,1,0,0 ,1,1,0,0,1,1,0,0  },    // 3
  { 1,0,0,0,1,0,0,0 ,1,0,0,0,1,0,0,0  },    // 4
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,0,0,0,0  },    // 5
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,1,0,1,0  },    // 6
  { 1,0,1,0,0,0,0,0 ,1,0,1,0,0,0,0,0  },    // 7
  { 0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0  },    // 8
  { 1,0,0,0,0,0,0,0 ,1,0,0,0,0,0,0,0  }};   // 9

byte patt_pos;
byte patt;

long mainLoop;
byte ledState = LOW;
byte mainStatus;

int debig = 1;      // debig level

///////////////////////////
// Setup 
void setup() {

  Wire.begin(addr);
  Wire.onReceive(RxEvent); 
  Serial.begin(BAUD); 

  pinMode(LE, OUTPUT);
  pinMode(RI, OUTPUT);
  pinMode(FR, OUTPUT);
  pinMode(RE, OUTPUT);

  digitalWrite(LE, DI);
  digitalWrite(RI, DI);
  digitalWrite(FR, DI);
  digitalWrite(RE, DI);

  //  delay(1000);
  patt_pos = 0;
  patt = 0;

}


///////////////////////////
// Main program
void loop() {
  //  Serial.
  if (millis() - mainLoop > LOOPTIME) {
    // save the last time you blinked the LED 
    mainLoop = millis();   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;      
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);

    if(mainStatus == 1) {
      digitalWrite(RE, flight_patt[patt][patt_pos]);
      Serial.print(patt, DEC);
      Serial.print("\t");
      Serial.print(patt_pos, DEC);
      Serial.print("\t");      
      Serial.println(flight_patt[patt][patt_pos],BIN);
      patt_pos++;
      if(patt_pos == 16) patt_pos = 0;
    }
  }
}



void RxEvent(int howMany) {
  x = Wire.receive();
  if(x == 0xff) {
    digitalWrite(LE, EN);
    digitalWrite(RI, EN);
    digitalWrite(FR, EN);
    digitalWrite(RE, EN);
    mainStatus = x;
  }
  if(x == 0xfe) {
    digitalWrite(LE, DI);
    digitalWrite(RI, DI);
    digitalWrite(FR, DI);
    digitalWrite(RE, DI);
    mainStatus = 0;
  }
  if(x == 0xfd) {
    digitalWrite(LE, EN);
    digitalWrite(RI, EN);
    digitalWrite(FR, EN);
    digitalWrite(RE, DI);
    mainStatus = 1;
  }
  if(x == 0xfc) {
    digitalWrite(LE, EN);
    digitalWrite(RI, EN);
    digitalWrite(FR, DI);
    digitalWrite(RE, DI);
    mainStatus = x;
  }
  if(x == 0xfb) {
    Flash_Circle();
    //    mainStatus = x;
  }
  if(x >= 0 && x <= 10) patt = x;
  Serial.print("Received command: ");  
  Serial.println(x, DEC);  
}




///////////////////////////
// Global functions

void Flash_Circle() {
  int count;
  Status_Save();
  //ChangeAll(0);

  for(count = 0; count <= 4; count++) {
    digitalWrite(LE, EN);
    digitalWrite(RI, DI);
    digitalWrite(FR, DI);
    digitalWrite(RE, DI);
    delay(Circle_Dly);
    digitalWrite(LE, DI);
    digitalWrite(RI, EN);
    digitalWrite(FR, DI);
    digitalWrite(RE, DI);
    delay(Circle_Dly);
    digitalWrite(LE, DI);
    digitalWrite(RI, DI);
    digitalWrite(FR, EN);
    digitalWrite(RE, DI);
    delay(Circle_Dly);
    digitalWrite(LE, DI);
    digitalWrite(RI, DI);
    digitalWrite(FR, DI);
    digitalWrite(RE, EN);
    delay(Circle_Dly);
  }
  Status_Recall();
}

void Status_Save() {
  Stat_LE = digitalRead(LE);
  Stat_RI = digitalRead(RI);
  Stat_FR = digitalRead(FR);
  Stat_RE = digitalRead(RE);
}

void Status_Recall() {
  digitalWrite(LE, Stat_LE); 
  digitalWrite(RI, Stat_RI); 
  digitalWrite(FR, Stat_FR); 
  digitalWrite(RE, Stat_RE); 
}

void ChangeAll(boolean Status) {
  if(!Status) {
    digitalWrite(LE, DI);
    digitalWrite(RI, DI);
    digitalWrite(FR, DI);
    digitalWrite(RE, DI);
  } else { 
    digitalWrite(LE, EN);
    digitalWrite(RI, EN);
    digitalWrite(FR, EN);
    digitalWrite(RE, EN);
  }
}

