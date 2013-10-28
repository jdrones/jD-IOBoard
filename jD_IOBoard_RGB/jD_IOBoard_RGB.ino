/*
 Copyright (c) 2012.  All rights reserved.
 An Open Source Arduino based jD_IOBoard 
 
 Program  : jD_IOBoard_RGB Demo
 Version  : V1.0, June 06 2012
 Author(s): Jani Hirvinen
 
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

///////////////////////////
// Global includes


///////////////////////////
// Global defines 

#define VER "v1.0"

#define EN 1
#define DI 0

#define O1 8      // High power Output 1
#define O2 9      // High power Output 2, PWM
#define O3 10     // High power Output 3, PWM
#define O4 4      // High power Output 4 
#define O5 3      // High power Output 5, PWM
#define O6 2      // High power Output 6

int Out[] = {
  8,9,10,4,3,2};   // Output I/O pin array

byte R = 0;
byte G = 0;
byte B = 0;

#define dly 1
#define MAX 253
#define MIN 1

byte debug = EN;   // or DI

void setup() { 
  // Initialize our Serial port and speed
  Serial.begin(57600); 

  // Initializing output pins
  for(int looper = 0; looper <= 5; looper++) {
    pinMode(Out[looper],OUTPUT);
  } 

}


void loop() { 
  
  // R from 0 to 255
  // G = 0
  // B = 0
  if(debug) Serial.println("## 1 ");  // Pattern 1
  for(R == MIN; R <= MAX; R++) {
    rgb();
    delay(dly);
  }

  // R = 255
  // G = 0
  // B from 0 to 255
  if(debug) Serial.println("## 2 ");  // Pattern 2
  for(B == MIN; B <= MAX; B++) {
    rgb();
    delay(dly);
  }

  // R from 255 to 0
  // G = 0
  // B = 255
  if(debug) Serial.println("## 3 ");  // Pattern 3
  for(R == MAX; R >= MIN; R--) {
    rgb();
    delay(dly);
  }

  // R = 0 
  // G from 0 to 255
  // B = 255
  if(debug) Serial.println("## 4 ");  // Pattern 4
  for(G = MIN; G <= MAX; G++) {
    rgb();
    delay(dly);
  }

  // R = 0
  // G = 255
  // B from 255 to 0
  if(debug) Serial.println("## 5 ");  // Pattern 5
  for(B = MAX; B >= MIN; B--) {
    rgb();
    delay(dly);
  }       

  // R from 0 to 255
  // G = 255
  // B = 0
  if(debug) Serial.println("## 6 ");  // Pattern 6
  for(R = MIN; R <= MAX; R++) {
    rgb();
    delay(dly);
  }       

  // R = 255
  // G from 255 to 0
  // B = 0
  if(debug) Serial.println("## 7 ");  // Pattern 7
  for(G = MAX; G >= MIN; G--) {
    rgb();
    delay(dly);
  }       

} // end of main loop


// Subroutine RGB()
// Update PWM output
void rgb() {
  // Print debug values to Serial if debug = ENABLE
  if(debug) {
    Serial.print(R);
    Serial.print(" ");
    Serial.print(G);
    Serial.print(" ");
    Serial.println(B);
  }     
  
  // Update PWM output to all 3 R.G.B outputs
  analogWrite(O2, R);
  analogWrite(O3, G);
  analogWrite(O5, B);

} // end of rgb()

