///////////////////////////////////////////////////////////////////////////////////////
//
// Name         : jD-IOBoard_RGB Demo
// Version      : v1.0 06-06-2012
// Author       : Jani Hirvinen, jani@j....com
// Co-Author(s) :
// 
//
// Copyright (c) 2013, Jani Hirvinen, jDrones & Co.
// All rights reserved.
//
// - Redistribution and use in source and binary forms, with or without 
//   modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this 
//  list of conditions and the following disclaimer.
//
// - Redistributions in binary form must reproduce the above copyright notice, 
//  this list of conditions and the following disclaimer in the documentation 
//  and/or other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
//  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
//  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
//  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
//  POSSIBILITY OF SUCH DAMAGE.
//
/////////////////////////////////////////////////////////////////////////////////////////

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

