/**********************************************************/
//
// Name         : jD_IOBoard_PWMReader.ino
// Version      : v1.0 02.07.2013
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
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  This sketch works only in jD-IOBoard V1.1 as they need D2 pin for proper Interrupt
//  Get your jD-IOBoard from http://store.jDrones.com
//
//  Connect D2 pin to your RC receiver Signal pin.
//  Connect GND to your RC Receiver GND pins
//  Connect +5V to your RC Receiver +5V pins
//
//  For +5V/GND pins you can use for example pins in I2C connector area next to FTDI connector.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  

#define INT0 0              // We are using INT.0 to capture  
#define SIGNAL_PIN 2        // Singal pin for incoming PWM is D2
#define RC_NEUTRAL 1500     // PWM pulse width for center stick
#define OUTPIN 7            // Output pin for relay, shutter etc output

volatile boolean       New_PWM_Frame = false; // Flag marker for new and changed PWM value
volatile int           PWM_IN;                // Value to hold PWM signal width. Exact value of it. Normally between 1000 - 2000ms while 1500 is center
volatile unsigned long int_Timer = 0;         // set in the INT0

void setup()
{
  attachInterrupt(INT0, ReadINT_PIN, CHANGE);  // Attach Reading function to INTERRUPT 

  pinMode(OUTPIN, OUTPUT);    // Make sure that our output pin is defined as OUTPUT
  Serial.begin(115200);       // FTDI serial speed for terminal connection

  // Check version numbering on jD-IOBoard. V1.0 has A6 floating, V1.1 has it connected to +5V
  if(!analogRead(A6) >= 1023) {
    delay(2000);
    Serial.println("You are running jD-IOBoard V1.0 and this board needs at least jD-IOBoard V1.1 to operate!!");
    for(;;);
  }  
}

void loop()
{
  // If we have new frame data, show it and do_something()...
  if(New_PWM_Frame) {

    // Print out current PWM Signal width in ms
    Serial.print(PWM_IN); 
    Serial.print(" ms which is: ");

    // If PWM is less than RC_NEUTRAL (1500) then show are we below or above
    // This is area where you can make your own tests for different PWM value
    // In this example we just put D7 output High and Low depending are we above/below 1500ms

    if(PWM_IN < RC_NEUTRAL) {
      Serial.print(" below 1500 ms");
      digitalWrite(OUTPIN, LOW);        // Put OUTPUT pin LOW
    } else {
      Serial.print(" above 1500 ms");
      digitalWrite(OUTPIN, HIGH);       // Put OUTPUT pin HIGH
    }

    Serial.println();   // Output linefeed

    // Set Frame flag to False
    New_PWM_Frame = false;
    
    // Wait a sec, mainly for Serial.print issues, if you don't use Serial.print to show your
    // data. You can remove this line for final application
    delay(50); 
   }
    
} // loop


// PWM Measurement
void ReadINT_PIN() {
  
  // We will start to read when signal goes HIGH
  if(digitalRead(SIGNAL_PIN) == HIGH) {

    // PWM Signal is HIGH, so measure it's length 
    int_Timer = micros();

  } else {

    // If PWM signal is getting LOW and timer is running, it must be falling edge and then we stop timer
    if(int_Timer && (New_PWM_Frame == false))
    {
      PWM_IN = (int)(micros() - int_Timer);
      int_Timer = 0;

      New_PWM_Frame = true;
    }
  }
}


