/*
///////////////////////////////////////////////////////////////////////
//
// Please read licensing, redistribution, modifying, authors and 
// version numbering from main sketch file. This file contains only
// a minimal header.
//
// Copyright (c) 2013, Jani Hirvinen, jDrones & Co.
// All rights reserved.
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
/////////////////////////////////////////////////////////////////////////////

 Relies on: 
 - jD IOBoard
 - SoftwareSerial
 - FrSky modules rx/tx
 - jD IOBoard, MAVLink code
 
 Connection:
 Connect following cables/pins from IOBoard to FrSky D8R-II or similar telemetry receiver
 
 IOB   RX
 -----------
 GND - GND
 D5  - Rx
 
 Details:
 Program creates and populates FrSky HUB style protocol messages and feeds it out from SoftwareSerial pins on IOBoard.
 SoftwareSerial uses inverted signaling to output data correctly, if signal is non-inverted data will be corrupt due 
 XORing and shifthing one step to right process. 
 
 FrSky uses 3 frames on their HUB protocol
 
 Frame 1, every 200ms,  payload: accel-x, accel-y, accel-z, Altitude(Vario), Temp1, Temp2, Voltage (multiple), RPM
 Frame 2, every 1000ms, payload: course, lat, lon, speed, altitude (GPS), fuel level
 Frame 3, every 5000ms, payload: date, time
 
 */

////////////////////////////////////////////////////////////
// update_FrSky() constructing FrSky data packets
//
void update_FrSky() {
//DPL("fr");
  f_curMillis = millis();
  if(f_curMillis - f_preMillis > f_delMillis) {
    // save the last time you sent the messaga 
    f_preMillis = f_curMillis;   

    // 200ms payload, construct Frame 1 on every loop
    packetOpen = TRUE;
    payloadLen += addPayload(0x24); // accel-x
    payloadLen += addPayload(0x25); // accel-y
    payloadLen += addPayload(0x26); // accel-z

    payloadLen += addPayload(0x10); // alt(vario). before "."
    payloadLen += addPayload(0x21); // alt, after "."
    
    payloadLen += addPayload(0x02); // Temperature 1
    payloadLen += addPayload(0x05); // Temperature 2
    
    payloadLen += addPayload(0x06); // Battery data, 
    payloadLen += addPayload(0x28); // Ampere
    
    payloadLen += addPayload(0x3A); // Voltage , before "."
    payloadLen += addPayload(0x3B); // Voltage , after "."

    payloadLen += addPayload(0x03); // rpm
     
    packetOpen = FALSE;
    payloadLen = sendPayload(payloadLen);


    // 1000ms (1s) payload, construct Frame 2 on every 5th loop
    if((msCounter % 5) == 0) {
      second++;
      updateTime();
      packetOpen = TRUE;
      payloadLen += addPayload(0x14);   // Course, degree
      payloadLen += addPayload(0x1c);   // Course, after "."
    
      payloadLen += addPayload(0x13);   // Longitude dddmmm 
      payloadLen += addPayload(0x1b);   // Longitude .mmmm (after ".")
      payloadLen += addPayload(0x23);   // E/W

      payloadLen += addPayload(0x12);   // Latitude dddmmm
      payloadLen += addPayload(0x1a);   // Latitude .mmmm (after ".")
      payloadLen += addPayload(0x22);   // N/S
    
      payloadLen += addPayload(0x11);   // GPS Speed Knots
      payloadLen += addPayload(0x19);   // GPS Speed after "."

      payloadLen += addPayload(0x01);   // GPS Altitude
      payloadLen += addPayload(0x09);   // GPS Altitude "."
    
      payloadLen += addPayload(0x04);   // Fuel level % 0,25,50,75,100
      
      payloadLen += addPayload(0x18);   // secs
       

      packetOpen = FALSE;
      payloadLen = sendPayload(payloadLen);

      
    }  


    // 5000ms (5s) payload, contruct Frame 3 on every 25th loop and reset counters
    if(msCounter >= 25) {
      packetOpen = TRUE;
      payloadLen += addPayload(0x15); // date/month      
      payloadLen += addPayload(0x16); // year      
      payloadLen += addPayload(0x17); // hour/min      
      payloadLen += addPayload(0x18); // secs     
      packetOpen = FALSE;
      payloadLen = sendPayload(payloadLen);
      msCounter = 0;
    }
    // Update loop counter
    msCounter ++;
  }
}

////////////////////////////////////////////////////////////
// addPayload() FrSky datapacket payloads
//
byte addPayload(byte DataID) {
  
//  int test = 0;
  
  byte addedLen;
  switch(DataID) {
    case 0x01:  // GPS Altitude
      outBuff[payloadLen + 0] = 0x01;
      outBuff[payloadLen + 1] = FixInt(int(iob_gps_alt), 1);
      outBuff[payloadLen + 2] = FixInt(int(iob_gps_alt), 2);
      addedLen = 3;      
      break;
    case 0x01+8:  // GPS Altitude
      {
      float tmp = (iob_gps_alt - int(iob_gps_alt)) * 10000.0f;
      outBuff[payloadLen + 0] = 0x01+8;
      outBuff[payloadLen + 1] = FixInt(int(tmp), 1);
      outBuff[payloadLen + 2] = FixInt(int(tmp), 2);
      addedLen = 3;      
      }
      break;
      
    case 0x02:  // Temperature 1
      outBuff[payloadLen + 0] = 0x02;
      outBuff[payloadLen + 1] = iob_temperature;
      outBuff[payloadLen + 2] = 0x00;
      
//      ShowPayload();
      addedLen = 3;      
      break;

    // RPM. Works ok 24.07.13 jp. We are showing throttle value in RPM field from 1000 to 2000, same as pulse width
    // Output is scaled to FrSky displayed output which is (x * 30)  Example: 34 = 30 = 1020
    // Works as ARMED/DISARMED indicator as if DISARMED RPM value is 0
    case 0x03:  
      outBuff[payloadLen + 0] = 0x03;
      /*
      if(isArmed) {
        outBuff[payloadLen + 1] = map(iob_throttle, 0, 100, 34, 66);
        outBuff[payloadLen + 2] = 0x00;
      } else {
        outBuff[payloadLen + 1] = 0x00;
        outBuff[payloadLen + 2] = 0x00;
      }  */

      if(isArmed) {
        outBuff[payloadLen + 1] = 0x66;
        outBuff[payloadLen + 2] = 0x00;
      } else {
        outBuff[payloadLen + 1] = 0x34;
        outBuff[payloadLen + 2] = 0x00;
      }
      addedLen = 3;      
      break;

    case 0x04:  // Fuel Level
      outBuff[payloadLen + 0] = 0x04;
      outBuff[payloadLen + 1] = FixInt(iob_battery_remaining_A, 1);
      outBuff[payloadLen + 2] = FixInt(iob_battery_remaining_A, 2);
      addedLen = 3;      
      break;

    // Temperature 2
    // We are using Temperature 2 to show Visible GPS satellites and also FIX type
    // Visible satellites is multiplied with 10 and fix type is added on final number
    // For example if we have 7 satellites and we have solid 3D fix outcome will be
    // (7 * 10) + 3 = 73   (7 satellites, 3 = 3D Fix)
    case 0x05:  
      outBuff[payloadLen + 0] = 0x05;
      outBuff[payloadLen + 1] = 10 * iob_satellites_visible + iob_fix_type;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

/*    case 0x06:  // Voltage, first 4 bits are cell number, rest 12 are voltage in 1/500v steps, scale 0-4.2v
      outBuff[payloadLen + 0] = 0x06;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0xff;
      outBuff[payloadLen + 3] = 0x06;
      outBuff[payloadLen + 4] = 0x10;
      outBuff[payloadLen + 5] = 0xff;
      outBuff[payloadLen + 6] = 0x06;
      outBuff[payloadLen + 7] = 0x20;
      outBuff[payloadLen + 8] = 0xff;
      outBuff[payloadLen + 9] = 0x06;
      outBuff[payloadLen + 10] = 0x30;
      outBuff[payloadLen + 11] = 0xff;
      addedLen = 12;      
      break;
*/
    // Little Endian exception
    case 0x06:  // Voltage, first 4 bits are cell number, rest 12 are voltage in 1/500v steps, scale 0-4.2v
 /*     if (cell_count < cell_numb) {
        int tmp1 = FixInt(cellV[cell_count], 2);
  
        outBuff[payloadLen + 0] = 0x06;
//        outBuff[payloadLen + 1] = tmp1 + (cell_count * 16);
//        outBuff[payloadLen + 2] = tmp2;

        outBuff[payloadLen + 1] = 158;
        outBuff[payloadLen + 2] = 55;
        addedLen = 3;
        
        cell_count++;
      } else {
        cell_count = 0; 
      }*/
      
      //==================Chagne Data Batt volt for Sending===========//
       if(Frsky_Count_Order_Batt < Batt_Cell_Detect)
       {
          Frsky_Batt_Volt_A=((((iob_vbat_A/Batt_Cell_Detect)*2100)/4.2));
          //DPL("Batt Cell--> ");
          //DPL(Batt_Cell_Detect);
          //DPL(Frsky_Count_Order_Batt);
         outBuff[payloadLen + 0] = 0x06;
         outBuff[payloadLen + 1] = (Frsky_Count_Order_Batt<<4)&0xF0 | ((Frsky_Batt_Volt_A>>8)&0x0F);  //(iob_vbat_A)
         outBuff[payloadLen + 2] = (Frsky_Batt_Volt_A)&0xFF;
         
         Frsky_Count_Order_Batt++;

         
         addedLen = 3;
       }
       else
       {
         Frsky_Count_Order_Batt=0;
         addedLen=0;
       }


//       if(Frsky_Count_Order_Batt < Batt_Cell_Detect)
//       {
//         Frsky_Batt_Volt_A=((((iob_vbat_A/Batt_Cell_Detect)*2100)/4.2));
//         outBuff[payloadLen + 0] = 0x06;
//         outBuff[payloadLen + 1] = (Frsky_Count_Order_Batt<<4)&0xF0 | ((Frsky_Batt_Volt_A>>8)&0x0F);  //(iob_vbat_A)
//         outBuff[payloadLen + 2] = (Frsky_Batt_Volt_A)&0xFF;
//         Frsky_Count_Order_Batt++;
//         addedLen = 3;
//       }
//       else
//       {
//         Frsky_Count_Order_Batt=0;
//         Frsky_Batt_Volt_A=((((iob_vbat_A/Batt_Cell_Detect)*2100)/4.2));
//         outBuff[payloadLen + 0] = 0x06;
//         outBuff[payloadLen + 1] = (Frsky_Count_Order_Batt<<4)&0xF0 | ((Frsky_Batt_Volt_A>>8)&0x0F);  //(iob_vbat_A)
//         outBuff[payloadLen + 2] = (Frsky_Batt_Volt_A)&0xFF;
//         Frsky_Count_Order_Batt++;
//         addedLen = 3;
//       }
       
      /* outBuff[payloadLen + 0] = 0x06;
       outBuff[payloadLen + 1] = 0x06;
       outBuff[payloadLen + 2] = 0xC3;
       addedLen = 3;*/
      //==============================================================//
      break;
      
    case 0x10:  // Altitude, before "." works on FLD-02, Taranis no
      outBuff[payloadLen + 0] = 0x10;
      outBuff[payloadLen + 1] = FixInt(iob_alt, 1);
      outBuff[payloadLen + 2] = FixInt(iob_alt, 2);
      addedLen = 3;      
      break;
    case 0x21:  // Altitude, after "."
      outBuff[payloadLen + 0] = 0x21;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

      
    case 0x11:  // GPS Speed, before "."
      outBuff[payloadLen + 0] = 0x11;
      outBuff[payloadLen + 1] = FixInt(iob_groundspeed, 1);
      outBuff[payloadLen + 2] = FixInt(iob_groundspeed, 2);
      addedLen = 3;      
      break;
    case 0x11+8:  // GPS Speed, after "."
      outBuff[payloadLen + 0] = 0x11+8;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

    //Little Endian exception
    case 0x12:  // Longitude, before "."
      outBuff[payloadLen + 0] = 0x12;
      outBuff[payloadLen + 1] = FixInt(long(iob_lon),1);
      outBuff[payloadLen + 2] = FixInt(long(iob_lon),2);
      addedLen = 3;      
      break;
    case 0x12+8:  // Longitude, after "."
      outBuff[payloadLen + 0] = 0x12+8;
      outBuff[payloadLen + 1] = FixInt(long((iob_lon - long(iob_lon)) * 10000.0), 1);  // Only allow .0000 4 digits
      outBuff[payloadLen + 2] = FixInt(long((iob_lon - long(iob_lon)) * 10000.0), 2);  // Only allow .0000 4 digits after .
      addedLen = 3;      
      break;
    case 0x1A+8:  // E/W
      outBuff[payloadLen + 0] = 0x1A+8;
      outBuff[payloadLen + 1] = iob_lon_dir;
      outBuff[payloadLen + 2] = 0;
      addedLen = 3;      
      break;

      
    //Little Endian exception
    case 0x13:  // Latitude, before "."
      outBuff[payloadLen + 0] = 0x13;
      outBuff[payloadLen + 1] = FixInt(long(iob_lat),1);
      outBuff[payloadLen + 2] = FixInt(long(iob_lat),2);
      addedLen = 3;      
      break;
    case 0x13+8:  // Latitude, after "."
      outBuff[payloadLen + 0] = 0x13+8;
      outBuff[payloadLen + 1] = FixInt(long((iob_lat - long(iob_lat)) * 10000.0), 1);
      outBuff[payloadLen + 2] = FixInt(long((iob_lat - long(iob_lat)) * 10000.0), 2);      
      addedLen = 3;      
      break;  
    case 0x1B+8:  // N/S
      outBuff[payloadLen + 0] = 0x1B+8;
      outBuff[payloadLen + 1] = iob_lat_dir;
      outBuff[payloadLen + 2] = 0;      
      addedLen = 3;      
      break;
   
    case 0x14:  // course, before ".". OK
      outBuff[payloadLen + 0] = 0x14;
      outBuff[payloadLen + 1] = FixInt(iob_heading, 1);
      outBuff[payloadLen + 2] = FixInt(iob_heading, 2);
      addedLen = 3;      
      break;
    case 0x14+8:  // course, after "."  .. check calculation
      outBuff[payloadLen + 0] = 0x14+8;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
      
    case 0x15: // date/month
      outBuff[payloadLen + 0] = 0x15;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x16: // year
      outBuff[payloadLen + 0] = 0x16;
      outBuff[payloadLen + 1] = 0x00;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;
    case 0x17: // hour/minute
      outBuff[payloadLen + 0] = 0x17;
      outBuff[payloadLen + 1] = hour, DEC;
      outBuff[payloadLen + 2] = minutes, DEC;
      addedLen = 3;      
      break;
    case 0x18: // second
      outBuff[payloadLen + 0] = 0x18;
      outBuff[payloadLen + 1] = second, DEC;
      outBuff[payloadLen + 2] = 0x00;
      addedLen = 3;      
      break;

    case 0x24:
      outBuff[payloadLen + 0] = 0x24;      
      outBuff[payloadLen + 1] = FixInt(iob_roll * 100, 1);
      outBuff[payloadLen + 2] = FixInt(iob_roll * 100, 2);
      addedLen = 3;      
      break;
    case 0x25:
      outBuff[payloadLen + 0] = 0x25;
      outBuff[payloadLen + 1] = FixInt(iob_pitch * 100, 1);
      outBuff[payloadLen + 2] = FixInt(iob_pitch * 100, 2);
      addedLen = 3;      
      break;
    case 0x26:
      outBuff[payloadLen + 0] = 0x26;
      outBuff[payloadLen + 1] = FixInt(iob_yaw * 100, 1);
      outBuff[payloadLen + 2] = FixInt(iob_yaw * 100, 2);
      addedLen = 3;      
      break;

    case 0x3A:  // Volt 
      //iob_vbat_A o boardVoltage
      outBuff[payloadLen + 0] = 0x3A;
      outBuff[payloadLen + 1] = FixInt(int(iob_vbat_A), 1);
      outBuff[payloadLen + 2] = FixInt(int(iob_vbat_A), 2);
      addedLen = 3;      
      break;
    case 0x3B:
      //iob_vbat_A o boardVoltage
      outBuff[payloadLen + 0] = 0x3B;
      outBuff[payloadLen + 1] = FixInt(int((iob_vbat_A - int(iob_vbat_A)) * 1000.0), 1);
      outBuff[payloadLen + 2] = FixInt(int((iob_vbat_A - int(iob_vbat_A)) * 1000.0), 2);
      
      addedLen = 3;      
      break;

    case 0x28:
      outBuff[payloadLen + 0] = 0x28;
      outBuff[payloadLen + 1] = FixInt(int(iob_ampbatt_A), 1);
      outBuff[payloadLen + 2] = FixInt(int(iob_ampbatt_A), 2);
      addedLen = 3;      
      break;
      
    default:
      addedLen = 0;
  }
  return addedLen; 

}

byte addEnd() {
 return 1; 
}

// Sending packets. Create frame with correct data
// Frame format:
//

byte sendPayload(byte len) {
  
  frSerial.write(0x5E);
  for(byte pos = 0; pos <= len-1 ; pos = pos + 3) {
    frSerial.write(byte(outBuff[pos + 0]));

    switch  (outBuff[pos + 1]) {
      case 0x5E:
        frSerial.write(byte(0x5D));
        frSerial.write(byte(0x3E));
        break;
      case 0x5D:
        frSerial.write(byte(0x5D));
        frSerial.write(byte(0x3D));
        break;
        
      default:
        frSerial.write(byte(outBuff[pos + 1]));
    }

    switch  (outBuff[pos + 2]) {
      case 0x5E:
        frSerial.write(byte(0x5D));
        frSerial.write(byte(0x3E));
        break;
      case 0x5D:
        frSerial.write(byte(0x5D));
        frSerial.write(byte(0x3D));
        break;        
      default:
        frSerial.write(byte(outBuff[pos + 2]));
    }
    frSerial.write(0x5E);
  }
  return 0;
}

// FrSky int handling Little Endian, Big Endian
long FixInt(long val, byte mp) {  
 if(mp == 2) return long(val / 256);
 if (val >= 256 && mp == 1) 
   return val % 256;  
}

////////////////////////////////////////////////////////////
// ShowPayload() debug function to show constructed payload
//
void ShowPayload() {
#ifdef SERDB  
   DPN("PL: ");
   DPN(outBuff[payloadLen + 1], DEC);
   DPN(", ");
   DPN(outBuff[payloadLen + 2], DEC);
   DPL(" "); 
#endif
}

////////////////////////////////////////////////////////////
// updateTime() Time counters for FrSky telemetry
//
void updateTime() {
  if(second >= 60) {
    second = 0;
    minutes++;
   }
   if(minutes >= 60) {
    second = 0;
    minutes = 0;
    hour++;
   } 
   if(hour >= 24) {
     second = 0;
     minutes = 0;
     hour = 0;
   }
}

////////////////////////////////////////////////////////////
// pp() Temporary debug function. Fills HEX output with
//      leading 0 if output is between 0 - 9
//
void pp (byte frameByte) {
#ifdef SERDB
  if(frameByte <= 9) DPN("0");
  DPN(frameByte, HEX);
#endif  
}
void pl (byte frameByte) {
#ifdef SERDB
  if(frameByte <= 9) DPN("0");
  DPL(frameByte, HEX);
#endif  
}

