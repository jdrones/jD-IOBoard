#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// this code was moved from libraries/GCS_MAVLink to allow compile
// time selection of MAVLink 1.0
BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

mavlink_system_t mavlink_system = {12,1,0,0};

#include "Mavlink_compat.h"

//#ifdef MAVLINK10
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

//#else
//#include "../GCS_MAVLink/include/mavlink/v0.9/mavlink_types.h"
//#include "../GCS_MAVLink/include/mavlink/v0.9/ardupilotmega/mavlink.h"
//#endif

// true when we have received at least 1 MAVLink packet
//static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;



void request_mavlink_rates()
{
  DPL("Requesting rates");

  const int  maxStreams = 7;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
					  MAV_DATA_STREAM_EXTENDED_STATUS,
                                          MAV_DATA_STREAM_RC_CHANNELS,
					  MAV_DATA_STREAM_POSITION,
                                          MAV_DATA_STREAM_EXTRA1, 
                                          MAV_DATA_STREAM_EXTRA2,
                                          MAV_DATA_STREAM_EXTRA3};
                                          
  const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02, 0x02};

  for (int i=0; i < maxStreams; i++) {
    	  mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
					       apm_mav_system, apm_mav_component,
					       MAVStreams[i], MAVRates[i], 1);
  }
}

void read_mavlink(){
  mavlink_message_t msg; 
  mavlink_status_t status;
  
  // grabing data 
  while(Serial.available() > 0) { 
    uint8_t c = Serial.read();
//    DPN(c);
            /* allow CLI to be started by hitting enter 3 times, if no
           heartbeat packets have been received */
        if (mavlink_active == 0 && millis() < 20000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
        }
    
    // trying to grab msg  
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
//       messageCounter = 0; 
       mavlink_active = 1;
       if(mavlink_active && LeRiPatt == 6) LeRiPatt = 0;
      // handle msg

      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            DPL("MAVink HeartBeat");
            mavbeat = 1;
	    apm_mav_system    = msg.sysid;
	    apm_mav_component = msg.compid;
            apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);

            iob_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
            if(iob_mode != iob_old_mode) {
              iob_old_mode = iob_mode;
              CheckFlightMode();
            }                
            iob_nav_mode = 0;

//            if((mavlink_msg_heartbeat_get_base_mode(&msg) & MOTORS_ARMED) == MOTORS_ARMED)
            if(isBit(mavlink_msg_heartbeat_get_base_mode(&msg),MOTORS_ARMED)) {
              if(isArmedOld == 0) {
                  CheckFlightMode();
                  isArmedOld = 1;
              }    
              isArmed = 1;  
            } else {
              isArmed = 0;
              isArmedOld = 0;
            }
 
            lastMAVBeat = millis();
//            if(waitingMAVBeats == 1){
//              enable_mav_request = 1;
//            }

#ifdef SERDB            
            if(debug == 2) {
              DPN("MAV: ");
              DPN((mavlink_msg_heartbeat_get_base_mode(&msg),DEC));
              DPN("  Modes: ");
              DPN(iob_mode);
              DPN("  Armed: ");
              DPN(isArmed);
              DPN("  FIX: ");
              DPN(iob_fix_type);
              DPN("  Sats: ");
              DPN(iob_satellites_visible);
              DPN("  CPUVolt: ");
              DPN(boardVoltage);
              DPN("  BatVolt: ");
              DPN(iob_vbat_A);
              DPL(" ");
            } 
#endif          
          }
          break;
          
        case MAVLINK_MSG_ID_SYS_STATUS:
          { 
            
  //          dbPRNL("MAV SYS_STATUS");
            iob_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);
            iob_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg);
            uint16_t tmp = mavlink_msg_sys_status_get_battery_remaining(&msg);

            cellVvalue();
            
            //---------Backup Battery Frist state--------//
            if(Batt_SR.Plugin_Frist!=TRUE&&iob_vbat_A>9)
            {
              Batt_SR.Plugin_Frist=TRUE;
              Batt_Volte_Backup=iob_vbat_A;
              //----------Set Cell Active----------
              if(iob_vbat_A>18)
              { 
                 Batt_Cell_Detect=0x06;
              }
              else if(iob_vbat_A>15)
              {
                Batt_Cell_Detect=0x05;
              }
              else if(iob_vbat_A>12)
              {
                Batt_Cell_Detect=0x04;
              }
              else if(iob_vbat_A>9)
              {
                Batt_Cell_Detect=0x03;
              }   
              Frsky_Count_Order_Batt=0;       
            }
            //---------Battery Backup------------------//

//            if (tmp < 13) {
//              iob_battery_remaining_A = 0;
//            } else if (tmp < 37 ) {
//              iob_battery_remaining_A = 25;
//            } else if (tmp < 63 ) {
//              iob_battery_remaining_A = 50;
//            } else if (tmp < 88 ) {
//              iob_battery_remaining_A = 75;
//            } else {
//              iob_battery_remaining_A = 100;
//            }

            if (tmp > 0) {
              iob_battery_remaining_A = tmp;
            } else {
              iob_battery_remaining_A = tmp;
            }            
          }
          break;
          
#ifndef MAVLINK10 
        case MAVLINK_MSG_ID_GPS_RAW:
          {
//         dbPRNL("MAV ID GPS");
            iob_fix_type = mavlink_msg_gps_raw_get_fix_type(&msg);
            
            iob_hdop=mavlink_msg_gps_raw_int_get_eph(&msg);
            iob_vdop=mavlink_msg_gps_raw_int_get_epv(&msg);
//            dbPRN("GPS FIX: ");
//            dbSerial.println(iob_fix_type);
          }
          break;
        case MAVLINK_MSG_ID_GPS_STATUS:
          {
          DPL("MAV ID_GPS_STATUS");
            iob_satellites_visible = mavlink_msg_gps_status_get_satellites_visible(&msg);
          }
          break;
#else
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          { 
            iob_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
// Patch from Simon / DIYD. Converting GPS locations to correct format            
            if (iob_lat < 0) {
              iob_lat_dir = 'S';
              iob_lat = fabs(iob_lat);
            } else {
              iob_lat_dir = 'N';
            }
// start of new lat code
            deg_dat = int(iob_lat);
            dec_deg = iob_lat - deg_dat;
            min_dat = int(dec_deg * 60);
            dec_min = dec_deg * 60 - min_dat;
            sec_dat = dec_min * 60;
            iob_lat = (deg_dat * 100) + min_dat + (sec_dat/100);
 // end of new lat code
 
            iob_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
            if (iob_lon < 0) {
              iob_lon_dir = 'W';
              iob_lon = fabs(iob_lon);
            } else {
              iob_lon_dir = 'E';
            }
            
// start of new lon code
            deg_dat = int(iob_lon);
            dec_deg = iob_lon - deg_dat;
            min_dat = int(dec_deg * 60);
            dec_min = dec_deg * 60 - min_dat;
            sec_dat = dec_min * 60;
            iob_lon = (deg_dat * 100) + min_dat + (sec_dat/100);
 // end of new lon code
 
            iob_gps_alt = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000.0f;
            iob_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
            iob_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
          }
          break;

#endif          

        case MAVLINK_MSG_ID_VFR_HUD:
          {
            iob_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
            iob_heading = mavlink_msg_vfr_hud_get_heading(&msg);// * 3.60f;//0-100% of 360
            iob_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
            if(iob_throttle > 100 && iob_throttle < 150) iob_throttle = 100; //Temporary fix for ArduPlane 2.28
            if(iob_throttle < 0 || iob_throttle > 150) iob_throttle = 0; //Temporary fix for ArduPlane 2.28
            iob_alt = mavlink_msg_vfr_hud_get_alt(&msg);
          }
          break;
          
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
          {
            iob_temperature = mavlink_msg_scaled_pressure_get_temperature(&msg) / 100;  // scaling 0.01
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:
          {
//          DPL("MAV ID_ATTITUDE");
            iob_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
            iob_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
            iob_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
          }
          break;
        case MAVLINK_MSG_ID_HWSTATUS:  
          {
            // Read our HW-Status
            boardVoltage = mavlink_msg_hwstatus_get_Vcc(&msg) / 1000.0f;      
            i2cErrorCount = mavlink_msg_hwstatus_get_I2Cerr(&msg);
            //DPL(boardVoltage);
           if(boardVoltage != 0 && boardVoltage < 4.0) {
              voltAlarm = 1;
           } else voltAlarm = 0;
          }
          break;
        case MAVLINK_MSG_ID_STATUSTEXT:
          {   
           DPL(mavlink_msg_statustext_get_severity(&msg));            
          }  
          break;
        default:
          //Do nothing
          break;
      }
    }
    delayMicroseconds(138);
    //next one
  }
  // Update global packet drops counter
  packet_drops += status.packet_rx_drop_count;
  parse_error += status.parse_error;

}

void cellVvalue() {
  int i;
  //no 3S temporary fix 
  int val = (int)(2100.0*(iob_vbat_A/(float(cell_numb)))/4.2);

  //no 3S temporary fix
  if ((val > 2100) && (cell_numb < MAXCELL)) {
    cell_numb++;
    return;
  }
    
  for (i=0;i<cell_numb;i++) {
    cellV[i]=val;
  }
}




