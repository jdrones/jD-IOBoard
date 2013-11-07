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
*/
//
// jD-IOBoard definitions
// 27.10.2013

// Some basic defualts
#define EN  1     // Enable value
#define DI  0     // Disable value
#define TRUE 1    // Like we would not know what true is
#define FALSE 0   // or this too...

// Flight mode defines
#define STAB 0
#define ACRO 1
#define ALTH 2
#define AUTO 3
#define LOIT 4
#define GUID 5
#define RETL 6
#define CIRC 7
#define POSI 8
#define LAND 9
#define OFLO 10
#define MANU 11
#define FBWA 12
#define FBWB 13

// MAVLink HeartBeat bits
#define MOTORS_ARMED 128

// LEFT/RIGHT Alarm/Info pattern definitions
#define ALLOK 0
#define LOWVOLTAGE 1
//#define NOTHINGYET 2
//#define NOTHINGYET 3
#define NOLOCK 4
//#define NOTHINGYET 5
#define NOMAVLINK 6
//#define NOTHINGYET 7

//Battery
#define MAXCELL 6

///////////////////////////
// Global variables
static int counter = 0;     // General counter

static int Out[] = {0,8,9,10,4,3,2};   // Output I/O pin array
static int IOState[] = {0,0,0,0,0,0};

static byte patt_pos;
static byte patt;
static byte pattByteA;
static byte pattByteB;

static int pwm1;  // value holders for pwm outputs (if any)
static int pwm2;
static int pwm3;

static boolean pwm1dir;
static boolean pwm2dir;
static boolean pwm3dir;

// Counters and millisecond placeholders used around the code
static long p_hbMillis;                         // HeartBeat counter
static long c_hbMillis;
static long d_hbMillis = 500;

static int8_t    iob_temperature;
static float    iob_vbat_A = 0;                 // Battery A voltage in milivolt
static uint16_t iob_battery_remaining_A = 0;    // 0 to 100 <=> 0 to 1000

static uint16_t iob_mode = 0;                   // Navigation mode from RC AC2 = CH5, APM = CH8
static uint8_t  iob_nav_mode = 0;               // Navigation mode from RC AC2 = CH5, APM = CH8
static uint16_t iob_old_mode = 0;

static int cell_numb = 3;
static int cell_count = 0;
static int cellV[MAXCELL]; // Volts for each cell

static float    iob_lat = 0;                    // latidude
static float    iob_lon = 0;                    // longitude
static uint8_t  iob_satellites_visible = 0;     // number of satelites
static uint8_t  iob_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D

static unsigned int iob_hdop=0;


static uint8_t  iob_got_home = 0;               // tels if got home position or not

static int8_t      iob_pitch = 0;                  // pitch form DCM
static int8_t      iob_roll = 0;                   // roll form DCM
static int8_t      iob_yaw = 0;                    // relative heading form DCM

static float    iob_heading = 0;                // ground course heading from GPS
static float    iob_alt = 0;                    // altitude
static float    iob_gps_alt = 0;                    // altitude
static float    iob_groundspeed = 0;            // ground speed
static uint16_t iob_throttle = 0;               // throtle

//MAVLink session control
static boolean  mavbeat = 0;
static float    lastMAVBeat = 0;
static boolean  waitingMAVBeats = 1;
static uint8_t  apm_mav_type;
static uint8_t  apm_mav_system; 
static uint8_t  apm_mav_component;
static boolean  enable_mav_request = 0;

byte iob_lat_dir;
byte iob_lon_dir;

int iob_ampbatt_A;

static float deg_dat;
static float dec_deg;
static float min_dat;
static float dec_min;
static float sec_dat;

//byte cell_count;
//byte cell_numb;

int tempvar;      // Temporary variable used on many places around the IOBoard

// General states
byte flMode;      // Our current flight mode as defined
byte isArmed = 0;
byte isArmedOld = 0;
byte isActive;

// OUTPUT LED dynamic place holders
byte LEFT;
byte RIGHT;
byte FRONT;
byte REAR;
byte ledPin;    // Heartbeat LED place holder if any

byte BattAlarmPercentage; 

byte isFrSky;

// Left/Right static patterns
static byte le_patt[8][16] = {
  { 1,1,1,1,1,1,1,1 ,1,1,1,1,1,1,1,1  },    // 0
  { 0,1,0,1,0,1,0,1 ,0,1,0,1,0,1,0,1  },    // 1
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,1,0,1,0  },    // 2
  { 1,1,0,0,1,1,0,0 ,1,1,0,0,1,1,0,0  },    // 3
  { 1,1,1,1,1,1,1,1 ,1,1,0,0,1,1,0,0  },    // 4
  { 1,1,1,1,1,1,1,1 ,1,1,1,1,1,1,1,1  },    // 5
  { 0,0,0,0,0,0,0,0 ,1,1,1,1,1,1,1,1  },    // 6
  { 0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0  }};   // 7
  
static byte ri_patt[8][16] = {
  { 1,1,1,1,1,1,1,1 ,1,1,1,1,1,1,1,1  },    // 0
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,1,0,1,0  },    // 1
  { 1,0,1,0,1,0,1,0 ,1,0,1,0,1,0,1,0  },    // 2
  { 1,1,0,0,1,1,0,0 ,1,1,0,0,1,1,0,0  },    // 3
  { 1,1,1,1,1,1,1,1 ,1,1,0,0,1,1,0,0  },    // 4
  { 1,1,1,1,1,1,1,1 ,1,1,1,1,1,1,1,1  },    // 5
  { 1,1,1,1,1,1,1,1 ,0,0,0,0,0,0,0,0  },    // 6
  { 0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0  }};   // 7
  



// FrSky modules addon
static long f_curMillis;
static long f_preMillis;
static int f_delMillis = 200;

unsigned char outBuff[48];     // Array for payloads
unsigned char outBuffFixed[48];     // Array for payloads

byte msCounter;
boolean packetOpen;
byte payloadLen;


byte hour;
byte minutes;
byte second;

// FrSky module addon - END
