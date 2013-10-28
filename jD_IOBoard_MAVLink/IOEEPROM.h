/*


*/


/* *********************************************** */
// EEPROM Storage addresses

// Main patterns Nr 1 - 16
// These patterns are run on "REAR" output pin
#define pat01_ADDR   0     // Pattern 1
#define pat02_ADDR   2
#define pat03_ADDR   4
#define pat04_ADDR   6
#define pat05_ADDR   8
#define pat06_ADDR   10    // Pattern 6
#define pat07_ADDR   12
#define pat08_ADDR   14
#define pat09_ADDR   16
#define pat10_ADDR   18
#define pat11_ADDR   20    // Pattern 11
#define pat12_ADDR   22
#define pat13_ADDR   24
#define pat14_ADDR   26
#define pat15_ADDR   28
#define pat16_ADDR   30    // Pattern 16

// Reserved for future pattern placeholders 17-32
#define pat17_ADDR   32
#define pat18_ADDR   34
#define pat19_ADDR   36
#define pat20_ADDR   38
#define pat21_ADDR   40
#define pat22_ADDR   42
#define pat23_ADDR   44
#define pat24_ADDR   46
#define pat25_ADDR   48
#define pat26_ADDR   50
#define pat27_ADDR   52
#define pat28_ADDR   54
#define pat29_ADDR   56
#define pat30_ADDR   58
#define pat31_ADDR   60
#define pat32_ADDR   62

// Configuration and other binders starts from 0x64

// 64 - 127 are general flight mode binders to light pattern
#define mbind01_ADDR  64  // 64 mode, 65 pattern
#define mbind02_ADDR  66  // 66 mode, 67 pattern
#define mbind03_ADDR  68
#define mbind04_ADDR  70
#define mbind05_ADDR  72
#define mbind06_ADDR  74
#define mbind07_ADDR  76
#define mbind08_ADDR  78
#define mbind09_ADDR  80
#define mbind10_ADDR  82
#define mbind11_ADDR  84
#define mbind12_ADDR  86
#define mbind13_ADDR  88
#define mbind14_ADDR  90
#define mbind15_ADDR  92
#define mbind16_ADDR  94

//#define mbind32_ADDR  127
// Reserver till 127 for binders

#define LEFT_IO_ADDR  128
#define RIGHT_IO_ADDR 129
#define FRONT_IO_ADDR 130
#define REAR_IO_ADDR  131
#define FLASH_IO_ADDR 132
#define LEDPIN_IO_ADDR 133

#define ISFRSKY 140

// Internal version, check placeholders
#define CHK1 1000
#define CHK2 1006
#define VERS 1010

#define EEPROM_MAX_ADDR 1024 // This is maximum for atmel 328 chip





