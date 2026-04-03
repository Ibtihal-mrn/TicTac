// config_coprocessor.h
#pragma once

// CONFIG for CoProcessor Sensor hub

// ================= ESP32_S3 pinout ====================
// |  3V3                         | GND 
// |  3V3                         | TX  - Reserved for PSRAM (DO NOT USE)
// |  RST                         | RX  - Reserved for PSRAM (DO NOT USE)
// --------------------------------------------------------
// |  4  – US Front Trig  (1)     | 1   - US Back Trig  (7)
// |  5  – US Front Echo  (1)     | 2   - US Back Trig  (7)
// |  6  - US Front Echo  (2)     | 42  - US Back Echo  (8)
// |  7  - US Front Trig  (2)     | 41  - US Back Trig  (8)
// |  15 - US Front Echo  (3)     | 40  - US Back Echo  (9)
// |  16 - US Front Trig  (3)     | 39  - US Back Trig  (9)
// |  17 - US Front Echo  (4)     | 38  - US Back Trig  (10)
// --------------------------------------------------------
// |  18 - US Left Trig   (4)     | 37  - / (NO) PSRAM
// |  8  - US Left Echo   (5)     | 36  - / (NO) PSRAM        
// |  3  - / (NO) STRAP           | 35  - / (NO) PSRAM
// |  46 - / (NO) STRAP           | 0   - / (NO) BOOT
// |  9  - US Left Trig   (5)     | 45  - / (NO) STRAP
// |  10 - US Left Echo   (6)     | 48  - / (NO) 1.8V logic
// |  11 - US Back Trig   (6)     | 47  - / (NO) 1.8V logic
// ----------------------------------------------------------
// |  12 - STOP_PIN               | 21  - US Back Echo  (10) 
// |  13 - SCL                    | 20  - / (NO) USB+
// |  14 - SDA                     | 19  - / (NO) USB+

// ~ around 23 free pins for general use
// 10 US sensors (20pins) + UART and stop pin

// RUN CMD
//
// robot_master :
//      ACM0
//          >> Upload & Monitor : pio run -t upload -t monitor -e robot_master --upload-port /dev/ttyACM0 --monitor-port /dev/ttyACM0
//          >> Monitor Only     : pio run -t monitor -e robot_master --monitor-port /dev/ttyACM0
//      ACM1
//          >> Upload & Monitor : ppio run -t upload -t monitor -e robot_master --upload-port /dev/ttyACM1 --monitor-port /dev/ttyACM1
//          >> Monitor Only     : pio run -t monitor -e robot_master --monitor-port /dev/ttyACM1
//      ACM2
//          >> Upload & Monitor : ppio run -t upload -t monitor -e robot_master --upload-port /dev/ttyACM2 --monitor-port /dev/ttyACM2
//          >> Monitor Only     : pio run -t monitor -e robot_master --monitor-port /dev/ttyACM2
//
// Sensor_hub : 
//      ACM0
//          >> Upload & Monitor : pio run -t upload -t monitor -e sensor_hub --upload-port /dev/ttyACM0 --monitor-port /dev/ttyACM0
//          >> Monitor Only     : pio run -t monitor -e sensor_hub --monitor-port /dev/ttyACM0
//      ACM1
//          >> Upload & Monitor : pio run -t upload -t monitor -e sensor_hub --upload-port /dev/ttyACM1 --monitor-port /dev/ttyACM1
//          >> Monitor Only     : pio run -t monitor -e sensor_hub --monitor-port /dev/ttyACM1
//      ACM2
//          >> Upload & Monitor : pio run -t upload -t monitor -e sensor_hub --upload-port /dev/ttyACM2 --monitor-port /dev/ttyACM2
//          >> Monitor Only     : pio run -t monitor -e sensor_hub --monitor-port /dev/ttyACM2




// I2C
#define STOP_PIN_HUB    12
#define SCL_PIN_HUB     13
#define SDA_PIN_HUB     14


#define US_TIMEOUT 20000UL
extern uint8_t US_OBSTACLE_THRESHOLD_CM;
extern uint8_t US_OBSTACLE_CLEAR_CM;     // to prevent bouncing, implement hysteresis
#define STOP_HOLD_MS    150   // small debounce


// ==================
//       FRONTs
// ==================
#define US_F1_TRIG 38     // Left
#define US_F1_ECHO 21

#define US_F2_TRIG      // Middle
#define US_F2_ECHO 

#define US_F3_TRIG     // Rigth
#define US_F3_ECHO 

// ==================
//       LEFT
// ==================
#define US_L1_TRIG      // Left
#define US_L1_ECHO 
#define US_L2_TRIG      // Middle
#define US_L2_ECHO 
#define US_L3_TRIG      // Rigth
#define US_L3_ECHO 

// ==================
//        RIGHT
// ==================
#define US_R1_TRIG      // Left
#define US_R1_ECHO 
#define US_R2_TRIG      // Middle
#define US_R2_ECHO 
#define US_R3_TRIG      // Rigth
#define US_R3_ECHO


// ==========
//   BACK
// ==========
#define US_B1_TRIG   // Left
#define US_B1_ECHO
#define US_B2_TRIG   // Middle
#define US_B2_ECHO
#define US_B3_TRIG   // Rigth
#define US_B3_ECHO




// =====================================
// =====================================
// #ifdef ESP32_DEVKIT

// Classic ESP32 Devkit Wroom :
//  
//      - AVOID : 
//              O, 1, 3, 6, 7, 8, 9, 10, 11
//              34, 35, 36, 39




// // I2C Communication :
// #define STOP_PIN 21
// #define SDA_PIN_HUB 22
// #define SCL_PIN_HUB 23


// // ==========
// //   FRONT
// // ==========
// #define US_F1_TRIG 19  // Left
// #define US_F1_ECHO 18
// #define US_F2_TRIG 5  // Middle
// #define US_F2_ECHO 17
// #define US_F3_TRIG 16  // Rigth
// #define US_F3_ECHO 4


// // ==========
// //   LEFT
// // ==========
// #define US_L1_TRIG 2  // Left
// #define US_L1_ECHO 15
// #define US_L2_TRIG 32  // Middle
// #define US_L2_ECHO 33
// #define US_L3_TRIG 25  // Rigth
// #define US_L3_ECHO 26

// // ==========
// //   RIGHT
// // ==========
// #define US_R1_TRIG 27  // Left
// #define US_R1_ECHO 14
// #define US_R2_TRIG 12  // Middle
// #define US_R2_ECHO 13
// #define US_R3_TRIG   // Rigth
// #define US_R3_ECHO


// // ==========
// //   BACK
// // ==========
// #define US_B1_TRIG   // Left
// #define US_B1_ECHO
// #define US_B2_TRIG   // Middle
// #define US_B2_ECHO
// #define US_B3_TRIG   // Rigth
// #define US_B3_ECHO