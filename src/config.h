#pragma once
/**
 * CONFIG.H
 *
 * Configuration centrale pour le système de mouvement du robot
 * Tous les paramètres physiques et pins sont définis ici
 */


// ================= ESP32_S3 pinout ====================
// |  3V3                         | GND 
// |  3V3                         | TX  - Reserved for PSRAM (DO NOT USE)
// |  RST                         | RX  - Reserved for PSRAM (DO NOT USE)
// --------------------------------------------------------
// |  4  – STOP pin               | 1   -             
// |  5  – SDA                    | 2   -    
// |  6  - SCL                    | 42  -
// |  7  -                        | 41  - 
// |  15 - Encoder                | 40  -   
// |  16 - Encoder                | 39  - 
// |  17 - Encoder                | 38  -
// --------------------------------------------------------
// |  18 - Encoder                | 37  - / (NO) PSRAM
// |  8  - TeamSwitch             | 36  - / (NO) PSRAM        
// |  3  - / (NO) STRAP           | 35  - / (NO) PSRAM
// |  46 - / (NO) STRAP           | 0   - / (NO) BOOT
// |  9  – Motor (ENA)            | 45  - / (NO) STRAP
// |  10 – Motor (IN1)            | 48  - / (NO) 1.8V logic
// |  11 – Motor (IN2)            | 47  - / (NO) 1.8V logic
// --------------------------------------------------------
// |  12 - Motor (IN3)            | 21  - 
// |  13 - Motor (IN4)            | 20  - / (NO) USB+
// |  14 - Motor (ENB)            | 19  - / (NO) USB+


// ===== UART =====
// #define UART_TX 43  // 
// #define UART_RX 44





// ======= NEVER USE =========
#define STRAP_0 0 // Strapping: boot mode. If pulled LOW at reset, enters download mode.
#define STRAP_3 3 // Strapping: JTAG source. Must NOT float at boot. Many designs avoid it.
#define USB_DM 19 // USB D-
#define USB_DP 20 // USB D+

#define PSRAM_IO35 35 // reserved for Octal SPI PSRAM
#define PSRAM_IO36 36
#define PSRAM_IO37 37

#define STRAP_45 45 // Strapping: VDD_SPI voltage. NEVER pull HIGH at power-up.
#define STRAP_46 46 // Strapping: boot mode/ROM print. Avoid strong pull-ups.
#define IO47_1V8 47
#define IO48_1V8 48

// ======== PINS GENERAL PURPOSE (GPIOs) =======
// 1  - 2  : GPIO
// 4  - 7  : gpio, PWM, ADC, Touch
// 8  - 14 : GPIO, PWM, ADC, Touch, SPI
// 13 - 18 : GPIO, PWM, ADC, Touch
// 21 -    : GPIO, PWM, I2C

// 38 - 39 : GPIO
// 40 - 41 : GPIO
// 42 -    : GPIO, MTMS
// 43 - 44 : UART (can be GPIO but no Serial functions ?)

// AVOID :
// 0,3,19-20,35-37,45-46 : Strapping/USB/PSRAM
// TO CHECK
// 47 ‑ 48 : ONLY SAFE AT (1.8 V) SPI / diff signals (CHECK with multimeter if 3.3V ok safe to use)
//           on the ‑N16R16VA variant only, this pin operates at 1.8 V. On all other variants it’s 3.3 V.

// STRAPPING PINS :
//      COULD be used as GPIO but with certain conditions :
//      - STRAP_0 : can be used as GPIO if not pulled LOW at reset (otherwise enters download mode)
//      - STRAP_3 : can be used as GPIO if not left floating at reset
//      AVOID STRAP_45 (VDD_SPI) and STRAP_46 (boot mode/ROM print) : never pull HIGH at power-up, avoid strong pull-ups

// ==========================================
//              PINOUT
// ==========================================
#define MATCH_DURATION_MS 50000


// --- I2C ----
#define STOP_PIN    4
#define SDA_PIN     5
#define SCL_PIN     6

// ===================== IMU =====================
// Mets -1.0f si le robot corrige dans le mauvais sens (rotation divergente)
#define IMU_GYRO_Z_SIGN 1.0f

// ===================== ROTATION =====================
// Calibration angulaire : si 90° demandé donne 82° mesuré, mets 90/82 = 1.10f
#define ROTATE_TARGET_SCALE 1.13f
#define ROTATE_KP 2.2f
#define ROTATE_KD 0.25f

// =================== ACTUATORS PINS ======================
#define LAUNCH_TRIGGER_PIN 1 // TODO: not used here, impl in main.cpp startSwitch !
#define TEAM_SWITCH_PIN 8
#define EBTN_PIN 48


// #define EBTN_PIN 8   // changed to Hardware Switch
#define EBTN_USE_INTERNAL_PULLUP 1
#define EBTN_PRESSED_STATE LOW // Sans resistance externe: INPUT_PULLUP, bouton vers GND

// ===================== MOTORS ============================
#define ENA 14 // ordre pins : enA, 1, 2, 3, 4, enB
#define IN1 13
#define IN2 12 // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)
#define IN3 11
#define IN4 10
#define ENB 9 // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple)

// Power pins : out1, out2, out3, out4 (respectivement M1+, M1-, M2+, M2- sur le driver)

// ==================== ENCODERS ===========================
#define ENC_L_A 15 // Pinout : Red (M+), Black (M-), Encoder_A = Yellow, Encoder_B = White, Blue (3.3VCC), Green (GND)
#define ENC_L_B 16
#define ENC_R_A 17 // Motor 2
#define ENC_R_B 18
// PETIT PAMI
// #define ENCODER_PIN_LEFT 34
// #define ENCODER_PIN_RIGHT 35

// ==================== STEPPER ============================
#define SERVO_RIGHT 4 // need PMW pin choose : (4, 5, 6, 7, 15, 16, 17 or 18)
#define SERVO_LEFT 5


// ========= Electro Aimant ==========
#define RELAY_PIN 38
#define SWITCH_PIN 48



// ===================== ULTRASONIC =====================
#define US_TIMEOUT 20000UL
#define US_TRIG_PIN 1  // US Left :  2, 42, 41, 40, 39, 38
#define US_ECHO_PIN 2
#define US2_TRIG_PIN 42  // US Middle :
#define US2_ECHO_PIN 41
#define US3_TRIG_PIN 40  // US Right :
#define US3_ECHO_PIN 39
#define US_OBSTACLE_THRESHOLD_CM 30

// #endif#define US2_TRIG_PIN 40#define US2_TRIG_PIN 40#define US2_TRIG_PIN 40#define US2_TRIG_PIN 40#define US2_TRIG_PIN 40