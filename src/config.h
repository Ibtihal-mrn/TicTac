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
// |  4  – STOP pin               | 1   - Servo Left            
// |  5  – SDA                    | 2   - Servo Right
// |  6  - SCL                    | 42  -
// |  7  -                        | 41  - RelaySwitch
// |  15 - Encoder                | 40  - Relay
// |  16 - Encoder                | 39  - Team Switch
// |  17 - Encoder                | 38  - Start Switch
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
#define PSRAM_UART_TX 49  // UART reserved for PSRAM
#define PSRAM_UART_RX 50  // NOT viable for standard UART communication 


// ==========================================
//              PINOUT
// ==========================================

#define MATCH_DURATION_MS 50000

// ==================== I2C ====================
#define STOP_PIN    4
#define SDA_PIN     5      // WhiteTrig  (4)
#define SCL_PIN     6     // Yellow
#define I2C_FREQ    100000

// =================== ACTUATORS PINS ====================
#define LAUNCH_TRIGGER_PIN 38     // TODO: not used here, impl in main.cpp startSwitch !
#define TEAM_SWITCH_PIN    39
// ==================== Electro Aimant ====================
#define RELAY_PIN 40
//#define SWITCH_PIN 41

// ==================== STEPPER ====================
#define SERVO_LEFT  1 // need PMW pin choose : (4, 5, 6, 7, 15, 16, 17 or 18)
#define SERVO_RIGHT 2


// ==================== MOTORS ====================
#define ENA 14 // ordre pins : enA, 1, 2, 3, 4, enB
#define IN1 13
#define IN2 12 // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)
#define IN3 11
#define IN4 10
#define ENB 9 // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple)

// Power pins : out1, out2, out3, out4 (respectivement M1+, M1-, M2+, M2- sur le driver)

// ==================== ENCODERS ====================
#define ENC_R_A 15 // Pinout : Red (M+), Black (M-), Encoder_A = Yellow, Encoder_B = White, Blue (3.3VCC), Green (GND)
#define ENC_R_B 16

#define ENC_L_A 17 // Motor 2
#define ENC_L_B 18



// ==================== ROTATION ====================
// Calibration angulaire : si 90° demandé donne 82° mesuré, mets 90/82 = 1.10f
#define ROTATE_TARGET_SCALE 1.13f
#define ROTATE_KP 2.2f
#define ROTATE_KD 0.25f


// =================== WHEELS =======================
#define WHEELDIAMM    68.44f     // perimetre=21.5cm donc p=2*pi*r <=> r = p/2pi et D=70.03mm
#define TRACKWIDTHMM  200.0f   // entraxe (centre roue gauche a centre roue droite) = 20cm
#define TICKSPERREV   3447 

// ticks : 
//      test1 : 200cm goal but actual 133cm (Encoders: L=21403 R=21242)
//      avg = 21322.5 (L+R/2)
//      ticksperrev = avg*wheelPerimeter/dist = 21322.5*215/1330 = 34 468.7


// ================== FreeRTOS ====================
#define BLE_TASK_STACK  4096
#define FSM_TASK_STACK  4096
#define BLE_TASK_PRIO   1
#define FSM_TASK_PRIO   2    // FSM légèrement plus prioritaire
