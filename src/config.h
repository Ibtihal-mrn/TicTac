#pragma once
/**
 * CONFIG.H
 * 
 * Configuration centrale pour le système de mouvement du robot
 * Tous les paramètres physiques et pins sont définis ici
 */

// ======= NEVER USE =========
#define STRAP_0   0   // Strapping: boot mode. If pulled LOW at reset, enters download mode.
#define STRAP_3   3   // Strapping: JTAG source. Must NOT float at boot. Many designs avoid it.
#define USB_DM   19   // USB D-
#define USB_DP   20   // USB D+

#define PSRAM_IO35 35  // reserved for Octal SPI PSRAM
#define PSRAM_IO36 36
#define PSRAM_IO37 37

#define STRAP_45 45   // Strapping: VDD_SPI voltage. NEVER pull HIGH at power-up.
#define STRAP_46 46   // Strapping: boot mode/ROM print. Avoid strong pull-ups.
#define IO47_1V8  47
#define IO48_1V8  48

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


// 
#define MATCH_DURATION_MS 50000 

// =================== ACTUATORS PINS ======================
// #define LAUNCH_TRIGGER_PIN A0
// #define TEAM_SWITCH_PIN A2
#define EBTN_PIN 8

// ===================== MOTORS ============================
#define ENA 14  // ordre pins : enA, 1, 2, 3, 4, enB
#define IN1 13  
#define IN2 12  // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)
#define IN3 11  
#define IN4 10
#define ENB 9  // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple) 

// Power pins : out1, out2, out3, out4 (respectivement M1+, M1-, M2+, M2- sur le driver)


// ==================== ENCODERS ===========================
#define ENC_L_A 15   // Pinout : Red (M+), Black (M-), Encoder_A = Yellow, Encoder_B = White, Blue (3.3VCC), Green (GND)
#define ENC_L_B 16
#define ENC_R_A 17   // Motor 2
#define ENC_R_B 18
// PETIT PAMI
    // #define ENCODER_PIN_LEFT 34   
    // #define ENCODER_PIN_RIGHT 35


// ==================== STEPPER ============================
#define SERVO_RIGHT 4  // need PMW pin choose : (4, 5, 6, 7, 15, 16, 17 or 18)
#define SERVO_LEFT 5



// ===================== I2C ============================
#define I2C_SDA  6   // GPIO6 = SDA (câblé sur le PCB)
#define I2C_SCL  7   // GPIO7 = SCL (câblé sur le PCB)
#define I2C_FREQ 100000  // 100 kHz — standard mode (compatible avec tous les chips I2C)

// ===================== IO EXPANDER =====================
//  Adresse I2C du TCA9554 / PCF8574 (dépend de A0/A1/A2 sur le chip)
//  A0=GND, A1=GND, A2=GND → 0x20. Vérifie avec un i2c_scanner() si besoin.
#define IOEXP_I2C_ADDR   0x20

//  Stack de la tâche FreeRTOS de l'IO Expander (en bytes)
//  2048 est suffisant pour de simples lectures I2C
#define IOEXP_TASK_STACK 2048
#define IOEXP_TASK_PRIO  1     // même priorité que BLE (pas critique en timing)

//  Registres du TCA9554 (compatible PCF8574 pour le reg 0x00)
//  0x00 = Input Port  : lire l'état des pins
//  0x01 = Output Port : écrire sur les pins configurées en output
//  0x02 = Polarity    : inverser la logique (optionnel)
//  0x03 = Config      : 1 = input, 0 = output (par pin)
//
//  Exemple : pins P0-P3 en input, P4-P7 en output → 0x0F
#define IOEXP_PIN_CONFIG 0x0F

// ===================== ULTRASONIC =====================
#define US_TIMEOUT 20000UL
#define US_TRIG_PIN 37
#define US_ECHO_PIN 38
#define US_OBSTACLE_THRESHOLD_CM 20



// #endif 