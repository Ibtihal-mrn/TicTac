#pragma once
/**
 * CONFIG.H
 * 
 * Configuration centrale pour le système de mouvement du robot
 * Tous les paramètres physiques et pins sont définis ici
 */

#define MATCH_DURATION_MS 50000 

// =================== ACTUATORS PINS ======================
// #define LAUNCH_TRIGGER_PIN A0
// #define TEAM_SWITCH_PIN A2
#define EBTN_PIN 8

// ===================== MOTORS ============================
#define ENA 19
#define IN1 20  
#define IN2 21  // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)

#define ENB 47  // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple) 
#define IN3 48  
#define IN4 45
// Power pins : out1, out2, out3, out4 (respectivement M1+, M1-, M2+, M2- sur le driver)


// ==================== ENCODERS ===========================
#define ENC_L_A 14   // Pinout : Red (M+), Black (M-), Encoder_A = Yellow, Encoder_B = White
#define ENC_L_B 13
#define ENC_R_A 12   // Motor 2
#define ENC_R_B 11
// PETIT PAMI
    // #define ENCODER_PIN_LEFT 34   
    // #define ENCODER_PIN_RIGHT 35


// ==================== STEPPER ============================
#define SERVO_RIGHT 4  // need PMW pin choose : (4, 5, 6, 7, 15, 16, 17 or 18)
#define SERVO_LEFT 5



// ===================== ULTRASONIC =====================
#define US_TIMEOUT 20000UL
#define US_TRIG_PIN 12
#define US_ECHO_PIN 13




// #endif 