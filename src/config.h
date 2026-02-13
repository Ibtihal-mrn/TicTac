#pragma once


#define MATCH_DURATION_MS 50000 

#define LAUNCH_TRIGGER_PIN A0
#define TEAM_SWITCH_PIN A2


// ======= MOTORS =======
#define IN1 33  // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)
#define IN2 25
#define ENA 32
#define IN3 26  // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple) 
#define IN4 27
#define ENB 14

// ====== ENCODERS ======
#define ENCODER_PIN_LEFT 34   // Arduino Uno: pins 2 et 3 supportent les interruptions
#define ENCODER_PIN_RIGHT 35

// ====== STEPPER =======
// #define STEPPER_M1_PIN1 8
// #define STEPPER_M1_PIN2 10
// #define STEPPER_M1_PIN3 9
// #define STEPPER_M1_PIN4 11

// #define STEPPER_M2_PIN1 4
// #define STEPPER_M2_PIN2 6
// #define STEPPER_M2_PIN3 5
// #define STEPPER_M2_PIN4 7



// Ultrasonic
#define US_TIMEOUT 20000UL
#define US_TRIG_PIN 12
#define US_ECHO_PIN 13




// #endif 