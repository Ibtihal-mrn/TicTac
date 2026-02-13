#pragma once


#define MATCH_DURATION_MS 50000 

#define LAUNCH_TRIGGER_PIN A0
#define TEAM_SWITCH_PIN A2


// ======= MOTORS =======
#define ENA 19
#define IN1 20  
#define IN2 21  // LEFT MOTOR : IN1, IN2, ENA (marron, gris, bleu)

#define ENB 47  // RIGHT MOTOR : IN3, IN4, ENB (rouge, jaune, purple) 
#define IN3 48  
#define IN4 45
// Power pins : out1, out2, out3, out4 (respectivement M1+, M1-, M2+, M2- sur le driver)


// ====== ENCODERS ======
#define ENCODER_PIN_LEFT 34   // Arduino Uno: pins 2 et 3 supportent les interruptions
#define ENCODER_PIN_RIGHT 35

#define ENC_L_A 2   // interruption
#define ENC_L_B 5
#define ENC_R_A 3  // interruption
#define ENC_R_B 9


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