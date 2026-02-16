#include "ultrasonic.h"
#include <Ultrasonic.h>
#include "../../src/config.h"
#include "../utils/Debug.h"


Ultrasonic us(US_TRIG_PIN, US_ECHO_PIN, US_TIMEOUT);

int obstcle_threshold_cm = US_OBSTACLE_THRESHOLD_CM;  // configurable dans config.h

// Public
bool ultrasonic_isObstacle() {
    int8_t distance = ultrasonic_read();
    debugPrintf(DBG_SENSORS, "Ultrasonic distance: %d cm\n", distance);
    return (distance > 0 && distance <= obstcle_threshold_cm);
}

// Private
int8_t ultrasonic_read() {
    return us.read(CM);
}

// ------ Debug ------
void printUltrasonicVal() {
    static unsigned long millis_print = 0;
    if(millis() - millis_print >= 2000) { 
        int8_t val = ultrasonic_read();
        Serial.print("Ultrasonic: ");
        Serial.print(val);
        Serial.println(" cm");

        millis_print = millis(); 
    }
}










// static int PIN_TRIG;
// static int PIN_ECHO;

// void ultrasonic_init(int trigPin, int echoPin) {
//     PIN_TRIG = trigPin;
//     PIN_ECHO = echoPin;

//     pinMode(PIN_TRIG, OUTPUT);
//     pinMode(PIN_ECHO, INPUT);
// }

// int ultrasonic_readDistance() {
//     // Envoi de l'impulsion
//     digitalWrite(PIN_TRIG, LOW);
//     delayMicroseconds(2);
//     digitalWrite(PIN_TRIG, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(PIN_TRIG, LOW);

//     // Lecture du retour
//     long duree = pulseIn(PIN_ECHO, HIGH, 4000UL); // timeout 30ms (sécurité)

//     if (duree == 0) {
//         return -1; // Pas de mesure valide
//     }

//     int distance = (int)(duree / 58UL);
//     return distance;
// }

// bool ultrasonic_isObstacle(int distance, int threshold) {
//     return (distance > 0 && distance <= threshold);
// }
