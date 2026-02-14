#include "ultrasonic.h"
#include <Ultrasonic.h>
#include "../../src/config.h"

Ultrasonic us(US_TRIG_PIN, US_ECHO_PIN, US_TIMEOUT);

static int PIN_TRIG;
static int PIN_ECHO;

void ultrasonic_init(int trigPin, int echoPin) {
    PIN_TRIG = trigPin;
    PIN_ECHO = echoPin;

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
}

int ultrasonic_readDistance() {
    // Envoi de l'impulsion
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // Lecture du retour
    long duree = pulseIn(PIN_ECHO, HIGH, 4000UL); // timeout 30ms (sécurité)

    if (duree == 0) {
        return -1; // Pas de mesure valide
    }

    int distance = (int)(duree / 58UL);
    return distance;
}

bool ultrasonic_isObstacle(int distance, int threshold) {
    return (distance > 0 && distance <= threshold);
}
