#include "ultrasonic.h"
#include "../../src/config.h"
#include "../utils/Debug.h"

static int PIN_TRIG = US_TRIG_PIN;
static int PIN_ECHO = US_ECHO_PIN;

int obstcle_threshold_cm = US_OBSTACLE_THRESHOLD_CM; // configurable dans config.h

// Implémentation manuelle du capteur ultrason
int ultrasonic_readDistance()
{
    // Initialisation des pins si nécessaire
    static bool initialized = false;
    if (!initialized)
    {
        pinMode(PIN_TRIG, OUTPUT);
        pinMode(PIN_ECHO, INPUT);
        initialized = true;
    }

    // Envoi de l'impulsion
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // Lecture du retour
    long duree = pulseIn(PIN_ECHO, HIGH, US_TIMEOUT); // timeout défini dans config.h

    if (duree == 0)
    {
        return -1; // Pas de mesure valide
    }

    int distance = (int)(duree / 58UL); // Conversion en cm
    return distance;
}

// Public
bool ultrasonic_isObstacle()
{
    int8_t distance = ultrasonic_read();
    debugPrintf(DBG_SENSORS, "Ultrasonic distance: %d cm\n", distance);
    return (distance > 0 && distance <= obstcle_threshold_cm);
}

// Private
int8_t ultrasonic_read()
{
    return ultrasonic_readDistance();
}

// ------ Debug ------
void printUltrasonicVal()
{
    static unsigned long millis_print = 0;
    if (millis() - millis_print >= 2000)
    {
        int8_t val = ultrasonic_read();
        Serial.print("Ultrasonic: ");
        Serial.print(val);
        Serial.println(" cm");

        millis_print = millis();
    }
}