#include "ultrasonic.h"

void ultrasonic_init() {
    Serial.println("Ultrasonic 3 capteurs init (pins 7,10,13)");
}

int ultrasonic_readDistance(int sensorIndex) {
    if (sensorIndex < 0 || sensorIndex >= NB_CAPTEURS) return -1;
    
    // FILTRE MOYENNE 3 lectures anti-bruit moteurs
    int sum = 0, valid = 0;
    for (int i = 0; i < 3; i++) {
        unsigned int dist = sonars[sensorIndex].ping_cm();
        if (dist > 0 && dist < MAX_DISTANCE) {
            sum += dist;
            valid++;
        }
        delay(10);
    }
    return (valid > 0) ? (sum / valid) : -1;
}

int ultrasonic_readAverageDistance() {
    int d1 = ultrasonic_readDistance(0);
    int d2 = ultrasonic_readDistance(1);
    int d3 = ultrasonic_readDistance(2);
    if (d1 < 0 || d2 < 0 || d3 < 0) return -1;
    return (d1 + d2 + d3) / 3;
}

bool ultrasonic_isObstacle(int threshold) {
    for (int i = 0; i < NB_CAPTEURS; i++) {
        int dist = ultrasonic_readDistance(i);
        if (dist > 0 && dist <= threshold) return true;
    }
    return false;
}
