#include "safety.h"
#include "../../src/config.h"
#include "i2c_comm.h"
// #include <Ultrasonic.h>
#include "EmergencyButton.h"
#include "BLEBridge.h"

// Obstacle détecté via I2C depuis le coprocesseur sensor hub
bool safety_update()
{
    static bool obstacleLatched = false;
    static unsigned long lastUS = 0;

    if (emergencyButton_isPressed())
    {
        obstacleLatched = true;
        return obstacleLatched;
    }

    // Poll le coprocesseur à 10Hz max
    if (millis() - lastUS >= 100) {
        lastUS = millis();

        SensorPacket data = getData();

        // danger_flags != 0 signifie qu'au moins un capteur détecte un obstacle
        if (data.danger_flags != 0) {
            obstacleLatched = true;
            static unsigned long lastLog = 0;
            if (millis() - lastLog >= 500) {
                lastLog = millis();
                bleSerial.printf("[SAFETY] Obstacle! flags=0x%02X F=%dmm L=%dmm R=%dmm B=%dmm\n",
                    data.danger_flags, data.front_mm, data.left_mm, data.right_mm, data.back_mm);
            }
        } else {
            obstacleLatched = false;
        }
    }
    return obstacleLatched;
}