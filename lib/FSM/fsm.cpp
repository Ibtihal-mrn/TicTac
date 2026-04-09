#include "fsm.h"
#include "../../src/config.h"
#include "../../src/globals.h"

#include "motors.h"
#include "encoders.h"
#include "us.h"
#include "imu.h"

#include "control.h"
// #include "kinematics.h"

//
#include "utils.h"
#include "Debug.h"

// I2C sensors
#include "i2c_comm.h"

#include "pid.h"


Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);
static PIDController motion(motors);




// TODO: add match timer init
void hardware_init(Context &ctx)
{
    // To be called in setup() in main.cpp

    // motors_init();
    encoders_init();
    // ultrasonic_init(13, 10);  // trig, echo
    // safety_init(40, 50);      // 40cm seuil, sonar toutes les 50ms

    // IMU
    if (!imu_init())
    {
        Serial.println("MPU6050 FAIL.");
    } else
    {
        delay(200);
        Serial.println("MPU6050 connected.");
        imu_calibrate(600, 2); // ~1.2s, robot immobile
        // Serial.println("IMU calibrated");
    }

    // Init Match Timer
    // ctx.matchActive = false;
    // ctx.matchDurationMs = MATCH_DURATION_MS;
    // ctx.matchStartMs = 0;
    // debugPrintf(DBG_FSM, "FSM -> INIT");
    ctx.currentAction = Robot::INIT;
    ctx.currentTeam = Team::TEAM_YELLOW;
}

// ---- US obstacle ------
void driveForward(float mm, int speed) {
    setZones(ZONE_FRONT | ZONE_LEFT | ZONE_RIGHT);
    motion.startLinear(mm, speed);
}

void driveBackward(float mm, int speed) {
    setZones(ZONE_BACK | ZONE_LEFT | ZONE_RIGHT);
    motion.startLinear(-mm, speed);
}

void rotate(float angle, int speed) {
    setZones(ZONE_FRONT | ZONE_LEFT | ZONE_RIGHT | ZONE_BACK);
    motion.startRotate(angle, speed);
}



// -------- helpers ----------
const char* robotStateToString(Robot state)
{
    static const char* const stateList[] = {
        "INIT",
        "IDLE",
        "DISPATCH_CMD",
        "EXEC_MOVE",
        "EXEC_ROTATE",
        "TASK",
        "TUNE_PID",
        "TIMER_END",
        "EMERGENCY_STOP"
    };

    const int index = static_cast<int>(state);
    if (index < 0 || index >= (int)(sizeof(stateList) / sizeof(stateList[0]))) {
        return "UNKNOWN";
    }

    return stateList[index];
}


// ========== FSM ============
void robot_step(Context &ctx)
{
    if (emergencyStop) {
        // motion.abort();
        // emergencyStop = false;
        // ctx.currentAction = Robot::EMERGENCY_STOP;
    }

    #if DBG_FSM
        static unsigned long Lpwm = 0;
        if (millis() - Lpwm >= 2000){
            Serial.print("state="); Serial.print(robotStateToString(ctx.currentAction));
            Serial.print(" busy="); Serial.print(motion.isBusy());
            Serial.print(" stop="); Serial.println(emergencyStop);
            Lpwm = millis();
        }
    #endif

    switch (ctx.currentAction)
    {   
        case Robot::INIT:
            // robot_init();  CALLED IN SETUP()
            
            // ctx.commandQueue.push({RobotCommandType::TUNE_PID, (float)TUNE_BOTH});
            debugPrintf(DBG_FSM, "FSM -> IDLE");
            driveForward(1000, 100);
            // ctx.currentAction = Robot::IDLE;
            ctx.currentAction = Robot::EXEC_MOVE;
            break;

        case Robot::IDLE:
            // ADD LaunchTrigger HERE
            // start 100sec timer
            // startMatchTimer(ctx);
            // debugPrintf(DBG_FSM, "FSM -> DISPATCH_CMD");
            // ctx.currentAction = Robot::DISPATCH_CMD;
            break;

        case Robot::EXEC_MOVE  :
        case Robot::EXEC_ROTATE:
            if (motion.update()) {
                ctx.currentAction = Robot::IDLE;
            }
            break;

        case Robot::EMERGENCY_STOP:
            motion.abort();
            if (digitalRead(STOP_PIN) == LOW) {
                emergencyStop = false;
                ctx.currentAction = Robot::IDLE;
            }
            break;

        default:
            break;
    }
}

