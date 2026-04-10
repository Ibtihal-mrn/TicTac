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


// TODO: remove old legacy code
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



// ------- ROUTINES ----------

void testRotation(Context &ctx){
    debugEnable(DBG_IMU);
    // clear queue
    ctx.commandQueue = std::queue<RobotCommand>(); 
    Serial.println("[TEST] rotation sequence start");


    ctx.commandQueue.push(RobotCommand{CommandType::Rotate, 180.0f, 80, 0});
    ctx.commandQueue.push(RobotCommand{CommandType::Wait, 0.0f, 0, 3000});
    ctx.commandQueue.push(RobotCommand{CommandType::Rotate, -180.0f, 80, 0});

    // ctx.commandQueue.push(RobotCommand{CommandType::Rotate, 180.0f, 100, 0});
    // ctx.commandQueue.push(RobotCommand{CommandType::Wait, 0.0f, 0, 3000});
    // ctx.commandQueue.push(RobotCommand{CommandType::Rotate, -180.0f, 100, 0});


    Serial.print("[TEST] queued=");
    Serial.println(ctx.commandQueue.size());
}


void testLinearMotion(Context &ctx){
    ctx.commandQueue = std::queue<RobotCommand>();
    Serial.println("[TEST] linear sequence start");

    debugEnable(DBG_ENCODER);

    ctx.commandQueue.push(RobotCommand{CommandType::MoveForward, 2000.0f, 100, 0});
    // ctx.commandQueue.push(RobotCommand{CommandType::Wait, 0.0f, 0, 2000});
    // ctx.commandQueue.push(RobotCommand{CommandType::MoveBackward, 1000.0f, 100, 0});

    Serial.print("[TEST] queued=");
    Serial.println(ctx.commandQueue.size());
}






// -------- helpers ----------
const char* commandTypeToString(CommandType type)
{
    switch (type) {
        case CommandType::MoveForward:   return "MoveForward";
        case CommandType::MoveBackward:   return "MoveBackward";
        case CommandType::Rotate:         return "Rotate";
        case CommandType::Wait:           return "Wait";
        case CommandType::Stop:           return "Stop";
        case CommandType::ClearQueue:     return "ClearQueue";
        case CommandType::SetDistancePID: return "SetDistancePID";
        case CommandType::SetAnglePID:    return "SetAnglePID";
        default:                          return "Unknown";
    }
}

void printCommand(const RobotCommand &cmd){
    Serial.print("[CMD] ");
    Serial.print(commandTypeToString(cmd.type));
    Serial.print(" value=");
    Serial.print(cmd.value, 2);
    Serial.print(" speed=");
    Serial.print(cmd.speed);
    Serial.print(" waitMs=");
    Serial.println(cmd.waitMs);
}



// ========== FSM ============
void robot_step(Context &ctx)
{
    if (emergencyStop) {
        // motion.abort();
        // emergencyStop = false;
        ctx.currentAction = Robot::EMERGENCY_STOP;
    }

    // BLE Stop & Updates


    // DBG
    #if DBG_FSM
        static unsigned long Lpwm = 0;
        if (millis() - Lpwm >= 2000){
            // Serial.print("state="); Serial.print(commandTypeToString(ctx.currentAction));
            // Serial.print(" busy="); Serial.print(motion.isBusy());
            // Serial.print(" stop="); Serial.println(emergencyStop);
            Lpwm = millis();
        }
    #endif
    // printIMUVal();
    // printIMUAngleTest();
    printEncodersVal();
    


    // ===================================================
    switch (ctx.currentAction)
    {   
        case Robot::INIT:

            // MOVE COMMANDS
            // testRotation(ctx);
            testLinearMotion(ctx);


            ctx.currentAction = Robot::WAIT_START;
            break;

        case Robot::WAIT_START:
            // ADD LaunchTrigger HERE
            // start 100sec timer
            // startMatchTimer(ctx);
            // debugPrintf(DBG_FSM, "FSM -> DISPATCH_CMD");
            ctx.currentAction = Robot::DISPATCH_CMD;
            break;


        case Robot::DISPATCH_CMD:
            // 1. Empty Queue
            if (ctx.commandQueue.empty()) { break; }

            // 2. Get next command
            ctx.currentCommand = ctx.commandQueue.front();
            ctx.commandQueue.pop();

            #if DBG_FSM
                printCommand(ctx.currentCommand);
                Serial.print(" queue=");
                Serial.println(ctx.commandQueue.size());
            #endif

            // 3. Dispatch to movement
            switch (ctx.currentCommand.type) {
                case  CommandType::MoveForward:
                    driveForward(ctx.currentCommand.value, ctx.currentCommand.speed);
                    ctx.currentAction = Robot::EXEC;
                    break;

                case CommandType::MoveBackward:
                    driveBackward(ctx.currentCommand.value, ctx.currentCommand.speed);
                    ctx.currentAction = Robot::EXEC;
                    break;

                case CommandType::Rotate:
                    rotate(ctx.currentCommand.value, ctx.currentCommand.speed);
                    ctx.currentAction = Robot::EXEC;
                    break;
                
                case CommandType::Wait:
                    // TODO add simple non-blocking wait timer here 
                    motion.abort();
                    ctx.waitEndMs = millis() + ctx.currentCommand.waitMs;
                    ctx.currentAction = Robot::WAIT_CMD;
                    break;

                // case STEPPER_UP:

                default: ctx.currentAction = Robot::DISPATCH_CMD; break;
            }
            break;



        case Robot::IDLE:
            break;

        case Robot::EXEC:
            if (motion.update()) {
                #if DBG_FSM
                    Serial.print("[FSM] END   ");
                    Serial.println(commandTypeToString(ctx.currentCommand.type));
                #endif
                ctx.currentAction = Robot::DISPATCH_CMD;
            }
            break;


        case Robot::WAIT_CMD:
            if ((long)(millis() - ctx.waitEndMs) >= 0) {
                #if DBG_FSM
                    Serial.print("[FSM] END   ");
                    Serial.println(commandTypeToString(ctx.currentCommand.type));
                #endif
                ctx.currentAction = ctx.commandQueue.empty() ? Robot::IDLE : Robot::DISPATCH_CMD;
            }
            break;

        case Robot::EMERGENCY_STOP:
            motion.abort();
            if (digitalRead(STOP_PIN) == LOW) {
                emergencyStop = false;
                ctx.currentAction = Robot::IDLE;
            }
            break;

        case Robot::TIMER_END:
        {
            motion.abort();
            ctx.matchActive = false;
            break;
        }

        default:
            break;
    }
}

