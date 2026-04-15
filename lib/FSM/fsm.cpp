#include "fsm.h"
#include "../../src/config.h"
#include "../../src/globals.h"

#include "motors.h"
#include "encoders.h"
#include "us.h"
#include "imu.h"
#include "pid.h"

// #include "control.h"
// #include "kinematics.h"

//
#include "utils.h"
#include "Debug.h"
#include "BLEBridge.h"

// I2C sensors
#include "i2c_comm.h"



Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);
static PIDController motion(motors);





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
    // ctx.currentAction = Robot::INIT;
    // ctx.currentTeam = Team::TEAM_YELLOW;
}

void fsm_init(Context& ctx, QueueHandle_t cmdQueue) {
    
    // Init freeRtos Queue
    ctx.commandQueue = cmdQueue;
    memset(&ctx.currentCommand, 0, sizeof(RobotCommand));

    // Context
    ctx.currentAction = Robot::INIT;
    ctx.currentTeam = Team::TEAM_YELLOW;
    // TOD: add match timer init
    // Init Match Timer
    // ctx.matchActive = false;
    // ctx.matchDurationMs = MATCH_DURATION_MS;
    // ctx.matchStartMs = 0;
    // debugPrintf(DBG_FSM, "FSM -> INIT");
    
    Serial.println("[FSM] Context initialized");
    bleSerial.println("[FSM] Context initialized");

}


// ---- US obstacle movement -------
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
    // debugEnable(DBG_IMU);
    // clear queue
    // ctx.commandQueue = std::queue<RobotCommand>(); 
    // Serial.println("[TEST] rotation sequence start");


    // ctx.commandQueue.push(RobotCommand{CommandType::Rotate, 180.0f, 80, 0});
    // ctx.commandQueue.push(RobotCommand{CommandType::Wait, 0.0f, 0, 3000});
    // ctx.commandQueue.push(RobotCommand{CommandType::Rotate, -180.0f, 80, 0});

    // ctx.commandQueue.push(RobotCommand{CommandType::Rotate, 180.0f, 100, 0});
    // ctx.commandQueue.push(RobotCommand{CommandType::Wait, 0.0f, 0, 3000});
    // ctx.commandQueue.push(RobotCommand{CommandType::Rotate, -180.0f, 100, 0});


    Serial.print("[TEST] queued=");
    // Serial.println(ctx.commandQueue.size());
}
void testLinearMotion(Context &ctx){
    if (!ctx.commandQueue) return;

    RobotCommand move1{CommandType::MoveForward, 1000.0f, 80, 0};
    xQueueSendToBack(ctx.commandQueue, &move1, 0);

    RobotCommand wait1{CommandType::Wait, 0.0f, 0, 5000};
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand move2{CommandType::MoveForward, 900.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move2, 0);
    
//     RobotCommand rotate1{CommandType::Rotate, 90.0f, 200, 0};
//     xQueueSendToBack(ctx.commandQueue, &rotate1, 0);

//     RobotCommand rotate2{CommandType::Rotate, -90.0f, 200, 0};
//     xQueueSendToBack(ctx.commandQueue, &rotate2, 0);
}

void enqueueTestRotation(QueueHandle_t q) {  //TODO: test this
    RobotCommand cmd;
    cmd.type = CommandType::Rotate;
    cmd.value = 180.0f;
    cmd.speed = 80;
    cmd.waitMs = 0;
    xQueueSendToBack(q, &cmd, 0);

    cmd.type = CommandType::Wait;
    cmd.value = 0;
    cmd.speed = 0;
    cmd.waitMs = 3000;
    xQueueSendToBack(q, &cmd, 0);
}



// -------- helpers ----------
const char* commandTypeToString(CommandType type)
{
    switch (type) {
        case CommandType::MoveForward:   return "MoveForward";
        case CommandType::MoveBackward:   return "MoveBackward";
        case CommandType::Rotate:         return "Rotate";
        case CommandType::Wait:           return "Wait";
        case CommandType::DeployServo:    return "DeployServo";
        case CommandType::RetractServo:   return "RetractServo";
        case CommandType::Ping:           return "Ping";
        case CommandType::ClearQueue:     return "ClearQueue";
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
static void clearCommandQueue(QueueHandle_t q) {
    if (!q) return;
    RobotCommand dropped;
    while (xQueueReceive(q, &dropped, 0) == pdTRUE) {
        // drop everything
    }
}


// ========== FSM ============
void robot_step(Context &ctx)
{
    if (emergencyStopUS) {
        // motion.abort();
        // emergencyStopUS = false;
        // ctx.currentAction = Robot::EMERGENCY_STOP;
        static unsigned long Lpwm = 0;
        if (millis() - Lpwm >= 2000){
            bleSerial.println("emergencyStopUS");
            Lpwm = millis();
        }
    }

    // BLE Stop & Updates
    if (bleStopRequested) {
        motion.abort();
        clearCommandQueue(ctx.commandQueue);
        ctx.currentAction = Robot::DISPATCH_CMD;
        bleStopRequested = false;

        Serial.println("[FSM] BLE STOP");
        bleSerial.println("[FSM] BLE STOP");
        return;
    }

    // DBG
    #if DBG_FSM
        static unsigned long lastStatePrintMs = 0;
        if (millis() - lastStatePrintMs >= 2000) {
            Serial.print("[FSM] state=");
            Serial.print(stateList[(int)ctx.currentAction]);
            Serial.print(" stop=");
            Serial.print(emergencyStopUS);
            Serial.print(" busy=");
            Serial.println(motion.isBusy());
            lastStatePrintMs = millis();
        }
    #endif
    // printIMUVal();
    // printIMUAngleTest();
    // printEncodersVal();
    


    // ===================================================
    switch (ctx.currentAction)
    {   
        case Robot::INIT:

            // MOVE COMMANDS
            // testRotation(ctx);
            // testLinearMotion(ctx);


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
            if (!ctx.commandQueue) { break;}

            // 2. Get next command
            if (xQueueReceive(ctx.commandQueue, &ctx.currentCommand, 0) != pdTRUE) { break; }


            #if DBG_FSM
                printCommand(ctx.currentCommand);
                Serial.print(" queue=");
                Serial.println(ctx.commandQueue ? uxQueueMessagesWaiting(ctx.commandQueue) : 0);
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
                    motion.abort();
                    ctx.waitEndMs = millis() + ctx.currentCommand.waitMs;
                    ctx.currentAction = Robot::EXEC_WAIT;
                    break;

                case CommandType::DeployServo:
                    break;
                case CommandType::RetractServo:
                    break;

                case CommandType::ClearQueue:
                    clearCommandQueue(ctx.commandQueue);
                    break;


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


        case Robot::EXEC_WAIT:
            if ((long)(millis() - ctx.waitEndMs) >= 0) {
                #if DBG_FSM
                    Serial.print("[FSM] END   ");
                    Serial.println(commandTypeToString(ctx.currentCommand.type));
                #endif
                ctx.currentAction = (ctx.commandQueue && uxQueueMessagesWaiting(ctx.commandQueue) > 0)
                    ? Robot::DISPATCH_CMD
                    : Robot::IDLE;            }
            break;

        case Robot::EMERGENCY_STOP:
            motion.abort();
            if (digitalRead(STOP_PIN) == LOW) {
                emergencyStopUS = false;
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

