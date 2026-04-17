#include "fsm.h"
#include "../../src/config.h"
#include "../../src/globals.h"

#include "motors.h"
#include "bras.h"
#include "encoders.h"
#include "us.h"
#include "imu.h"
#include "pid.h"
#include "Relais.h"

// #include "control.h"
// #include "kinematics.h"
#include "StartSwitch.h"
#include "TeamSwitch.h"

//
#include "utils.h"
#include "Debug.h"
#include "BLEBridge.h"

// I2C sensors
#include "i2c_comm.h"

Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);
static PIDController motion(motors);
StartSwitch startSwitch(GPIO_NUM_38);
TeamSwitch teamSwitch((gpio_num_t)TEAM_SWITCH_PIN);

// ------- INITS ---------
void fsm_init(Context& ctx, QueueHandle_t cmdQueue) {
    
    // Init freeRtos Queue
    ctx.commandQueue = cmdQueue;
    memset(&ctx.currentCommand, 0, sizeof(RobotCommand));

    // Context
    ctx.currentAction = Robot::INIT;
    ctx.currentTeam = Team::TEAM_YELLOW;
    // TOD: add match timer init
    // Init Match Timer
    ctx.matchActive = false;
    ctx.matchDurationMs = MATCH_DURATION_MS;
    ctx.matchStartMs = 0;
    debugPrintf(DBG_FSM, "FSM -> INIT");
    
    Serial.println("[FSM] Context initialized");
    bleSerial.println("[FSM] Context initialized");

}

void hardware_init(Context &ctx)
{
    // To be called in setup() in main.cpp
    bras_init();
    encoders_init();

    // Init relais
    relais_init(RELAY_PIN, true);

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
    ctx.matchActive = false;
    ctx.matchDurationMs = MATCH_DURATION_MS;
    ctx.matchStartMs = 0;
    // debugPrintf(DBG_FSM, "FSM -> INIT");

    // Read switch
    ctx.currentTeam = Team::TEAM_YELLOW;    // TODO: read TeamSwicth
    // heartbeatTeamName(); // done in main.cpp
    ctx.currentAction = Robot::INIT;

}


// Match Timer
void startMatchTimer(Context& ctx) {
    ctx.matchActive = true;
    ctx.matchStartMs = millis();
    ctx.matchDurationMs = MATCH_DURATION_MS;
}

bool isMatchTimeEnded(const Context& ctx) {
    return ctx.matchActive && (millis() - ctx.matchStartMs >= ctx.matchDurationMs);
}



// ---- US obstacle movement -------
void driveForward(float mm, int speed) {
    setZones(ZONE_FRONT);
    motion.startLinear(mm, speed);
}
void driveBackward(float mm, int speed) {
    setZones(ZONE_BACK);
    motion.startLinear(-mm, speed);
}
void rotate(float angle, int speed) {
    setZones(ZONE_FRONT | ZONE_LEFT | ZONE_RIGHT | ZONE_BACK);
    motion.startRotate(angle, speed);
}



static void emergencyBrakePulse(float gyroRateDps, CommandType lastCmdType) {
    // keep your forward braking behavior unchanged
    if (lastCmdType == CommandType::MoveForward) {
        const float baseBrake = 230.0f;
        const float extraBrake = constrain(fabsf(gyroRateDps) * 1.5f, 0.0f, 40.0f);
        const float brakePwm = constrain(baseBrake + extraBrake, 50.0f, 240.0f);

        motors.applyMotorOutputs(-brakePwm, -brakePwm);
        vTaskDelay(pdMS_TO_TICKS(90));
        motors.stopMotors();
        return;
    }

    // backward braking: opposite of backward motion
    if (lastCmdType == CommandType::MoveBackward) {
        const float brakePwm = 230.0f;   // adjust if needed
        motors.applyMotorOutputs(brakePwm, brakePwm);
        vTaskDelay(pdMS_TO_TICKS(70));
        motors.stopMotors();
        return;
    }

    // rotation braking: gentler, shorter pulse to avoid reversing too far
    if (lastCmdType == CommandType::Rotate) {
        if (fabsf(gyroRateDps) < 5.0f) {
            motors.stopMotors();
            return;
        }

        const float brakePwm = constrain(200.0f + fabsf(gyroRateDps) * 0.35f, 25.0f, 230.0f);

        // positive gyro => rotate one way, brake with opposite torque
        if (gyroRateDps > 0.0f) {
            motors.applyMotorOutputs(brakePwm, -brakePwm);
        } else {
            motors.applyMotorOutputs(-brakePwm, brakePwm);
        }

        vTaskDelay(pdMS_TO_TICKS(35));
        motors.stopMotors();
        return;
    }

    motors.stopMotors();
}


// ------- ROUTINES ----------
void testRotation(Context &ctx){
    RobotCommand rot1{CommandType::Rotate, 90.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot1, 0);
    RobotCommand wait1{CommandType::Wait}; xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand rot2{CommandType::Rotate, -90.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot2, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand rot3{CommandType::Rotate, 360.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot3, 0);



}
void testLinearMotion(Context &ctx){
    if (!ctx.commandQueue) return;

    RobotCommand move1{CommandType::MoveForward, 500.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move1, 0);

    RobotCommand wait1{CommandType::Wait, 0.0f, 0, 2000};
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand move2{CommandType::MoveBackward, 500.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move2, 0);
    
//     RobotCommand rotate1{CommandType::Rotate, 90.0f, 200, 0};
//     xQueueSendToBack(ctx.commandQueue, &rotate1, 0);

//     RobotCommand rotate2{CommandType::Rotate, -90.0f, 200, 0};
//     xQueueSendToBack(ctx.commandQueue, &rotate2, 0);
}

void testServos(Context &ctx){
    if (!ctx.commandQueue) return;

    RobotCommand deploy1{CommandType::DeployServo};
    xQueueSendToBack(ctx.commandQueue, &deploy1, 0);

    RobotCommand wait1{CommandType::Wait, 0.0f, 0, 2000};
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand retract1{CommandType::RetractServo};
    xQueueSendToBack(ctx.commandQueue, &retract1, 0);
}


void testElectroAimant(Context &ctx){
    // ========= SEQUENCE ELECTROAIMANT =========
    // Avancer 10 cm
    RobotCommand move1{CommandType::MoveForward, 100.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move1, 0);

    // Arrêt 2 secondes + relais on pendant ce temps
    RobotCommand wait1{CommandType::Wait, 0.0f, 0, 2000};
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand relOn{CommandType::RelaisOn, 0.0f, 0, 0};
    xQueueSendToBack(ctx.commandQueue, &relOn, 0);

    // Avancer 10 cm relais actif
    RobotCommand move2{CommandType::MoveForward, 100.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move2, 0);

    // Relais off
    RobotCommand relOff{CommandType::RelaisOff, 0.0f, 0, 0};
    xQueueSendToBack(ctx.commandQueue, &relOff, 0);

    // Continuer 10 cm
    RobotCommand move3{CommandType::MoveForward, 100.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move3, 0);
}

void competitionRoutine(Context &ctx){
    if (!ctx.commandQueue) return;

    // SERVO
    RobotCommand servo1{CommandType::DeployServo}; xQueueSendToBack(ctx.commandQueue, &servo1, 0);
    RobotCommand wait1{CommandType::Wait}; xQueueSendToBack(ctx.commandQueue, &wait1, 0);
    RobotCommand servo2{CommandType::RetractServo}; xQueueSendToBack(ctx.commandQueue, &servo2, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);


    // MOVE FORWARD 130cm-à
    RobotCommand move1{CommandType::MoveForward, 1690.0f, 200, 0}; xQueueSendToBack(ctx.commandQueue, &move1, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    // ROTATE -90°s
    RobotCommand rot1{CommandType::Rotate, 90.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot1, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    // MOVE FORWARD 30cm
    RobotCommand move2{CommandType::MoveForward, 500.0f, 200, 0}; xQueueSendToBack(ctx.commandQueue, &move2, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    // ROTATE -90°
    RobotCommand rot2{CommandType::Rotate, 80.0f, 90, 0}; xQueueSendToBack(ctx.commandQueue, &rot2, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    // MOVE FORWARD 150cm
    RobotCommand move3{CommandType::MoveForward, 1700.0f, 200, 0}; xQueueSendToBack(ctx.commandQueue, &move3, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    // MANGER :  SERVO FINISHER: 20 deploy/retract cycles
    for (int i = 0; i < 20; i++) {
        RobotCommand deploy{CommandType::DeployServo};
        xQueueSendToBack(ctx.commandQueue, &deploy, 0);

        RobotCommand waitDeploy{CommandType::Wait, 0.0f, 0, 300};
        xQueueSendToBack(ctx.commandQueue, &waitDeploy, 0);

        RobotCommand retract{CommandType::RetractServo};
        xQueueSendToBack(ctx.commandQueue, &retract, 0);

        RobotCommand waitRetract{CommandType::Wait, 0.0f, 0, 300};
        xQueueSendToBack(ctx.commandQueue, &waitRetract, 0);
    }

}

void testObstacleBrakeRoutine(Context &ctx) {
    if (!ctx.commandQueue) return;

    // 1) Forward obstacle test
    // Put an obstacle in FRONT so the US stop/brake can trigger.
    RobotCommand forward1{CommandType::MoveForward, 800.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &forward1, 0);

    RobotCommand wait1{CommandType::Wait, 0.0f, 0, 1500};
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    // 2) Backward obstacle test
    // Put an obstacle in BACK so it can trigger while reversing.
    RobotCommand backward1{CommandType::MoveBackward, 500.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &backward1, 0);

    RobotCommand wait2{CommandType::Wait, 0.0f, 0, 1500};
    xQueueSendToBack(ctx.commandQueue, &wait2, 0);

    // 3) Rotation obstacle test
    // Put obstacles around the robot, especially on the side where it rotates into them.
    RobotCommand rotate1{CommandType::Rotate, 180.0f, 80, 0};
    xQueueSendToBack(ctx.commandQueue, &rotate1, 0);

    RobotCommand wait3{CommandType::Wait, 0.0f, 0, 1500};
    xQueueSendToBack(ctx.commandQueue, &wait3, 0);

    RobotCommand rotate2{CommandType::Rotate, -180.0f, 80, 0};
    xQueueSendToBack(ctx.commandQueue, &rotate2, 0);
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



void debugPrints(Context &ctx){
    #if DBG_FSM
        static unsigned long lastStatePrintMs = 0;
        if (millis() - lastStatePrintMs >= 2000) {
            bleSerial.print("[FSM] state="); bleSerial.print(stateList[(int)ctx.currentAction]);
            bleSerial.print(" stop="); bleSerial.print(emergencyStopUS);
            Serial.print(" busy="); Serial.println(motion.isBusy());
            lastStatePrintMs = millis();
        }
    #endif
    // printIMUVal();
    // printIMUAngleTest();
    printEncodersVal();
    
}

// Stop Conditions
void US_STOP(Context &ctx){
    if (emergencyStopUS) {
        const float gyroRateDps = imu_readGyroZ_dps();

        // Go backwards when front obstacle
        if (ctx.currentCommand.type == CommandType::MoveForward) {
            ctx.pendingBlockedRecovery = true;
            ctx.blockedRecoverySinceMs = millis();
        }

        motion.abort();
        emergencyBrakePulse(gyroRateDps, ctx.currentCommand.type);


        emergencyStopUS = false;
        ctx.currentAction = Robot::EMERGENCY_STOP_US;
        static unsigned long Lpwm = 0;
        if (millis() - Lpwm >= 2000){ bleSerial.println("emergencyStopUS"); Lpwm = millis(); }

        // Turn On builtin led
        #ifdef LED_BUILTIN
            digitalWrite(LED_BUILTIN, HIGH);
        #endif
    }
}
void BLE_STOP(Context &ctx){
    if (bleStopRequested) {
        motion.abort();
        clearCommandQueue(ctx.commandQueue);
        ctx.currentAction = Robot::DISPATCH_CMD;
        bleStopRequested = false;

        Serial.println("[FSM] BLE STOP");
        bleSerial.println("[FSM] BLE STOP");
        // return true;
    // } else { return false;}
    }
}


// ========== FSM ============
void robot_step(Context &ctx)
{

    // Stop Conditions
    BLE_STOP(ctx);
    if(ctx.currentAction == Robot::EXEC) US_STOP(ctx);   //TODO: only apply for mvmnt, not servo ?
    
    // US obstacle LED
    if (emergencyStopUS) { digitalWrite(LED_BUILTIN, HIGH);} else{digitalWrite(LED_BUILTIN, LOW);}

    // DBG
    debugPrints(ctx);
    
    // Match time end check
    // if (isMatchTimeEnded(ctx)) { ctx.currentAction = Robot::TIMER_END; }

    // ===================================================
    switch (ctx.currentAction)
    {   
        case Robot::INIT:
            // MOVE COMMANDS
            // testRotation(ctx);
            // testLinearMotion(ctx);
            // testServos(ctx);
            // testElectroAimant(ctx);
            
            // competitionRoutine(ctx);
            // testObstacleBrakeRoutine(ctx);
            ctx.currentAction = Robot::WAIT_START;
            break;

        case Robot::WAIT_START:
            // ADD LaunchTrigger HERE
            // startSwitch.begin();
            // if (startSwitch.isInserted()) {
                // start 100sec timer
                // startMatchTimer(ctx);    // TODO
                // setThresholds(10, 15);
                
                setZones(0);
                // debugPrintf(DBG_FSM, "FSM -> DISPATCH_CMD");
                ctx.currentAction = Robot::DISPATCH_CMD;
            // }
            // startSwitch.waitForStart();

            break;


        case Robot::DISPATCH_CMD:
            // 1. Empty Queue
            if (!ctx.commandQueue) {break;}

            // 2. Get next command
            if (xQueueReceive(ctx.commandQueue, &ctx.currentCommand, 0) != pdTRUE) {
                if (ctx.queueWasRunning) {
                    ctx.queueWasRunning = false;
                    Serial.println("[FSM] Queue terminee");
                    bleBridge.sendLog("[EVENT] QUEUE_DONE");
                }
                // ctx.currentAction = Robot::IDLE;
                break;
            }

            ctx.queueWasRunning = true;

            setZones(0);
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

                // Controle electroaimant
                case CommandType::RelaisOn:
                    relais_on();
                    ctx.currentAction = Robot::DISPATCH_CMD;
                    break;

                case CommandType::RelaisOff:
                    relais_off();
                    ctx.currentAction = Robot::DISPATCH_CMD;
                    break;

                case CommandType::DeployServo:
                    bras_deployer();
                    break;
                case CommandType::RetractServo:
                    bras_retracter();
                    break;

                case CommandType::ClearQueue:
                    clearCommandQueue(ctx.commandQueue);
                    break;

                case CommandType::SetZones:
                    setZones((uint8_t)ctx.currentCommand.value);
                    ctx.currentAction = Robot::DISPATCH_CMD;
                    break;
                
                case CommandType::SetThresholds:
                    setThresholds(  (uint8_t)ctx.currentCommand.speed,
                                    (uint8_t)ctx.currentCommand.waitMs);
                    ctx.currentAction = Robot::DISPATCH_CMD;
                    break;

                default: ctx.currentAction = Robot::DISPATCH_CMD; break;
            }
            break;



        case Robot::IDLE:
            break;

        case Robot::EXEC:
            if (motion.update()) {

                // Forward detection for timeout go backwards and do next task
                if (motion.blockedForward()) {
                    motion.clearBlockedForward();

                    // short recovery back
                    driveBackward(100.0f, 120);
                    ctx.currentAction = Robot::EXEC;
                    break;
                }

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

        case Robot::EMERGENCY_STOP_US:
            motion.abort();

            // Obstacle not there, resume motion
            if (digitalRead(STOP_PIN) == LOW) {
                emergencyStopUS = false;
                ctx.currentAction = Robot::DISPATCH_CMD;
                // Turn off builtin led
                #ifdef LED_BUILTIN
                    digitalWrite(LED_BUILTIN, LOW);
                #endif
            }

            // Forward blocked for 8s, move backwards to clear
            if (ctx.pendingBlockedRecovery) {
        if (millis() - ctx.blockedRecoverySinceMs >= 8000UL) {
            ctx.pendingBlockedRecovery = false;
            driveBackward(100.0f, 120);
            ctx.currentAction = Robot::EXEC;
        }
        break;
    }



            break;

        case Robot::TIMER_END:
        {
            motion.abort();
            ctx.matchActive = false;
            static unsigned long lastStatePrintMs = 0;
            if (millis() - lastStatePrintMs >= 2000) {
                bleSerial.println("[FSM] TIMER_END");
                lastStatePrintMs = millis();
            }
            
            break;
        }

        default:
            break;
    }
}

