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
    // ctx.matchActive = false;
    // ctx.matchDurationMs = MATCH_DURATION_MS;
    // ctx.matchStartMs = 0;
    // debugPrintf(DBG_FSM, "FSM -> INIT");
    
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
    debugPrintf(DBG_FSM, "FSM -> INIT");
    ctx.currentAction = Robot::INIT;

    // Read switch
    bool team = 0;  //TODO read teamswitch
    ctx.currentTeam = Team::TEAM_YELLOW;    // TODO: read TeamSwicth
    // heartbeatTeamName(); // done in main.cpp
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
    RobotCommand rot1{CommandType::Rotate, 90.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot1, 0);
    RobotCommand wait1{CommandType::Wait}; xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand rot2{CommandType::Rotate, -90.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot2, 0);
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand rot3{CommandType::Rotate, 360.0f, 80, 0}; xQueueSendToBack(ctx.commandQueue, &rot3, 0);



}
void testLinearMotion(Context &ctx){
    if (!ctx.commandQueue) return;

    RobotCommand move1{CommandType::MoveForward, 1000.0f, 200, 0};
    xQueueSendToBack(ctx.commandQueue, &move1, 0);

    RobotCommand wait1{CommandType::Wait, 0.0f, 0, 5000};
    xQueueSendToBack(ctx.commandQueue, &wait1, 0);

    RobotCommand move2{CommandType::MoveBackward, 900.0f, 200, 0};
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
    printEncodersVal();
    
}

// Stop Conditions
void US_STOP(Context &ctx){
    if (emergencyStopUS) {
        motion.abort();
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
        ctx.queueWasRunning = false;
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
    if (isMatchTimeEnded(ctx)) { ctx.currentAction = Robot::TIMER_END; }

    // ===================================================
    switch (ctx.currentAction)
    {   
        case Robot::INIT:
            // MOVE COMMANDS
            testRotation(ctx);
            // testLinearMotion(ctx);
            // testServos(ctx);
            // testElectroAimant(ctx);
            ;
            // competitionRoutine(ctx);
            ctx.currentAction = Robot::WAIT_START;
            break;

        case Robot::WAIT_START:
            // ADD LaunchTrigger HERE
            startSwitch.begin();
            if (startSwitch.isInserted()) {
                // start 100sec timer
                startMatchTimer(ctx);    // TODO

                
                // debugPrintf(DBG_FSM, "FSM -> DISPATCH_CMD");
                ctx.currentAction = Robot::DISPATCH_CMD;
            }
            // startSwitch.waitForStart();

            
            break;


        case Robot::DISPATCH_CMD:
            // 1. Empty Queue
            if (!ctx.commandQueue) { break;}

            // 2. Get next command
            if (xQueueReceive(ctx.commandQueue, &ctx.currentCommand, 0) != pdTRUE) {
                if (ctx.queueWasRunning) {
                    ctx.queueWasRunning = false;
                    Serial.println("[FSM] Queue terminee");
                    bleBridge.sendLog("[EVENT] QUEUE_DONE");
                }
                ctx.currentAction = Robot::IDLE;
                break;
            }

            ctx.queueWasRunning = true;


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
                    ctx.queueWasRunning = false;
                    break;


                default: ctx.currentAction = Robot::DISPATCH_CMD; break;
            }
            break;



        case Robot::IDLE:
            // Vérifier si de nouvelles commandes BLE sont arrivées
            if (ctx.commandQueue && uxQueueMessagesWaiting(ctx.commandQueue) > 0) {
                ctx.currentAction = Robot::DISPATCH_CMD;
            }
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

        case Robot::EMERGENCY_STOP_US:
            motion.abort();
            if (digitalRead(STOP_PIN) == LOW) {
                emergencyStopUS = false;
                ctx.currentAction = Robot::DISPATCH_CMD;

                // Turn off builtin led
                #ifdef LED_BUILTIN
                    digitalWrite(LED_BUILTIN, LOW);
                #endif
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

