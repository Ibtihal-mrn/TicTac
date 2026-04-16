#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
// #include <queue>

enum class Team { TEAM_YELLOW = 0, TEAM_BLUE = 1 };

enum class Robot {
	INIT,             // Hardware init
	WAIT_START,		  // Launch trigger wait
	IDLE,			  // Wait for commands
	DISPATCH_CMD,     // Send to Command
	EXEC,	          // Exec Commands
	EXEC_WAIT,		  // Wait ms
    EXEC_SERVO,
	EMERGENCY_STOP_US,   // Obstacle Detected (Hardware US)
	
	// EXEC_MOVE,
	// EXEC_ROTATE,
	// TASK,
	// TUNE_PID,
	TIMER_END,
	
};

static const char* const stateList[] = {
    "INIT",
    "WAIT_START",
    "IDLE",
    "DISPATCH_CMD",
    "EXEC",
    "EXEC_WAIT",
    "EMERGENCY_STOP_US",
    "TIMER_END"
};



// -----ROBOT Commands ---------
enum class CommandType {
    MoveForward,
    MoveBackward,
    Rotate,
    Wait,
    DeployServo,
    RetractServo,
    RelaisOn,
    RelaisOff,

    //
    Ping,
    ClearQueue,
    // SetDistancePID,
    // SetAnglePID
};

struct RobotCommand {
    CommandType type;
    float value = 0.0f;
    int speed = 0;
    unsigned long waitMs = 0;

    RobotCommand() = default;
    RobotCommand(CommandType t, float v = 0.0f, int s = 0, unsigned long w = 0)
        : type(t), value(v), speed(s), waitMs(w) {}
};


struct Context {
    Team currentTeam = Team::TEAM_YELLOW;
    Robot currentAction = Robot::INIT;

    bool matchActive = false;
    unsigned long stateStartMs = 0;
    unsigned long matchStartMs = 0;
    unsigned long matchDurationMs = 0;
    unsigned long waitEndMs = 0;

     // Queue of robot commands
    QueueHandle_t commandQueue = nullptr;
    RobotCommand currentCommand{}; // Optionally keep a pointer to the current command
};




// Fsm
void fsm_init(Context& ctx, QueueHandle_t cmdQueue);
void hardware_init(Context &ctx);
void robot_step(Context &ctx);



// Avec US
void driveForward(float mm, int speed);
void driveBackward(float mm, int speed);
void rotate(float angle, int speed);
