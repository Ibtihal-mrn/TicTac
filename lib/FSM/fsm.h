#pragma once
#include <Arduino.h>

enum class Team { TEAM_YELLOW = 0, TEAM_BLUE = 1 };

enum class Robot {
	INIT,
	IDLE,
	DISPATCH_CMD,
	EXEC_MOVE,
	EXEC_ROTATE,
	TASK,
	TUNE_PID,
	TIMER_END,
	EMERGENCY_STOP,
};

static const char* const stateList[] = { // for debug prints ..
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
struct Context {
    Team currentTeam;
	Robot currentAction;
    // RobotCommand currentCommand; on;

    bool matchActive = false;
	unsigned long stateStartMs, matchStartMs, matchDurationMs;

     // Queue of robot commands
    // std::queue<RobotCommand> commandQueue;
    // RobotCommand currentCommand; 
};



// Fsm
void robot_init();
void hardware_init(Context &ctx);
void robot_step(Context &ctx);



// Avec US
void driveForward(float mm, int speed);
void driveBackward(float mm, int speed);
void rotate(float angle, int speed);
