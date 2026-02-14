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








// ------------------- Robot API -------------------------------------
void robot_init();

void robot_move_distance(float dist_mm, int speed);
void robot_rotate(float angle_deg, int speed);
void robot_stop();
void robot_rotate_gyro(float target_deg, int pwmMax);

void robot_test();  

void robot_step();