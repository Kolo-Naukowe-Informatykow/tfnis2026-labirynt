#ifndef MOTORS_H
#define MOTORS_H

extern volatile float target_wheel_velocities[2];

enum MOTOR_SIDE {
	MOTOR_LEFT = 0,
	MOTOR_RIGHT = 1
};

void motors_exec(void);
void motors_start();
void motors_stop();

#endif