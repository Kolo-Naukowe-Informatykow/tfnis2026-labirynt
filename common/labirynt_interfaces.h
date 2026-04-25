#ifndef LABIRYNT_INTERFACES_H
#define LABIRYNT_INTERFACES_H

#include <stdint.h>

typedef struct {
	void (*fn_print)(const char *fmt, ...);
	void (*fn_imu_calibrate_async)(uint64_t duration_ms);
	void (*fn_wait)(uint64_t ms);
} labirynt_setup_input;

typedef struct {
} labirynt_setup_output;

typedef struct {
	float dt;
	float battery_volts;
	float integrated_yaw;
	double encoders_angle[2];
	struct {
		uint32_t distance;
		uint32_t status;
	} lidar[6];
} labirynt_loop_input;

typedef struct {
	float wheel_velocities[2];
} labirynt_loop_output;

#endif
