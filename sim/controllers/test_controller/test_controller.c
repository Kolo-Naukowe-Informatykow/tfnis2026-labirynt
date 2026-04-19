#include <stdarg.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include "common/labirynt_impl.h"

#define TIME_STEP 32
#define NUM_SENSORS 6

void print(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vprintf(fmt, args);
	va_end(args);
}

int main(int argc, char *argv[]) {
	wb_robot_init();

	// Get motor devices
	WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
	WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

	// Set motors to velocity control mode (INFINITY means unbounded position)
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

	// Get and enable distance sensors
	WbDeviceTag ds[NUM_SENSORS];
	const char *ds_names[NUM_SENSORS] = {
		"lf lidar", "ld lidar", "ll lidar",
		"rr lidar", "rd lidar", "rf lidar"};

	for (int i = 0; i < NUM_SENSORS; i++) {
		ds[i] = wb_robot_get_device(ds_names[i]);
		wb_distance_sensor_enable(ds[i], TIME_STEP);
	}

	// Initialize control algorithm
	labirynt_setup_input setup_in;
	setup_in.fn_print = print;
	labirynt_setup_output setup_out;
	labirynt_setup(setup_in, &setup_out);

	// Main control loop
	labirynt_loop_input loop_in;
	labirynt_loop_output loop_out;

	while (wb_robot_step(TIME_STEP) != -1) {
		// Read sensor values
		for (int i = 0; i < NUM_SENSORS; i++) {
			loop_in.lidar[i].distance = (uint32_t)(wb_distance_sensor_get_value(ds[i]) * 1000.0);
		}

		// Call control algorithm
		labirynt_loop(loop_in, &loop_out);

		// Set motor velocities
		wb_motor_set_velocity(left_motor, loop_out.wheel_velocities[0]);
		wb_motor_set_velocity(right_motor, loop_out.wheel_velocities[1]);
	}

	// Cleanup and exit
	wb_robot_cleanup();
	return 0;
}
