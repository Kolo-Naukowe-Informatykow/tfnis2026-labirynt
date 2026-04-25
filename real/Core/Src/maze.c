#include "maze.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "imu.h"
#include "labirynt_impl.h"
#include "logging.h"
#include "motors.h"
#include "ranging.h"
#include "task.h"

static TickType_t xLastWakeTime;

static void wait(uint64_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
	xLastWakeTime = xTaskGetTickCount();
}

void maze_exec() {
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
	const float tick_delta_seconds = 0.01f;

	labirynt_setup_input setup_in = {0};
	setup_in.fn_print = print;
	setup_in.fn_imu_calibrate_async = imu_calibrate_async;
	setup_in.fn_wait = wait;

	labirynt_setup_output setup_out;
	labirynt_setup(setup_in, &setup_out);

	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		labirynt_loop_input loop_in = {0};
		loop_in.battery_volts = battvolts;
		loop_in.integrated_yaw = imu_integrated_angle_z;
		loop_in.encoders_angle[0] = encoders_angle[0];
		loop_in.encoders_angle[1] = encoders_angle[1];
		loop_in.dt = tick_delta_seconds;
		taskENTER_CRITICAL();
		loop_in.encoders_angle[0] = encoders_angle[0];
		loop_in.encoders_angle[1] = encoders_angle[1];
		taskEXIT_CRITICAL();
		taskENTER_CRITICAL();
		for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
			loop_in.lidar[i].distance = tof_measurements[i].distance_mm;
			loop_in.lidar[i].status = tof_measurements[i].status;
		}
		taskEXIT_CRITICAL();

		labirynt_loop_output loop_out;
		labirynt_loop(loop_in, &loop_out);
		for (int i = 0; i < 2; i++) {
			target_wheel_velocities[i] = loop_out.wheel_velocities[i];
		}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
