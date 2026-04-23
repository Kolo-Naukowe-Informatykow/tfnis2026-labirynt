#include "motors.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "inttypes.h"
#include "logging.h"
#include "main.h"
#include "math.h"
#include "task.h"

volatile float target_wheel_velocities[2] = {0.f, 0.f};
volatile double encoders_angle[2] = {0.0, 0.0};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

struct {
	TIM_HandleTypeDef *timer;
	struct {
		uint8_t forward;
		uint8_t backward;
	} channel;
	float last_volts;
} motors[2] = {
	{&htim2, {TIM_CHANNEL_1, TIM_CHANNEL_2}, 0.0f},
	{&htim2, {TIM_CHANNEL_3, TIM_CHANNEL_4}, 0.0f}};

struct {
	TIM_HandleTypeDef *htim;
	uint16_t last_count;
	int16_t delta;
	int64_t total;
	float rads_delta;
} encoders[2] = {{&htim3, 0, 0, 0.0f}, {&htim4, 0, 0, 0.0f}};

void motors_init() {
	for (int i = 0; i < 2; i++) {
		HAL_TIM_PWM_Start(motors[i].timer, motors[i].channel.forward);
		HAL_TIM_PWM_Start(motors[i].timer, motors[i].channel.backward);
	}
	for (int i = 0; i < 2; i++) {
		HAL_TIM_Encoder_Start(encoders[i].htim, TIM_CHANNEL_ALL);
	}
}

void motors_start() {
	HAL_GPIO_WritePin(M_NSLEEP_GPIO_Port, M_NSLEEP_Pin, GPIO_PIN_SET);
}

void motors_stop() {
	HAL_GPIO_WritePin(M_NSLEEP_GPIO_Port, M_NSLEEP_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < 2; i++) {
		__HAL_TIM_SET_COMPARE(motors[i].timer, motors[i].channel.forward, 0);
		__HAL_TIM_SET_COMPARE(motors[i].timer, motors[i].channel.backward, 0);
		motors[i].last_volts = 0.f;
	}
}

void motor_set_raw_pwm(enum MOTOR_SIDE side, uint16_t pwm_forward, uint16_t pwm_backward) {
	__HAL_TIM_SET_COMPARE(motors[side].timer, motors[side].channel.forward, pwm_forward);
	__HAL_TIM_SET_COMPARE(motors[side].timer, motors[side].channel.backward, pwm_backward);
}

void motor_set_pwm(enum MOTOR_SIDE side, float pwm) {
	if (pwm < 0) {
		pwm *= -1;
		motor_set_raw_pwm(side, 0, pwm * 1023);
	} else {
		motor_set_raw_pwm(side, pwm * 1023, 0);
	}
}

void set_motor_volts(enum MOTOR_SIDE side, float requsted_volts, float battery_volts) {
	float volts_abs = fabsf(requsted_volts);
	float pwm = volts_abs / battery_volts;
	pwm = fmaxf(0.f, fminf(pwm, 1.f));
	motor_set_pwm(side, requsted_volts < 0 ? -pwm : pwm);
	motors[side].last_volts = requsted_volts;
}

float rotation_setpoints[2] = {0.0f, 0.0f};
float old_rotation_errors[2] = {0.0f, 0.0f};
float rotation_error_sums[2] = {0.0f, 0.0f};
float steps_per_tick[2] = {0.0f, 0.0f};

void motor_pid_tick(enum MOTOR_SIDE side, float battery_volts) {
	float encoder_val = encoders[side].delta;
	float setpoint = rotation_setpoints[side];
	rotation_setpoints[side] -= encoder_val;
	float error = setpoint - encoder_val;

	float kp = 0.06f;
	float kd = 0.2f;
	float ki = 0.002f;

	float p = kp * error;
	float d = kd * (error - old_rotation_errors[side]);
	float i = ki * rotation_error_sums[side];

	old_rotation_errors[side] = error;

	rotation_error_sums[side] += error;
	rotation_error_sums[side] *= 0.9f;

	float input = p + i + d;

	input = fmaxf(-1.0f, fminf(input, 1.0f));
	set_motor_volts(side, input * battery_volts, battery_volts);
}

void motors_exec() {
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
	const float ticks_per_second = 100.0f;
	const float encoder_steps_per_rotation = 271.0f;

	motors_init();
	motors_start();
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		float battery_volts = battvolts;
		if (battery_volts < 3.f) {
			motors_stop();
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
			continue;
		} else {
			motors_start();
		}

		for (int i = 0; i < 2; i++) {
			uint16_t count = __HAL_TIM_GET_COUNTER(encoders[i].htim);
			encoders[i].delta = (int16_t)(count - encoders[i].last_count);
			encoders[i].last_count = count;
			encoders[i].total += encoders[i].delta;

			// Update absolute position (radians) in critical section
			float delta_rads = encoders[i].delta * (2.0f * M_PI) / encoder_steps_per_rotation;
			taskENTER_CRITICAL();
			encoders_angle[i] += delta_rads;
			taskEXIT_CRITICAL();

			float target_rps = target_wheel_velocities[i] / (2.0f * M_PI);
			steps_per_tick[i] = (encoder_steps_per_rotation / ticks_per_second) * target_rps;

			rotation_setpoints[i] += steps_per_tick[i];
			motor_pid_tick(i, battery_volts);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
