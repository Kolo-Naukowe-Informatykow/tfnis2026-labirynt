#include "motors.h"
#include "FreeRTOS.h"
#include "inttypes.h"
#include "logging.h"
#include "main.h"
#include "task.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

enum MOTOR_SIDE {
	MOTOR_LEFT = 0,
	MOTOR_RIGHT = 1
};

struct {
	TIM_HandleTypeDef *timer;
	struct {
		uint8_t forward;
		uint8_t backward;
	} channel;
} motors[2] = {
	{&htim2, {TIM_CHANNEL_1, TIM_CHANNEL_2}},
	{&htim2, {TIM_CHANNEL_3, TIM_CHANNEL_4}}};

struct {
	TIM_HandleTypeDef *htim;
	uint16_t last_count;
	uint16_t delta;
} encoders[2] = {{&htim3, 0, 0}, {&htim4, 0, 0}};

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

void motors_exec() {
	motors_init();
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	// motors_start();
	// motor_set_pwm(MOTOR_LEFT, 1.0);
	// motor_set_pwm(MOTOR_RIGHT, 1.0);
	for (;;) {
		for (int i = 0; i < 2; i++) {
			uint16_t count = __HAL_TIM_GET_COUNTER(encoders[i].htim);
			encoders[i].delta = count - encoders[i].last_count;
			encoders[i].last_count = count;
		}
		// Logging_Print("%" PRId16 ", %" PRId16 "\r\n", encoders[0].delta, encoders[1].delta);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
