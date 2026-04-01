#include "motors.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "inttypes.h"
#include "logging.h"
#include "main.h"
#include "math.h"
#include "task.h"

volatile float desired_wheel_velocities[2] = {0.f, 0.f};

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

typedef struct {
	// theta_est zero
	float omega_est;

	float A;
	float B;
	float dt;
	float v_deadzone;

	float L1;
	float L2;

	double total_theta;

	// for plotting
	float correction;
} MotorObserver;

void observer_init(MotorObserver *obs, float tau, float gain, float deadzone, float dt, float l1, float l2) {
	obs->dt = dt;
	obs->v_deadzone = deadzone;

	obs->A = expf(-dt / tau);

	obs->B = gain * (1.0f - obs->A);

	obs->L1 = l1;
	obs->L2 = l2;

	obs->omega_est = 0.0f;

	obs->total_theta = 0.0;
}

float observer_update(MotorObserver *obs, float v_cmd, float delta_theta_actual) {
	float v_physics = 0.0f;

	if (v_cmd > obs->v_deadzone)
		v_physics = v_cmd - obs->v_deadzone;
	else if (v_cmd < -obs->v_deadzone)
		v_physics = v_cmd + obs->v_deadzone;

	float omega_pred = (obs->A * obs->omega_est) + (obs->B * v_physics);
	float theta_pred = omega_pred * obs->dt;

	obs->correction = delta_theta_actual - theta_pred;

	float theta_corrected = theta_pred + (obs->L1 * obs->correction);
	obs->omega_est = omega_pred + (obs->L2 * obs->correction / obs->dt);

	obs->total_theta += (double)theta_corrected;

	return obs->omega_est;
}

MotorObserver observers[2];

void motors_exec() {
	motors_init();
	const TickType_t xFrequency = pdMS_TO_TICKS(1);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	observer_init(&observers[0], 0.12f, 44.f, 3.0f, 0.001f, 0.6f, 0.3f);
	observer_init(&observers[1], 0.1f, 46.f, 3.0f, 0.001f, 0.6f, 0.3f);
	motors_start();
	print("\r\n\r\n");
	for (;;) {
		float battery_volts = battvolts;
		for (int i = 0; i < 2; i++) {
			uint16_t count = __HAL_TIM_GET_COUNTER(encoders[i].htim);
			encoders[i].delta = count - encoders[i].last_count;
			encoders[i].last_count = count;
			encoders[i].total += encoders[i].delta;
			encoders[i].rads_delta = encoders[i].delta * (2.0f * M_PI) / 271.f; // 271 counts per revolution
		}
		// print("%" PRId16 ", %" PRId16 "\r\n", encoders[0].total, encoders[1].total);
		float estimated_velocities[2];
		for (int i = 0; i < 2; i++) {
			estimated_velocities[i] = observer_update(&observers[i], motors[i].last_volts, encoders[i].rads_delta);
		}
		// TODO PID
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
