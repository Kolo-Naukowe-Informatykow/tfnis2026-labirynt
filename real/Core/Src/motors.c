#include "motors.h"
#include "FreeRTOS.h"
#include "battery.h"
#include "inttypes.h"
#include "logging.h"
#include "main.h"
#include "math.h"
#include "task.h"

volatile float target_wheel_velocities[2] = {0.f, 0.f};

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
	float dist_est;

	float A;
	float B;
	float dt;
	float v_deadzone;

	float L1;
	float L2;
	float L3;

	double total_theta;

	// for plotting
	float correction;
} MotorObserver;

void observer_init(MotorObserver *obs, float tau, float gain, float deadzone, float dt, float l1, float l2, float l3) {
	obs->dt = dt;
	obs->v_deadzone = deadzone;

	obs->A = expf(-dt / tau);

	obs->B = gain * (1.0f - obs->A);

	obs->L1 = l1;
	obs->L2 = l2;
	obs->L3 = l3;

	obs->omega_est = 0.0f;
	obs->dist_est = 0.0f;

	obs->total_theta = 0.0;
}

float observer_update(MotorObserver *obs, float v_cmd, float delta_theta_actual) {
	float v_physics = 0.0f;

	if (v_cmd > obs->v_deadzone)
		v_physics = v_cmd - obs->v_deadzone;
	else if (v_cmd < -obs->v_deadzone)
		v_physics = v_cmd + obs->v_deadzone;

	float omega_pred = (obs->A * obs->omega_est) + (obs->B * v_physics) - obs->dist_est;
	float theta_pred = omega_pred * obs->dt;

	obs->correction = delta_theta_actual - theta_pred;

	float theta_corrected = theta_pred + (obs->L1 * obs->correction);
	obs->omega_est = omega_pred + (obs->L2 * obs->correction / obs->dt);

	obs->dist_est -= obs->L3 * obs->correction;

	obs->total_theta += (double)theta_corrected;
	return obs->omega_est;
}

MotorObserver observers[2];

typedef struct {
	float Kp;
	float Ki;
	float Kf;
	float v_deadzone;
	float out_max;
	float dt;
	float error_sum;
} MotorController;

void controller_init(MotorController *ctrl, float tau, float gain, float deadzone, float bandwidth_rad_s, float max_out, float dt) {
	ctrl->Kp = (tau * bandwidth_rad_s) / gain;
	ctrl->Ki = bandwidth_rad_s / gain;
	ctrl->Kf = 1.0f / gain;
	ctrl->v_deadzone = deadzone;
	ctrl->out_max = max_out;
	ctrl->dt = dt;
	ctrl->error_sum = 0.0f;

	if (ctrl->Kp < 1e-6f) {
		ctrl->Kp = 1e-6f;
	}
}

float controller_update(MotorController *ctrl, float target_omega, float actual_omega) {
	float error = target_omega - actual_omega;

	float p_out = ctrl->Kp * error;
	float i_out = ctrl->Ki * ctrl->error_sum;
	float ff_out = target_omega * ctrl->Kf;

	float v_total = p_out + i_out + ff_out;

	if (target_omega > 1e-3f) {
		v_total += ctrl->v_deadzone;
	} else if (target_omega < -1e-3f) {
		v_total -= ctrl->v_deadzone;
	}

	float v_out = v_total;
	if (v_out > ctrl->out_max)
		v_out = ctrl->out_max;
	else if (v_out < -ctrl->out_max)
		v_out = -ctrl->out_max;

	float windup_correction = v_out - v_total;
	ctrl->error_sum += (error * ctrl->dt) + (windup_correction * ctrl->dt / ctrl->Kp);

	if (target_omega == 0.0f && (actual_omega < 0.1f && actual_omega > -0.1f)) {
		v_out = 0.0f;
		ctrl->error_sum = 0.0f;
	}

	return v_out;
}

void controller_reset(MotorController *ctrl) {
	ctrl->error_sum = 0.0f;
}

MotorController controllers[2];

void motors_exec() {
	const TickType_t xFrequency = pdMS_TO_TICKS(1);
	const float tick_delta_seconds = 0.001f;
	motors_init();
	observer_init(&observers[0], 0.1f, 44.f, 2.8f, tick_delta_seconds, 0.8f, 0.01f, 0.1f);
	observer_init(&observers[1], 0.1f, 45.f, 2.9f, tick_delta_seconds, 0.8f, 0.01f, 0.1f);
	controller_init(&controllers[0], 0.1f, 44.f, 2.8f, 100.f, 6.f, tick_delta_seconds);
	controller_init(&controllers[1], 0.1f, 45.f, 2.9f, 100.f, 6.f, tick_delta_seconds);
	motors_start();
	TickType_t xLastWakeTime = xTaskGetTickCount();
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
		// {
		// 	int i = 1;
		// 	print("%f, %f, %f\r\n", encoders[i].rads_delta / tick_delta_seconds, estimated_velocities[i], observers[i].correction);
		// 	set_motor_volts(i, target_wheel_velocities[i], battery_volts);
		// }
		float commanded_volts[2];
		for (int i = 0; i < 2; i++) {
			commanded_volts[i] = controller_update(&controllers[i], target_wheel_velocities[i], estimated_velocities[i]);
			set_motor_volts(i, commanded_volts[i], battery_volts);
		}
		// {
		// 	int i = 1;
		// 	print("%f, %f, %f\r\n", target_wheel_velocities[i], estimated_velocities[i], commanded_volts[i]);
		// }
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
