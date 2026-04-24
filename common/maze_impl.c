#include <math.h>
#include <stdint.h>

#include "labirynt_impl.h"

static void (*s_print)(const char *fmt, ...);

typedef enum {
	STATE_DECIDE = 0,
	STATE_TURNING,
	STATE_MOVING
} state_t;

typedef enum {
	TURN_RIGHT = 0,
	TURN_LEFT,
	TURN_BACK,
	TURN_NONE
} turn_cmd_t;

static state_t s_state = STATE_DECIDE;
static float s_turn_target_yaw = 0.0f;
static float s_move_start_left = 0.0f;
static float s_move_start_right = 0.0f;

static float normalize_angle(float a) {
	while (a > (float)M_PI)
		a -= (float)(2.0f * (float)M_PI);
	while (a < (float)-M_PI)
		a += (float)(2.0f * (float)M_PI);
	return a;
}

static float angle_diff(float target, float current) {
	return normalize_angle(target - current);
}

static float clampf(float x, float lo, float hi) {
	return (x < lo) ? lo : (x > hi) ? hi
	                                : x;
}

static void set_wheels(labirynt_loop_output *out, float left, float right) {
	out->wheel_velocities[0] = left;
	out->wheel_velocities[1] = right;
}

static void start_turn(float current_yaw, turn_cmd_t cmd) {
	float delta = 0.0f;
	switch (cmd) {
	case TURN_RIGHT:
		delta = -(float)M_PI_2;
		break;
	case TURN_LEFT:
		delta = +(float)M_PI_2;
		break;
	case TURN_BACK:
		delta = +(float)M_PI;
		break;
	default:
		delta = 0.0f;
		break;
	}
	s_turn_target_yaw = normalize_angle(current_yaw + delta);
	s_state = STATE_TURNING;
}

static void start_move(labirynt_loop_input in) {
	s_move_start_left = (float)in.encoders_angle[0];
	s_move_start_right = (float)in.encoders_angle[1];
	s_state = STATE_MOVING;
}

void labirynt_setup(labirynt_setup_input in, labirynt_setup_output *out) {
	(void)out;
	s_print = in.fn_print;
	s_state = STATE_DECIDE;
}

void labirynt_loop(labirynt_loop_input in, labirynt_loop_output *out) {
	// Tunables
	const float wall_mm = 100.0f;  // threshold for "wall present"
	const float turn_speed = 3.0f; // rad/s wheel velocity (Webots motor velocity)
	const float fwd_speed = 4.0f;
	const float yaw_eps = 0.03f; // ~1.7 deg

	const float move_balance_k = 1.0f;
	const float center_k = 0.05f;

	const float cell_dist_rad = 11.f;     // encoder radians to move one cell
	const float target_side_dist = 75.0f; // desired distance from walls
	const float front_dist_drive_more = 160.0f;

	// Sensors
	const float right = (float)in.lidar[3].distance;
	const float front = ((float)in.lidar[0].distance + (float)in.lidar[1].distance) / 2.0f;
	const float left = (float)in.lidar[2].distance;
	const float yaw = in.integrated_yaw;

	const int right_open = (right > wall_mm);
	const int left_open = (left > wall_mm);
	const int front_open = (front > wall_mm);

	// printf("L: %.1f  F: %.1f  R: %.1f  Yaw: %.2f\n", left, front, right, yaw);

	switch (s_state) {
	case STATE_DECIDE: {
		// prefer: turn right, else go straight, else turn left, else turn back.
		if (right_open) {
			start_turn(yaw, TURN_RIGHT);
		} else if (front_open) {
			start_move(in);
		} else {
			if (left_open)
				start_turn(yaw, TURN_LEFT);
			else
				start_turn(yaw, TURN_BACK);
		}
		set_wheels(out, 0.0f, 0.0f);
		break;
	}

	case STATE_TURNING: {
		float e = angle_diff(s_turn_target_yaw, yaw);
		if (fabsf(e) < yaw_eps) {
			// stop, then move one cell forward
			start_move(in);
			set_wheels(out, 0.0f, 0.0f);
			break;
		}

		float dir = (e > 0.0f) ? 1.0f : -1.0f;
		float v = clampf(fabsf(e) * 2.0f, 0.8f, 1.0f) * turn_speed;
		set_wheels(out, -dir * v, +dir * v);
		break;
	}

	case STATE_MOVING: {
		float dl = (float)in.encoders_angle[0] - s_move_start_left;
		float dr = (float)in.encoders_angle[1] - s_move_start_right;
		float d = 0.5f * (dl + dr);

		if ((d >= cell_dist_rad && front > front_dist_drive_more) || (front < target_side_dist)) {
			// printf(d >= cell_dist_rad ? "Reached cell target\n" : "Obstacle ahead\n");
			s_state = STATE_DECIDE;
			set_wheels(out, 0.0f, 0.0f);
			break;
		}

		// Simple balance correction to reduce drift
		float balance = (dl - dr) * move_balance_k;

		// Center using left and right walls when they exist
		if (!left_open && !right_open) {
			balance += (left - right) * center_k;
		} else if (!left_open) {
			balance += (left - target_side_dist) * center_k;
		} else if (!right_open) {
			balance += (target_side_dist - right) * center_k;
		}

		balance = clampf(balance, -1.0f, 1.0f);
		set_wheels(out, fwd_speed - balance, fwd_speed + balance);
		break;
	}

	default:
		s_state = STATE_DECIDE;
		set_wheels(out, 0.0f, 0.0f);
		break;
	}
}
