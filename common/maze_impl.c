#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "labirynt_impl.h"

static void (*f_print)(const char *fmt, ...);
static void (*f_imu_calib)(uint64_t ms);
static void (*f_wait)(uint64_t ms);

typedef enum {
	STATE_DECIDE = 0,
	STATE_TURNING,
	STATE_MOVING,
	STATE_IMU_CALIB,
	STATE_FAIL,
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

typedef enum {
	NAV_UP = 0,
	NAV_RIGHT,
	NAV_DOWN,
	NAV_LEFT
} nav_dir_t;

typedef enum {
	CELL_UNVISITED = 0,
	CELL_VISITED = 1,
	CELL_UNREACHABLE = 2
} nav_cell_state_t;

typedef struct {
	uint8_t walls; // bitfield: 1=up, 2=right, 4=down, 8=left
	nav_cell_state_t state;
} nav_cell_t;

typedef enum {
	GOAL,
	EXPLORATION,
	RETURN
} nav_stage_t;

typedef struct {
	int current_x, current_y;
	nav_dir_t current_dir;
	nav_cell_t maze[10][10];
	int exploration_goal_x, exploration_goal_y;
	nav_stage_t stage;
} navigation_state_t;

static navigation_state_t nav_state;

void select_exploration_point(void) {
	int best_x = 0;
	int best_y = 0;
	int min_dist = 9999;

	for (int i = 1; i < 9; i++) {
		for (int j = 1; j < 9; j++) {
			if (nav_state.maze[i][j].state == CELL_UNVISITED && nav_state.maze[i][j].state != CELL_UNREACHABLE) {
				// pick the closest unvisited cell (Manhattan distance)
				int dx = nav_state.current_x - j;
				int dy = nav_state.current_y - i;
				int dist = (dx > 0 ? dx : -dx) + (dy > 0 ? dy : -dy);
				if (dist < min_dist) {
					min_dist = dist;
					best_x = j;
					best_y = i;
				}
			}
		}
	}

	nav_state.exploration_goal_x = best_x;
	nav_state.exploration_goal_y = best_y;
}

void evaluate_nav_stage() {
	switch (nav_state.stage) {
	case GOAL:
		if (nav_state.current_x >= 4 && nav_state.current_x <= 5 && nav_state.current_y >= 4 && nav_state.current_y <= 5) {
			s_state = STATE_IMU_CALIB;
			nav_state.stage = EXPLORATION;
			select_exploration_point();
		}
		break;
	case EXPLORATION:
		if (nav_state.current_x == nav_state.exploration_goal_x && nav_state.current_y == nav_state.exploration_goal_y) {
			select_exploration_point();
			if (nav_state.exploration_goal_x == 0 && nav_state.exploration_goal_y == 0) {
				nav_state.stage = RETURN;
			}
		}
		break;
	case RETURN:
		if (nav_state.current_x == 1 && nav_state.current_y == 1) {
			s_state = STATE_IMU_CALIB;
			nav_state.stage = GOAL;
		}
		break;
	}
}

static void nav_next_cell() {
	switch (nav_state.current_dir) {
	case NAV_UP:
		nav_state.current_y -= 1;
		break;
	case NAV_RIGHT:
		nav_state.current_x += 1;
		break;
	case NAV_DOWN:
		nav_state.current_y += 1;
		break;
	case NAV_LEFT:
		nav_state.current_x -= 1;
		break;
	}
	evaluate_nav_stage();
}

static void nav_mark_walls(int right_open, int front_open, int left_open) {
	int x = nav_state.current_x;
	int y = nav_state.current_y;
	nav_state.maze[y][x].state = CELL_VISITED;
	int up_wall = 0, right_wall = 0, down_wall = 0, left_wall = 0;

	switch (nav_state.current_dir) {
	case NAV_UP:
		up_wall = !front_open;
		right_wall = !right_open;
		left_wall = !left_open;
		break;
	case NAV_RIGHT:
		right_wall = !front_open;
		down_wall = !right_open;
		up_wall = !left_open;
		break;
	case NAV_DOWN:
		down_wall = !front_open;
		left_wall = !right_open;
		right_wall = !left_open;
		break;
	case NAV_LEFT:
		left_wall = !front_open;
		up_wall = !right_open;
		down_wall = !left_open;
		break;
	}

	if (up_wall) {
		nav_state.maze[y][x].walls |= 1;
		nav_state.maze[y - 1][x].walls |= 4;
	} else {
		nav_state.maze[y][x].walls &= ~1;
		nav_state.maze[y - 1][x].walls &= ~4;
	}
	if (right_wall) {
		nav_state.maze[y][x].walls |= 2;
		nav_state.maze[y][x + 1].walls |= 8;
	} else {
		nav_state.maze[y][x].walls &= ~2;
		nav_state.maze[y][x + 1].walls &= ~8;
	}
	if (down_wall) {
		nav_state.maze[y][x].walls |= 4;
		nav_state.maze[y + 1][x].walls |= 1;
	} else {
		nav_state.maze[y][x].walls &= ~4;
		nav_state.maze[y + 1][x].walls &= ~1;
	}
	if (left_wall) {
		nav_state.maze[y][x].walls |= 8;
		nav_state.maze[y][x - 1].walls |= 2;
	} else {
		nav_state.maze[y][x].walls &= ~8;
		nav_state.maze[y][x - 1].walls &= ~2;
	}
}

typedef struct {
	int x, y;
} point_t;

bool floodfill(bool is_goal[10][10], turn_cmd_t *out) {
	uint8_t dist[10][10];
	for (int i = 0; i < 10; i++)
		for (int j = 0; j < 10; j++)
			dist[i][j] = 255;
	point_t q[100];
	int head = 0, tail = 0;

	for (int y = 0; y < 10; y++) {
		for (int x = 0; x < 10; x++) {
			if (is_goal[y][x]) {
				dist[y][x] = 0;
				q[tail++] = (point_t){x, y};
			}
		}
	}

	while (head < tail) {
		point_t p = q[head++];
		int d = dist[p.y][p.x] + 1;
		// up
		if (p.y > 0 && !(nav_state.maze[p.y][p.x].walls & 1) && dist[p.y - 1][p.x] == 255) {
			dist[p.y - 1][p.x] = d;
			q[tail++] = (point_t){p.x, p.y - 1};
		}
		// right
		if (p.x < 9 && !(nav_state.maze[p.y][p.x].walls & 2) && dist[p.y][p.x + 1] == 255) {
			dist[p.y][p.x + 1] = d;
			q[tail++] = (point_t){p.x + 1, p.y};
		}
		// down
		if (p.y < 9 && !(nav_state.maze[p.y][p.x].walls & 4) && dist[p.y + 1][p.x] == 255) {
			dist[p.y + 1][p.x] = d;
			q[tail++] = (point_t){p.x, p.y + 1};
		}
		// left
		if (p.x > 0 && !(nav_state.maze[p.y][p.x].walls & 8) && dist[p.y][p.x - 1] == 255) {
			dist[p.y][p.x - 1] = d;
			q[tail++] = (point_t){p.x - 1, p.y};
		}
	}

	int cx = nav_state.current_x;
	int cy = nav_state.current_y;
	int min_dist = 255;

	int d_up = (cy > 0 && !(nav_state.maze[cy][cx].walls & 1)) ? dist[cy - 1][cx] : 255;
	int d_right = (cx < 9 && !(nav_state.maze[cy][cx].walls & 2)) ? dist[cy][cx + 1] : 255;
	int d_down = (cy < 9 && !(nav_state.maze[cy][cx].walls & 4)) ? dist[cy + 1][cx] : 255;
	int d_left = (cx > 0 && !(nav_state.maze[cy][cx].walls & 8)) ? dist[cy][cx - 1] : 255;

	if (d_up < min_dist)
		min_dist = d_up;
	if (d_right < min_dist)
		min_dist = d_right;
	if (d_down < min_dist)
		min_dist = d_down;
	if (d_left < min_dist)
		min_dist = d_left;

	if (min_dist == 255) {
		uint8_t r_dist[10][10];
		for (int i = 0; i < 10; i++)
			for (int j = 0; j < 10; j++)
				r_dist[i][j] = 255;
		
		int r_head = 0, r_tail = 0;
		r_dist[cy][cx] = 0;
		q[r_tail++] = (point_t){cx, cy};

		while (r_head < r_tail) {
			point_t p = q[r_head++];
			int d = r_dist[p.y][p.x] + 1;
			if (p.y > 0 && !(nav_state.maze[p.y][p.x].walls & 1) && r_dist[p.y - 1][p.x] == 255) {
				r_dist[p.y - 1][p.x] = d;
				q[r_tail++] = (point_t){p.x, p.y - 1};
			}
			if (p.x < 9 && !(nav_state.maze[p.y][p.x].walls & 2) && r_dist[p.y][p.x + 1] == 255) {
				r_dist[p.y][p.x + 1] = d;
				q[r_tail++] = (point_t){p.x + 1, p.y};
			}
			if (p.y < 9 && !(nav_state.maze[p.y][p.x].walls & 4) && r_dist[p.y + 1][p.x] == 255) {
				r_dist[p.y + 1][p.x] = d;
				q[r_tail++] = (point_t){p.x, p.y + 1};
			}
			if (p.x > 0 && !(nav_state.maze[p.y][p.x].walls & 8) && r_dist[p.y][p.x - 1] == 255) {
				r_dist[p.y][p.x - 1] = d;
				q[r_tail++] = (point_t){p.x - 1, p.y};
			}
		}

		for (int y = 0; y < 10; y++) {
			for (int x = 0; x < 10; x++) {
				if (r_dist[y][x] == 255 && nav_state.maze[y][x].state == CELL_UNVISITED) {
					nav_state.maze[y][x].state = CELL_UNREACHABLE;
				}
			}
		}
		return false;
	}

	int dirs[4] = {d_up, d_right, d_down, d_left};
	int front_dir = nav_state.current_dir;
	int right_dir = (nav_state.current_dir + 1) % 4;
	int back_dir = (nav_state.current_dir + 2) % 4;
	int left_dir = (nav_state.current_dir + 3) % 4;

	if (dirs[front_dir] == min_dist)
		*out = TURN_NONE;
	else if (dirs[right_dir] == min_dist)
		*out = TURN_RIGHT;
	else if (dirs[left_dir] == min_dist)
		*out = TURN_LEFT;
	else if (dirs[back_dir] == min_dist)
		*out = TURN_BACK;
	else
		*out = TURN_NONE;

	return true;
}

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
		switch (nav_state.current_dir) {
		case NAV_UP:
			nav_state.current_dir = NAV_RIGHT;
			break;
		case NAV_RIGHT:
			nav_state.current_dir = NAV_DOWN;
			break;
		case NAV_DOWN:
			nav_state.current_dir = NAV_LEFT;
			break;
		case NAV_LEFT:
			nav_state.current_dir = NAV_UP;
			break;
		}
		break;
	case TURN_LEFT:
		delta = +(float)M_PI_2;
		switch (nav_state.current_dir) {
		case NAV_UP:
			nav_state.current_dir = NAV_LEFT;
			break;
		case NAV_RIGHT:
			nav_state.current_dir = NAV_UP;
			break;
		case NAV_DOWN:
			nav_state.current_dir = NAV_RIGHT;
			break;
		case NAV_LEFT:
			nav_state.current_dir = NAV_DOWN;
			break;
		}
		break;
	case TURN_BACK:
		delta = +(float)M_PI;
		switch (nav_state.current_dir) {
		case NAV_UP:
			nav_state.current_dir = NAV_DOWN;
			break;
		case NAV_RIGHT:
			nav_state.current_dir = NAV_LEFT;
			break;
		case NAV_DOWN:
			nav_state.current_dir = NAV_UP;
			break;
		case NAV_LEFT:
			nav_state.current_dir = NAV_RIGHT;
			break;
		}
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
	f_imu_calib = in.fn_imu_calibrate_async;
	f_print = in.fn_print;
	f_wait = in.fn_wait;
	s_state = STATE_DECIDE;

	nav_state.current_x = 1;
	nav_state.current_y = 1;
	nav_state.current_dir = NAV_RIGHT;
	nav_state.exploration_goal_x = 0;
	nav_state.exploration_goal_y = 0;
	nav_state.stage = GOAL;
	for (int y = 0; y < 10; y++) {
		for (int x = 0; x < 10; x++) {
			if (y == 0 || y == 9 || x == 0 || x == 9) {
				nav_state.maze[y][x].walls = 15;
				nav_state.maze[y][x].state = CELL_UNREACHABLE;
			} else {
				nav_state.maze[y][x].walls = 0;
				nav_state.maze[y][x].state = CELL_UNVISITED;
				if (y == 1)
					nav_state.maze[y][x].walls |= 1;
				if (x == 8)
					nav_state.maze[y][x].walls |= 2;
				if (y == 8)
					nav_state.maze[y][x].walls |= 4;
				if (x == 1)
					nav_state.maze[y][x].walls |= 8;
			}
		}
	}

	nav_state.maze[1][1].state = CELL_VISITED;
}

void labirynt_loop(labirynt_loop_input in, labirynt_loop_output *out) {
	// Tunables
	const float wall_mm = 120.0f;  // threshold for "wall present"
	const float turn_speed = 3.0f; // rad/s wheel velocity (Webots motor velocity)
	const float fwd_speed = 4.0f;
	const float yaw_eps = 0.03f; // ~1.7 deg

	const float move_balance_k = 1.0f;
	const float center_k = 0.05f;

	const float cell_dist_rad = 11.f;     // encoder radians to move one cell
	const float target_side_dist = 75.0f; // desired distance from walls

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
		set_wheels(out, 0.0f, 0.0f);
		nav_mark_walls(right_open, front_open, left_open);
		bool goals[10][10] = {0};
		switch (nav_state.stage) {
		case GOAL:
			goals[4][4] = true;
			goals[4][5] = true;
			goals[5][4] = true;
			goals[5][5] = true;
			break;
		case EXPLORATION:
			if (nav_state.exploration_goal_x >= 0 && nav_state.exploration_goal_y >= 0) {
				goals[nav_state.exploration_goal_y][nav_state.exploration_goal_x] = true;
			} else {
				goals[1][1] = true;
			}
			break;
		case RETURN:
			goals[1][1] = true;
			break;
		}
		turn_cmd_t cmd;
		// for (int y = 0; y < 10; y++) {
		// 	for (int x = 0; x < 10; x++) {
		// 		if (goals[y][x]) {
		// 			printf("G ");
		// 		} else if (x == nav_state.current_x && y == nav_state.current_y) {
		// 			printf("R ");
		// 		} else if (nav_state.maze[y][x].state == CELL_UNREACHABLE) {
		// 			printf("X ");
		// 		} else if (nav_state.maze[y][x].state == CELL_UNVISITED) {
		// 			printf(". ");
		// 		} else {
		// 			printf("  ");
		// 		}
		// 	}
		// 	printf("\n");
		// }
		if (!floodfill(goals, &cmd)) {
			if (nav_state.stage == EXPLORATION) {
				select_exploration_point();
				if (nav_state.exploration_goal_x == 0 && nav_state.exploration_goal_y == 0) {
					nav_state.stage = RETURN;
				}
				break;
			} else {
				s_state = STATE_FAIL;
			}
			break;
		}
		start_turn(yaw, cmd);
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

		if ((d >= cell_dist_rad && front > wall_mm) || (front < target_side_dist)) {
			// printf(d >= cell_dist_rad ? "Reached cell target\n" : "Obstacle ahead\n");
			set_wheels(out, 0.0f, 0.0f);
			s_state = STATE_DECIDE;
			nav_next_cell();
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
	case STATE_IMU_CALIB:
		f_imu_calib(2000);
		f_wait(2500);
		s_state = STATE_DECIDE;
		break;

	case STATE_FAIL:
		set_wheels(out, 0.0f, 0.0f);
		break;

	default:
		s_state = STATE_DECIDE;
		set_wheels(out, 0.0f, 0.0f);
		break;
	}
}
