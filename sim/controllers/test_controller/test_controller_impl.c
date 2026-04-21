#include "common/labirynt_impl.h"

void (*fn_print)(const char *fmt, ...);

void labirynt_setup(labirynt_setup_input in, labirynt_setup_output *out) {
	fn_print = in.fn_print;
}

void labirynt_loop(labirynt_loop_input in, labirynt_loop_output *out) {
	out->wheel_velocities[0] = 1.0;
	out->wheel_velocities[1] = 1.0;
	fn_print("%u\n", in.lidar[0].distance);
}
