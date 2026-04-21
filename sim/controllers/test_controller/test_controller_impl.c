#include "common/labirynt_impl.h"

// Provided by common/maze_impl.c (maze solving algorithm implementation)
extern void maze_setup(labirynt_setup_input in, labirynt_setup_output *out);
extern void maze_loop(labirynt_loop_input in, labirynt_loop_output *out);

void (*fn_print)(const char *fmt, ...);

void labirynt_setup(labirynt_setup_input in, labirynt_setup_output *out) {
  fn_print = in.fn_print;
  maze_setup(in, out);
}

void labirynt_loop(labirynt_loop_input in, labirynt_loop_output *out) {
  maze_loop(in, out);
}
