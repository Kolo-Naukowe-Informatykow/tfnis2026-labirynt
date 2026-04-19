#include <stdio.h>
#include <webots/robot.h>

int main() {
	wb_robot_init();

	while (wb_robot_step(32) != -1)
		printf("Hello World!\n");

	wb_robot_cleanup();
	return 0;
}
