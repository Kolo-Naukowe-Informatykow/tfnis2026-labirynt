#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "common/labirynt_impl.h"

#define TIME_STEP 32
#define NUM_SENSORS 6

static void wait(uint64_t ms) {
  float time_step = wb_robot_get_basic_time_step();
  double start_time = wb_robot_get_time();
  while (wb_robot_step(time_step) != -1) {
    if (wb_robot_get_time() >= start_time + (double)ms / 1000.0)
      break;
  }
}

static void print(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
}

static void imu_calibrate_async(uint64_t duration_ms) {
  // No calibration needed in simulation
}

static float normalize_angle(float a) {
  while (a > (float)M_PI)
    a -= (float)(2.0f * (float)M_PI);
  while (a < (float)-M_PI)
    a += (float)(2.0f * (float)M_PI);
  return a;
}

int main(int argc, char *argv[]) {
  wb_robot_init();

  // Motors (velocity-controlled)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Wheel encoders
  WbDeviceTag left_enc = wb_robot_get_device("left wheel encoder");
  WbDeviceTag right_enc = wb_robot_get_device("right wheel encoder");
  wb_position_sensor_enable(left_enc, TIME_STEP);
  wb_position_sensor_enable(right_enc, TIME_STEP);

  // Lidar distance sensors
  WbDeviceTag ds[NUM_SENSORS];
  const char *ds_names[NUM_SENSORS] = {"lf lidar", "ld lidar", "ll lidar", "rr lidar", "rd lidar", "rf lidar"};
  for (int i = 0; i < NUM_SENSORS; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

  // IMU (for yaw around vertical axis)
  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu, TIME_STEP);
  int yaw_offset_initialized = 0;
  float yaw_offset = 0.0f;

  // Initialize control algorithm
  labirynt_setup_input setup_in = {0};
  setup_in.fn_print = print;
  setup_in.fn_imu_calibrate_async = imu_calibrate_async;
  setup_in.fn_wait = wait;
  labirynt_setup_output setup_out;
  labirynt_setup(setup_in, &setup_out);

  // Main loop
  labirynt_loop_input loop_in = {0};
  labirynt_loop_output loop_out = {0};

  while (wb_robot_step(TIME_STEP) != -1) {
    // Constant battery voltage in simulation
    loop_in.battery_volts = 7.4f;

    // Encoders: radians
    loop_in.encoders_angle[0] = wb_position_sensor_get_value(left_enc);
    loop_in.encoders_angle[1] = wb_position_sensor_get_value(right_enc);

    // Lidars: meters -> millimeters; status: 0=OK (sim)
    for (int i = 0; i < NUM_SENSORS; i++) {
      loop_in.lidar[i].distance = (uint32_t)(wb_distance_sensor_get_value(ds[i]) * 1000.0);
      loop_in.lidar[i].status = 0;
    }

    // integrated_yaw: vertical axis rotation with initial offset so that start direction is 0
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    float yaw = (float)rpy[2];
    if (!yaw_offset_initialized) {
      yaw_offset = yaw;
      yaw_offset_initialized = 1;
    }
    loop_in.integrated_yaw = normalize_angle(yaw - yaw_offset);

    labirynt_loop(loop_in, &loop_out);

    wb_motor_set_velocity(left_motor, loop_out.wheel_velocities[0]);
    wb_motor_set_velocity(right_motor, loop_out.wheel_velocities[1]);
  }

  wb_robot_cleanup();
  return 0;
}
