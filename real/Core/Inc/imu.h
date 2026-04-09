#ifndef IMU_H
#define IMU_H

extern volatile float imu_angular_velocity_z; // Z-axis (vertical) angular velocity
extern volatile float imu_integrated_angle_z; // Integrated angle around Z-axis

void imu_set_data_ready(uint16_t timer_value);
void imu_exec(void);

void imu_calibrate_async(uint64_t duration_ms);

#endif
