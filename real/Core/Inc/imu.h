#ifndef IMU_H
#define IMU_H

extern volatile float imu_angular_velocity_z;  // Z-axis (vertical) angular velocity

void imu_set_data_ready(void);
void imu_exec(void);

#endif
