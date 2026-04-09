#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32h5xx_hal.h"

#include "app_freertos.h"
#include "imu.h"

#include "custom_mems_conf.h"
#include "lsm6dsl.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "logging.h"

extern TIM_HandleTypeDef htim6;

volatile float imu_angular_velocity_z = 0.0f;
volatile float imu_integrated_angle_z = 0.0f;

static LSM6DSL_Object_t imu_sensor;

static const float imu_radians_multiplier_250 = 0.000152716f;
static int32_t imu_z_gyro_bias = 0;

int32_t imu_get_tick(void) {
	return xTaskGetTickCount();
}

void imu_delay_ms(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

int32_t imu_spi_write(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len) {
	int32_t ret = 0;
	uint8_t cmd = (uint8_t)(reg & 0x7F);

	HAL_GPIO_WritePin(CUSTOM_LSM6DSL_0_CS_PORT, CUSTOM_LSM6DSL_0_CS_PIN, GPIO_PIN_RESET);

	ret = CUSTOM_LSM6DSL_0_SPI_Send(&cmd, 1);

	if (ret == 0) {
		ret = CUSTOM_LSM6DSL_0_SPI_Send(data, len);
	}

	HAL_GPIO_WritePin(CUSTOM_LSM6DSL_0_CS_PORT, CUSTOM_LSM6DSL_0_CS_PIN, GPIO_PIN_SET);

	return ret;
}

int32_t imu_spi_read(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len) {
	int32_t ret = 0;
	uint8_t cmd = (uint8_t)(reg & 0x7F) | 0x80;

	HAL_GPIO_WritePin(CUSTOM_LSM6DSL_0_CS_PORT, CUSTOM_LSM6DSL_0_CS_PIN, GPIO_PIN_RESET);

	ret = CUSTOM_LSM6DSL_0_SPI_Send(&cmd, 1);

	if (ret == 0) {
		ret = CUSTOM_LSM6DSL_0_SPI_Recv(data, len);
	}

	HAL_GPIO_WritePin(CUSTOM_LSM6DSL_0_CS_PORT, CUSTOM_LSM6DSL_0_CS_PIN, GPIO_PIN_SET);

	return ret;
}

void Init_IMU_Sensor(void) {
	int32_t ret;

	LSM6DSL_IO_t io_ops = {
		.Init = CUSTOM_LSM6DSL_0_SPI_Init,
		.DeInit = CUSTOM_LSM6DSL_0_SPI_DeInit,
		.WriteReg = imu_spi_write,
		.ReadReg = imu_spi_read,
		.GetTick = imu_get_tick,
		.Delay = imu_delay_ms,
		.BusType = LSM6DSL_SPI_4WIRES_BUS,
		.Address = 0,
	};

	ret = LSM6DSL_RegisterBusIO(&imu_sensor, &io_ops);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	ret = LSM6DSL_Init(&imu_sensor);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	ret = LSM6DSL_GYRO_Enable(&imu_sensor);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	ret = LSM6DSL_GYRO_SetOutputDataRate(&imu_sensor, 104.0f);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	ret = LSM6DSL_GYRO_SetFullScale(&imu_sensor, 250);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	ret = lsm6dsl_gy_power_mode_set(&imu_sensor.Ctx, LSM6DSL_GY_HIGH_PERFORMANCE);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	ret = lsm6dsl_gy_band_pass_set(&imu_sensor.Ctx, LSM6DSL_HP_DISABLE_LP_STRONG);
	if (ret != LSM6DSL_OK) {
		vTaskSuspend(NULL);
	}

	LSM6DSL_GYRO_Set_INT1_DRDY(&imu_sensor, 1);
}

static volatile bool imu_calibrating = false;
static volatile int64_t imu_calibration_sum = 0;
static volatile int64_t imu_calibration_samples = 0;
static volatile TickType_t imu_calibration_start_tick = 0;
static volatile TickType_t imu_calibration_ticks = 0;

void imu_calibrate_async(uint64_t duration_ms) {
	imu_calibrating = false;
	imu_calibration_samples = 0;
	imu_calibration_sum = 0;
	imu_calibration_ticks = pdMS_TO_TICKS(duration_ms);
	imu_calibration_start_tick = xTaskGetTickCount();
	imu_calibrating = true;
}

void imu_exec(void) {
	Init_IMU_Sensor();
	uint16_t imu_last_clock_tick = 0;
	while (1) {
		uint32_t notification_value = 0;
		xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, portMAX_DELAY);
		uint16_t current_microsecond = (uint16_t)notification_value;

		LSM6DSL_Axes_t gyro_data;
		int32_t ret = LSM6DSL_GYRO_GetAxes(&imu_sensor, &gyro_data);

		if (ret == LSM6DSL_OK) {
			if (imu_calibrating) {
				imu_calibration_sum += gyro_data.z;
				imu_calibration_samples++;
				if (xTaskGetTickCount() - imu_calibration_start_tick >= imu_calibration_ticks) {
					imu_z_gyro_bias = imu_calibration_sum / imu_calibration_samples;
					imu_calibrating = false;
				}
			}
			imu_angular_velocity_z = -(gyro_data.z - imu_z_gyro_bias) * imu_radians_multiplier_250;
			// print("%d\r\n", gyro_data.z);
			if (imu_last_clock_tick != 0) {
				uint16_t delta_ticks = current_microsecond - imu_last_clock_tick;
				float delta_time = delta_ticks * 0.000001f;
				imu_integrated_angle_z += imu_angular_velocity_z * delta_time;
			}
			imu_last_clock_tick = current_microsecond;
			// print("%f, %f\r\n", imu_angular_velocity_z, imu_integrated_angle_z);
		}
	}
}

void imu_set_data_ready(uint16_t timer_value) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(imuTaskHandle, timer_value, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
