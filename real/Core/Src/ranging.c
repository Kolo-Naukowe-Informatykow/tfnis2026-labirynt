#include "ranging.h"
#include "app_freertos.h"

#include "custom_tof_conf.h"
#include "main.h"
#include "vl53l4cd.h"

#include "vl53l4cd_api.h"
#include "vl53l4cd_calibration.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "logging.h"

TOF_Measurement tof_measurements[TOF_SENSOR_COUNT] = {{0, 255}, {0, 255}, {0, 255}, {0, 255}, {0, 255}, {0, 255}};

VL53L4CD_Object_t sensors[TOF_SENSOR_COUNT];

const uint8_t SENSOR_ADDRESSES[TOF_SENSOR_COUNT] = {0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E};

const int16_t tof_calibrations[TOF_SENSOR_COUNT] = {-12, -16, -17, -14, -19, -8};

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} XSHUT_Pin_t;

const XSHUT_Pin_t XSHUT_PINS[TOF_SENSOR_COUNT] = {
	{DIST1_XSHUT_GPIO_Port, DIST1_XSHUT_Pin}, // 0
	{DIST5_XSHUT_GPIO_Port, DIST5_XSHUT_Pin}, // 1
	{DIST3_XSHUT_GPIO_Port, DIST3_XSHUT_Pin}, // 2
	{DIST4_XSHUT_GPIO_Port, DIST4_XSHUT_Pin}, // 3
	{DIST6_XSHUT_GPIO_Port, DIST6_XSHUT_Pin}, // 4
	{DIST2_XSHUT_GPIO_Port, DIST2_XSHUT_Pin}, // 5
};

int32_t get_tick(void) {
	return xTaskGetTickCount();
}

uint8_t VL53L4CD_WaitMs(
	VL53L4CD_Object_t *Dev,
	uint32_t TimeMs) {
	vTaskDelay(pdMS_TO_TICKS(TimeMs));

	return 0;
}

void Init_All_Sensors(void) {
	int32_t ret;

	for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
		HAL_GPIO_WritePin(XSHUT_PINS[i].port, XSHUT_PINS[i].pin, GPIO_PIN_RESET);
	}
	vTaskDelay(pdMS_TO_TICKS(5));

	VL53L4CD_IO_t io_base = {
		.Init = CUSTOM_VL53L4CD_I2C_INIT,
		.DeInit = CUSTOM_VL53L4CD_I2C_DEINIT,
		.Address = VL53L4CD_DEVICE_ADDRESS,
		.WriteReg = CUSTOM_VL53L4CD_I2C_WRITEREG,
		.ReadReg = CUSTOM_VL53L4CD_I2C_READREG,
		.GetTick = get_tick};

	for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
		HAL_GPIO_WritePin(XSHUT_PINS[i].port, XSHUT_PINS[i].pin, GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(5));

		io_base.Address = VL53L4CD_DEVICE_ADDRESS;
		ret = VL53L4CD_RegisterBusIO(&sensors[i], &io_base);
		if (ret != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}

		ret = VL53L4CD_Init(&sensors[i]);
		if (ret != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}

		ret = VL53L4CD_SetAddress(&sensors[i], SENSOR_ADDRESSES[i]);
		if (ret != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}
	}
}

void Start_All_Sensors_IT(void) {
	int32_t ret;
	VL53L4CD_Error ret_err;
	VL53L4CD_ProfileConfig_t profile = {0};
	profile.TimingBudget = 10;
	profile.Frequency = 0;

	VL53L4CD_ITConfig_t it_cfg = {0};
	it_cfg.Criteria = VL53L4CD_IT_DEFAULT;

	for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
		ret_err = VL53L4CD_SetOffset(&sensors[i], tof_calibrations[i]);
		if (ret_err != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}

		ret = VL53L4CD_ConfigProfile(&sensors[i], &profile);
		if (ret != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}

		ret = VL53L4CD_ConfigIT(&sensors[i], &it_cfg);
		if (ret != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}

		ret = VL53L4CD_Start(&sensors[i], VL53L4CD_MODE_ASYNC_CONTINUOUS);
		if (ret != VL53L4CD_OK) {
			vTaskSuspend(NULL);
		}
	}
}

void tof_exec(void) {
	Init_All_Sensors();

	// while (1) {
	// 	print("Calibrating sensors...\r\n");
	// 	for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
	// 		VL53L4CD_Error ret;
	// 		int16_t calib;
	// 		vTaskDelay(pdMS_TO_TICKS(10000));
	// 		print("Calibrating lidar %d...\r\n", i);
	// 		ret = VL53L4CD_CalibrateOffset(&sensors[i], 100, &calib, 10);
	// 		if (ret != VL53L4CD_OK) {
	// 			print("Calibration failed for lidar %d: %d\r\n", i, ret);
	// 		} else {
	// 			print("Calibration successful for lidar %d, offset: %d mm\r\n", i, calib);
	// 		}
	// 	}
	// }

	Start_All_Sensors_IT();

	while (1) {
		uint32_t notified_value = 0;
		xTaskNotifyWait(0x00, 0xFFFFFFFF, &notified_value, portMAX_DELAY);
		VL53L4CD_Result_t result;
		for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
			if (notified_value & (1 << i)) {
				int ret = VL53L4CD_GetDistance(&sensors[i], &result);
				if (ret == VL53L4CD_OK) {
					uint32_t distance_mm = result.ZoneResult[0].Distance[0];
					uint32_t status = result.ZoneResult[0].Status[0];
					xSemaphoreTake(TofMutexHandle, portMAX_DELAY);
					tof_measurements[i].distance_mm = distance_mm;
					tof_measurements[i].status = status;
					xSemaphoreGive(TofMutexHandle);
					// print("%d, %d, %d\r\n", i, distance_mm, status);
				}
			}
		}
	}
}

void tof_set_data_ready(uint8_t sensor_index) {
	if (sensor_index < TOF_SENSOR_COUNT) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(tofTaskHandle, (1 << sensor_index), eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}
