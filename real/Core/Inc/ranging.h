#ifndef RANGING_H
#define RANGING_H

#include <stdbool.h>
#include <stdint.h>

#define TOF_SENSOR_COUNT 6

typedef struct {
	uint32_t distance_mm;
	uint32_t status;
} TOF_Measurement;

extern TOF_Measurement tof_measurements[TOF_SENSOR_COUNT];

void tof_set_data_ready(uint8_t sensor_index);

void tof_exec(void);

#endif