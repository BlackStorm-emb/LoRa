/*
 * Battery.c
 *
 *  Created on: Oct 3, 2021
 *      Author: Тлехас Алий
 */

#include "Battery.h"

extern ADC_HandleTypeDef hadc;

const float R1 = 10000.0;
const float R2 = 10000.0;

const float divider = R2/(R1 + R2);
const float Vref = 3.3;

const float bat_levels[] = {4.0, 3.8, 3.6, 3.4};
const uint8_t levelsCount = sizeof(bat_levels) / sizeof(float);

const uint16_t num_counts = 10;

const float _eps = 0.01;

float getBatVoltage(void) {
	uint32_t sumRawValue = 0;
	uint32_t midRawValue = 0;

	for (uint8_t i = 0; i < num_counts; i++) {
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		sumRawValue += HAL_ADC_GetValue(&hadc);
	}

	midRawValue = sumRawValue / num_counts;

	return (float)(midRawValue * Vref / 4096.0) / divider;
}

uint8_t getBatLevel(void) {
	float curren_voltage = getBatVoltage();
	uint8_t level = 0;

	while (level < levelsCount) {
		//if (fabs(curren_voltage - bat_levels[level]) <= _eps) break;
		if (curren_voltage < bat_levels[3 - level]) break;
		level++;
	}

	return level;
}
