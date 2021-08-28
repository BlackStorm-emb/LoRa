/*
 * beeper.c
 *
 *  Created on: Aug 17, 2021
 *      Author: Тлехас Алий
 */
#include <beeper.h>

extern TIM_HandleTypeDef htim2;

void BEEPER_Enable(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void BEEPER_Disable(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void BEEPER_SetFreq(uint16_t freq) {
	TIM2->PSC = (SystemCoreClock / (2 * BUZZER_VOLUME_MAX * freq)) - 1;
}

void BEEPER_SetVolume(uint16_t volume) {
	if(volume > BUZZER_VOLUME_MAX)
			volume = BUZZER_VOLUME_MAX;

		TIM2->CCR1 = volume;
}
