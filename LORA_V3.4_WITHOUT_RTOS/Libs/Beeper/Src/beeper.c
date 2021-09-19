/*
 * beeper.c
 *
 *  Created on: Aug 17, 2021
 *      Author: Тлехас Алий
 */
#include <beeper.h>

extern TIM_HandleTypeDef htim2;

volatile       uint32_t          _beep_duration;
volatile       _Bool             _tones_playing;
volatile const Tone_TypeDef     *_tones;
volatile 	   uint16_t			 _volume;

void BEEPER_IT(void) {
	_beep_duration--;
	if (_beep_duration == 0) {
		if (_tones_playing) {
			// Currently playing tones, take next tone
			_tones++;
			if (_tones->frequency == 0 && _tones->duration == 0) {
				// Last tone in sequence
				BEEPER_Disable();
				_tones_playing = 0;
				_tones = NULL;
			} else {
				if (_tones->frequency == 0) {
					// Silence period
					BEEPER_TIM->ARR = SystemCoreClock / (100 * BEEPER_TIM->PSC) - 1;
					BEEPER_TIM->CCR1 = 0; // 0% duty cycle
					_beep_duration = _tones->duration + 1;
				} else {
					// Play next tone in sequence
					BEEPER_Enable(_tones->frequency,_tones->duration);
				}
			}
		} else {
			BEEPER_Disable();
		}
	}
}

void BEEPER_Enable(uint16_t frequency, uint32_t duration) {
	if (frequency < 100 || frequency > 8000 || duration == 0) {
			return;
	}
	else {
		_beep_duration = (frequency / 100) * duration + 1;
		BEEPER_TIM->ARR = SystemCoreClock / (frequency * BEEPER_TIM->PSC) - 1;
	    //BEEPER_TIM->CCR2 = BEEPER_TIM->ARR >> 1; // 50% duty cycle
		BEEPER_TIM->CCR1 = BEEPER_TIM->ARR >> (BUZZER_VOLUME_MAX + 1 - _volume);
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	}
}

void BEEPER_Disable(void) {
    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
}

void BEEPER_SetVolume(uint16_t volume) {
	if (volume > BUZZER_VOLUME_MAX) volume = BUZZER_VOLUME_MAX;
	_volume = volume;
}

void BEEPER_PlayTones(const Tone_TypeDef * tones) {
	_tones = tones;
	_tones_playing = 1;
	BEEPER_Enable(_tones->frequency,_tones->duration);
}
