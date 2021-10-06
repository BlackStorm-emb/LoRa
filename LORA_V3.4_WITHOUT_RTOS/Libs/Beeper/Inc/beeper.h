/*
 * beeper.h
 *
 *  Created on: Aug 17, 2021
 *      Author: Тлехас Алий
 */

#ifndef INC_BEEPER_H_
#define INC_BEEPER_H_

// Define to prevent recursive inclusion -------------------------------------
#ifndef __BEEPER_H
#define __BEEPER_H

#include "main.h"

#define BEEPER_TIM TIM2

// Single tone definition
typedef struct {
	uint16_t frequency;
	uint8_t  duration;
} Tone_TypeDef;

// "Super Mario bros." =)
static const Tone_TypeDef tones_SMB[] = {
		{2637,18}, // E7 x2
		{   0, 9}, // x3
		{2637, 9}, // E7
		{   0, 9}, // x3
		{2093, 9}, // C7
		{2637, 9}, // E7
		{   0, 9}, // x3
		{3136, 9}, // G7
		{   0,27}, // x3
		{1586, 9}, // G6
		{   0,27}, // x3

		{2093, 9}, // C7
		{   0,18}, // x2
		{1586, 9}, // G6
		{   0,18}, // x2
		{1319, 9}, // E6
		{   0,18}, // x2
		{1760, 9}, // A6
		{   0, 9}, // x1
		{1976, 9}, // B6
		{   0, 9}, // x1
		{1865, 9}, // AS6
		{1760, 9}, // A6
		{   0, 9}, // x1

		{1586,12}, // G6
		{2637,12}, // E7
		{3136,12}, // G7
		{3520, 9}, // A7
		{   0, 9}, // x1
		{2794, 9}, // F7
		{3136, 9}, // G7
		{   0, 9}, // x1
		{2637, 9}, // E7
		{   0, 9}, // x1
		{2093, 9}, // C7
		{2349, 9}, // D7
		{1976, 9}, // B6
		{   0,18}, // x2

		{2093, 9}, // C7
		{   0,18}, // x2
		{1586, 9}, // G6
		{   0,18}, // x2
		{1319, 9}, // E6
		{   0,18}, // x2
		{1760, 9}, // A6
		{   0, 9}, // x1
		{1976, 9}, // B6
		{   0, 9}, // x1
		{1865, 9}, // AS6
		{1760, 9}, // A6
		{   0, 9}, // x1

		{1586,12}, // G6
		{2637,12}, // E7
		{3136,12}, // G7
		{3520, 9}, // A7
		{   0, 9}, // x1
		{2794, 9}, // F7
		{3136, 9}, // G7
		{   0, 9}, // x1
		{2637, 9}, // E7
		{   0, 9}, // x1
		{2093, 9}, // C7
		{2349, 9}, // D7
		{1976, 9}, // B6

		{   0, 0}
};

#define BUZZER_VOLUME_MAX	10
#define BUZZER_VOLUME_MUTE	0

// Function prototypes
void BEEPER_Enable(uint16_t frequency, uint32_t duration); //включение пищалки на определенную частоту на время duration
void BEEPER_Disable(void); //Отключение генерации
void BEEPER_SetVolume(uint16_t volume); //Установка громкости
void BEEPER_PlayTones(const Tone_TypeDef * melody); //Загрузка мелодии

void BEEPER_IT(void); //функция вызываемая в прерывании

#endif // __BEEPER_H

#endif /* INC_BEEPER_H_ */
