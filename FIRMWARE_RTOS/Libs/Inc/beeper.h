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

typedef struct
{
	uint16_t freq;
	uint16_t volume;
	uint16_t duration;
} BEPPER_Parameters_t;

#define BUZZER_QUEUE_LEN	10

#define BUZZER_VOLUME_MAX	10
#define BUZZER_VOLUME_MUTE	0

// Function prototypes
//void BEEPER_Init(void);
void BEEPER_Enable(void);
void BEEPER_Disable(void);
void BEEPER_SetFreq(uint16_t freq);
void BEEPER_SetVolume(uint16_t volume);

#endif // __BEEPER_H

#endif /* INC_BEEPER_H_ */
