/*
 * config.h
 *
 *  Created on: Sep 4, 2021
 *      Author: Тлехас Алий
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

/* Keyboard settings */
#define ROWS 6
#define COLS 5


#define KEY_SHIFT 2
#define KEY_ENTER 3
#define KEY_DOWN 5

const Keypad_dio_t colPins[COLS] = {
		{K_1_Pin, K_1_GPIO_Port},
		{K_2_Pin, K_2_GPIO_Port},
		{K_3_Pin, K_3_GPIO_Port},
		{K_4_Pin, K_4_GPIO_Port},
		{K_5_Pin, K_5_GPIO_Port}
};

const Keypad_dio_t rowPins[ROWS] = {
		{A_Pin, A_GPIO_Port},
		{B_Pin, B_GPIO_Port},
		{C_Pin, C_GPIO_Port},
		{D_Pin, D_GPIO_Port},
		{E_Pin, E_GPIO_Port},
		{F_Pin, F_GPIO_Port},
};

const char symbolKeys[ROWS][COLS] = {
		{'Q', 'W', 'E', 'R', 'T',},
		{'Y', 'U', 'I', 'O', 'P',},
		{'A', 'S', 'D', 'F', 'G',},
		{'H', 'J', 'K', 'L', 'Z',},
		{'X', 'C', 'V', 'B', 'N',},
		{'M', ' ', KEY_ENTER, KEY_SHIFT, KEY_DOWN}
};

const char shiftKeys[ROWS][COLS] = {
		{'1', '2', '3', '4', '5',},
		{'6', '7', '8', '9', '0',},
		{'!', '"', 'n', ';', '%',},
		{':', '?', '*', '(', ')',},
		{'-', '+', '=', '/', ',',},
		{'.', ' ', KEY_ENTER, KEY_SHIFT, KEY_DOWN}
};


#endif /* INC_CONFIG_H_ */
