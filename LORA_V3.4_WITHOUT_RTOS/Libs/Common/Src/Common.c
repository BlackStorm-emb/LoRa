/*
 * Common.c
 *
 *  Created on: 8 сент. 2021 г.
 *      Author: Тлехас Алий
 */

#include "Common.h"

void bitWrite(uint16_t* word, uint8_t pos, _Bool bit) {
	if (bit)
		*word = *word | (1 << pos);
	else
		*word = *word & ~(1 << pos);
}

_Bool bitRead(uint16_t word, uint8_t pos) {
	return (word >> pos) & 0x01;
}

void _swap_uint16_t(uint16_t *a, uint16_t *b) {
	uint16_t swap;
	swap = *a;
	*a = *b;
	*b = swap;
}

void _swap_int16_t(int16_t *a, int16_t *b) {
	int16_t swap;
	swap = *a;
	*a = *b;
	*b = swap;
}
