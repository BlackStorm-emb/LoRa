/*
 * Keypad.h
 *
 *  Created on: Sep 5, 2021
 *      Author: Тлехас Алий
 *
 *	Library support only one KeyBoard because complete OOP will cost additional resources
 *	Based on https://github.com/Chris--A/Keypad
 */

#ifndef KEYBOARD_KEYPAD_INC_KEYPAD_H_
#define KEYBOARD_KEYPAD_INC_KEYPAD_H_

#include  "main.h"
#include "key.h"

typedef char KeypadEvent;

#define OPEN 0
#define CLOSED 1

#define LIST_MAX 10		// Max number of keys on the active list.
#define MAPSIZE 10		// MAPSIZE is the number of rows (times 16 columns)
#define makeKeymap(x) ((char*)x)

typedef struct {
	int pin;
	GPIO_TypeDef *port;
} Keypad_dio_t;

typedef struct {
    uint8_t rows;
    uint8_t columns;
} KeypadSize;

typedef struct {
	uint16_t bitMap[MAPSIZE];
	Key_t key[LIST_MAX];
	uint32_t holdTimer;
	_Bool isInit;
} Keypad_t;


_Bool Keypad_create(Keypad_t *keypad, char *userKeymap, const Keypad_dio_t *row, const Keypad_dio_t *col, uint8_t numRows, uint8_t numCols);
void Keypad_delete(Keypad_t *keypad);

char Keypad_getKey(Keypad_t *keypad);
_Bool Keypad_getKeys(Keypad_t *keypad);
KeyState Keypad_getState(Keypad_t *keypad);
void Keypad_begin(char *userKeymap);
_Bool Keypad_isPressed(Keypad_t *keypad, char keyChar);
void Keypad_setDebounceTime(uint16_t debounce);
void Keypad_setHoldTime(uint16_t time);
void Keypad_addEventListener(void (*listener)(char));
int Keypad_findInListChar(Keypad_t *keypad, char keyChar);
int Keypad_findInListInt(Keypad_t *keypad, int keyCode);
char Keypad_waitForKey(Keypad_t *keypad);
_Bool Keypad_keyStateChanged(Keypad_t *keypad);
uint8_t Keypad_numKeys(Keypad_t *keypad);


#endif /* KEYBOARD_KEYPAD_INC_KEYPAD_H_ */
