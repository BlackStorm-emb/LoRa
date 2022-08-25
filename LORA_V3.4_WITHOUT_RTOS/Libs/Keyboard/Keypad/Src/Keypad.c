/*
 * Keypad.c
 *
 *  Created on: Sep 5, 2021
 *      Author: Тлехас Алий
 */

#include "Keypad.h"

/*////////////////////////////////////////////////*/
	static uint16_t startTime;
	static char *keymap;
	static Keypad_dio_t *rowPins;
	static Keypad_dio_t *columnPins;
	static KeypadSize sizeKpd;
	static uint16_t debounceTime;
	static uint16_t holdTime;
	static _Bool single_key;

	void (*keypadEventListener)(char);

	static _Bool isInit = 0;

	static void scanKeys(Keypad_t *keypad);
	static _Bool updateList(Keypad_t *keypad);
	static void nextKeyState(Keypad_t *keypad, uint8_t idx, _Bool button);
	static void transitionTo(Keypad_t *keypad, uint8_t idx, KeyState nextState);
/*////////////////////////////////////////////////*/

_Bool Keypad_create(Keypad_t *keypad, char *userKeymap, const Keypad_dio_t *row, const Keypad_dio_t *col, uint8_t numRows, uint8_t numCols) {
	if (isInit) return 0;
	rowPins = (Keypad_dio_t*)row;
	columnPins = (Keypad_dio_t*)col;
	sizeKpd.rows = numRows;
	sizeKpd.columns = numCols;

	Keypad_begin(userKeymap);

	Keypad_setDebounceTime(10);
	Keypad_setHoldTime(300);
	//keypadEventListener = 0;

	startTime = 0;
	single_key = 0;

	for (uint8_t r = 0; r < sizeKpd.rows; r++) {
		HAL_GPIO_WritePin(rowPins[r].port, rowPins[r].pin, SET);
	}

	keypad->isInit = 1;
	isInit = 1;
	return 1;
}

void Keypad_delete(Keypad_t *keypad) {
	keypad->isInit = 0;
	isInit = 0;
}

char Keypad_getKey(Keypad_t *keypad) {
	single_key = 1;

	if (Keypad_getKeys(keypad) && keypad->key[0].stateChanged && (keypad->key[0].kstate==PRESSED))
			return keypad->key[0].kchar;

	single_key = false;

	return NO_KEY;
}

_Bool Keypad_getKeys(Keypad_t *keypad) {
	_Bool keyActivity = 0;

	if ( (HAL_GetTick() - startTime) > debounceTime ) {
		scanKeys(keypad);
		keyActivity = updateList(keypad);
		startTime = HAL_GetTick();
	}

	return keyActivity;
}

KeyState Keypad_getState(Keypad_t *keypad) {
	return keypad->key[0].kstate;
}

void Keypad_begin(char *userKeymap) {
	keymap = userKeymap;
}

_Bool Keypad_isPressed(Keypad_t *keypad, char keyChar) {
	for (uint8_t i = 0; i < LIST_MAX; i++) {
		if (keypad->key[i].kchar == keyChar ) {
			if ( (keypad->key[i].kstate == PRESSED) && keypad->key[i].stateChanged )
				return true;
		}
	}
	return false;	// Not pressed.
}

void Keypad_setDebounceTime(uint16_t debounce) {
	debounceTime = debounce;
}

void Keypad_setHoldTime(uint16_t time) {
	holdTime = time;
}

void Keypad_addEventListener(void (*listener)(char)) {
	keypadEventListener = listener;
}

int Keypad_findInListChar(Keypad_t *keypad, char keyChar) {
	for (uint8_t i=0; i < LIST_MAX; i++) {
		if (keypad->key[i].kchar == keyChar) {
			return i;
		}
	}
	return -1;
}

int Keypad_findInListInt(Keypad_t *keypad, int keyCode) {
	for (uint8_t i=0; i < LIST_MAX; i++) {
		if (keypad->key[i].kcode == keyCode) {
			return i;
		}
	}
	return -1;
}

char Keypad_waitForKey(Keypad_t *keypad) {
	char waitKey = NO_KEY;
	while( (waitKey = Keypad_getKey(keypad)) == NO_KEY );	// Block everything while waiting for a keypress.
	return waitKey;
}

_Bool Keypad_keyStateChanged(Keypad_t *keypad) {
	return keypad->key[0].stateChanged;
}

uint8_t Keypad_numKeys(Keypad_t *keypad) {
	return sizeof(keypad->key)/sizeof(Key_t);
}

////////////////////////////////////////////////////////////////////////
void scanKeys(Keypad_t *keypad) {
	for (uint8_t r = 0; r < sizeKpd.rows; r++) {
		HAL_GPIO_WritePin(rowPins[r].port, rowPins[r].pin, RESET);
		for (uint8_t c = 0; c < sizeKpd.columns; c++) {
			bitWrite(&(keypad->bitMap[r]), c, !HAL_GPIO_ReadPin(columnPins[c].port, columnPins[c].pin));
		}
		HAL_GPIO_WritePin(rowPins[r].port, rowPins[r].pin, SET);
	}
}

_Bool updateList(Keypad_t *keypad) {
	_Bool anyActivity = false;

		// Delete any IDLE keys
		for (uint8_t i=0; i < LIST_MAX; i++) {
			if (keypad->key[i].kstate==IDLE) {
				keypad->key[i].kchar = NO_KEY;
				keypad->key[i].kcode = -1;
				keypad->key[i].stateChanged = false;
			}
		}

		// Add new keys to empty slots in the key list.
		for (uint8_t r=0; r < sizeKpd.rows; r++) {
			for (uint8_t c = 0; c < sizeKpd.columns; c++) {
				_Bool button = bitRead(keypad->bitMap[r], c);
				char keyChar = keymap[r * sizeKpd.columns + c];
				int keyCode = r * sizeKpd.columns + c;
				int idx =  Keypad_findInListInt(keypad, keyCode);
				// Key is already on the list so set its next state.
				if (idx > -1)	{
					nextKeyState(keypad, idx, button);
				}
				// Key is NOT on the list so add it.
				if ((idx == -1) && button) {
					for (uint8_t i = 0; i < LIST_MAX; i++) {
						if (keypad->key[i].kchar == NO_KEY) {		// Find an empty slot or don't add key to list.
							keypad->key[i].kchar = keyChar;
							keypad->key[i].kcode = keyCode;
							keypad->key[i].kstate = IDLE;		// Keys NOT on the list have an initial state of IDLE.
							nextKeyState(keypad, i, button);
							break;	// Don't fill all the empty slots with the same key.
						}
					}
				}
			}
		}

		// Report if the user changed the state of any key.
		for (uint8_t i=0; i < LIST_MAX; i++) {
			if (keypad->key[i].stateChanged) anyActivity = 1;
		}

		return anyActivity;
}

void nextKeyState(Keypad_t *keypad, uint8_t idx, _Bool button) {
	keypad->key[idx].stateChanged = false;

	switch (keypad->key[idx].kstate) {
		case IDLE:
			if (button == CLOSED) {
				transitionTo(keypad, idx, PRESSED);
				keypad->holdTimer = HAL_GetTick(); }		// Get ready for next HOLD state.
			break;
		case PRESSED:
			if ((HAL_GetTick() - keypad->holdTimer) > holdTime)	// Waiting for a key HOLD...
				transitionTo(keypad, idx, HOLD);
			else if (button == OPEN)				// or for a key to be RELEASED.
				transitionTo (keypad, idx, RELEASED);
			break;
		case HOLD:
			if (button == OPEN)
				transitionTo(keypad, idx, RELEASED);
			break;
		case RELEASED:
			transitionTo(keypad, idx, IDLE);
			break;
	}
}

void transitionTo(Keypad_t *keypad, uint8_t idx, KeyState nextState) {
	keypad->key[idx].kstate = nextState;
	keypad->key[idx].stateChanged = true;


	if (single_key)  {
		if ( (keypadEventListener!=NULL) && (idx==0) )  {
			keypadEventListener(keypad->key[0].kchar);
		}
	}

	else {
		if (keypadEventListener!=NULL)  {
			keypadEventListener(keypad->key[idx].kchar);
		}
	}
}


