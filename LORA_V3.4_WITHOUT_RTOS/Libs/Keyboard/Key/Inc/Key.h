/*
 * Key.h
 *
 *  Created on: Sep 5, 2021
 *      Author: Тлехас Алий
 */

#ifndef KEYBOARD_KEY_INC_KEY_H_
#define KEYBOARD_KEY_INC_KEY_H_

typedef enum{ IDLE, PRESSED, HOLD, RELEASED } KeyState;

#define NO_KEY '\0'

typedef struct {
	char kchar;
	int  kcode;
	KeyState kstate;
	_Bool stateChanged;
} Key_t;

void key_ctor(Key_t *key, char userKeyChar);
void key_update(Key_t *key, char userKeyChar, KeyState userState, _Bool userStatus);

#endif /* KEYBOARD_KEY_INC_KEY_H_ */
