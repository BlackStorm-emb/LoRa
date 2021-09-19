/*
 * Key.c
 *
 *  Created on: Sep 5, 2021
 *      Author: Тлехас Алий
 */

#include "Key.h"

void key_ctor(Key_t *key, char userKeyChar) {
	key->kchar = userKeyChar;
	key->kstate = IDLE;
	key->stateChanged = 0;
	key->kcode = - 1;
}

void key_update(Key_t *key, char userKeyChar, KeyState userState, _Bool userStatus) {
	key->kchar = userKeyChar;
	key->kstate = userState;
	key->stateChanged = userStatus;
}
