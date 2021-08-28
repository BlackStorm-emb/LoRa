/*
 * keyboard.h
 *
 *  Created on: Aug 16, 2021
 *      Author: Тлехас Алий
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#include "main.h"

#define SHORT_CLICK_DELAY 70
#define LONG_CLICK_DELAY 40

#define KEYBOARD_BUFFER_SYMB_LENGTH 500
#define KEYBOARD_BUFFER_CTRL_LENGTH 10

void keyboard_startHANDLE(void);
void keyboard_HANDLE(void); //process clicks

void keyboard_INIT(void);


char keyboard_ReadLast(_Bool isSymb);
void keyboard_BufRead(char * buf);

#endif /* INC_KEYBOARD_H_ */
