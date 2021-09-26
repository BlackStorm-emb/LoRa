/*
 * menus.h
 *
 *  Created on: 25 сент. 2021 г.
 *      Author: Тлехас Алий
 */

#ifndef INC_MENUS_H_
#define INC_MENUS_H_

#include "main.h"

#define MENU_AMOUNT 2

typedef enum {
	MENU_STATE_MAIN_MENU,
	MENU_STATE_WRITE_MESSAGE,
} MENU_STATE_t;

typedef void *(*MENU_FUNC_PTR_t)();

typedef struct {
	MENU_STATE_t	current_menu;
	MENU_FUNC_PTR_t current_menu_func;
	_Bool 			updated;
} MENU_t;

void init_menus(MENU_STATE_t initial_state);

MENU_STATE_t	get_current_menu(void);
MENU_FUNC_PTR_t get_current_menu_func_ptr(void);
_Bool menu_get_need_update(void);
void menu_set_need_update(void);
void set_current_menu(MENU_STATE_t state);


//menu functions

void *main_menu(char* text1, char* text2);
void *write_message_menu(char* text1);

#endif /* INC_MENUS_H_ */
