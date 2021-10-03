/*
 * menus.h
 *
 *  Created on: 25 сент. 2021 г.
 *      Author: Тлехас Алий
 */

#ifndef INC_MENUS_H_
#define INC_MENUS_H_

#include "main.h"

#define MENU_AMOUNT 4

#define BUFFER_LENGTH 100
#define FIFO_LENGTH 10

typedef struct{
	uint8_t i;
	char buf[BUFFER_LENGTH];
} text_buf_t;

typedef struct{
	uint8_t i;
	char texts[FIFO_LENGTH][BUFFER_LENGTH];
} text_FIFO_buf_t;

typedef enum {
	MENU_STATE_MAIN_MENU,
	MENU_STATE_SETUP,
	MENU_STATE_WRITE_MESSAGE,
	MENU_STATE_READ_MESSAGES
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

void *main_menu(uint16_t number_new, uint8_t ID, uint8_t bat_level);
void *setup_menu(uint8_t freq, uint8_t spr, uint8_t power, uint8_t ID);
void *read_message_menu(text_FIFO_buf_t * text_buf_rx);
void *write_message_menu(char* text1, _Bool shifted);


#endif /* INC_MENUS_H_ */
