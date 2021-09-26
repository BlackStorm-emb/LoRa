/*
 * menus.c
 *
 *  Created on: 25 сент. 2021 г.
 *      Author: Тлехас Алий
 */

#include "menus.h"

static MENU_t menu;

MENU_FUNC_PTR_t func_table[MENU_AMOUNT] = {
	main_menu,
	write_message_menu,
};

void init_menus(MENU_STATE_t initial_state) {
	menu.current_menu = initial_state;
	menu.current_menu_func = func_table[initial_state];
	menu.updated = 0;
}

MENU_STATE_t	get_current_menu(void) {
	return menu.current_menu;
}

MENU_FUNC_PTR_t get_current_menu_func_ptr(void) {
	return menu.current_menu_func;
}

_Bool menu_get_need_update(void) {
	return !menu.updated;
}

void menu_set_need_update(void) {
	menu.updated = 0;
}

void set_current_menu(MENU_STATE_t initial_state) {
	menu.current_menu = initial_state;
	menu.current_menu_func = func_table[initial_state];
}

//menu functions

void *main_menu(char* text1, char* text2, _Bool shifted) {
	static text_area_t text_area1;
	static text_area_t text_area2;
	static text_area_t text_area3;
	ST7735_setTextArea(&text_area1, 1, ST7735_HEIGHT / 2 + 3, 120, 50, &Font_7x10);
	ST7735_setTextArea(&text_area2, 4,  4, 100, 50, &Font_7x10);
	ST7735_setTextArea(&text_area3, 80, 20, 60, 40, &Font_7x10);


	ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, ST7735_COLOR565(0x2b, 0x2d, 0x30));
	ST7735_drawRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, ST7735_WHITE);
	ST7735_drawLine(0, ST7735_HEIGHT / 2, ST7735_WIDTH, ST7735_HEIGHT / 2, ST7735_WHITE);
	ST7735_WriteString(4, 5, "Messages: ", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_writeToTextArea(&text_area1, text1, ST7735_RED, ST7735_BLACK);
	ST7735_writeToTextArea(&text_area2, text2, ST7735_RED, ST7735_BLACK);
	if (shifted) ST7735_writeToTextArea(&text_area3, "Shifted", ST7735_RED, ST7735_BLACK);
	menu.updated = 1;
	return 0;
}

void *write_message_menu(char* text1) {
	return 0;
}
