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
	setup_menu,
	write_message_menu,
	read_message_menu
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

/*
void *main_menu(char* text1, char* text2, _Bool shifted) {
	static text_area_t text_area1;
	static text_area_t text_area2;
	static text_area_t text_area3;
	ST7735_setTextArea(&text_area1, 1, ST7735_HEIGHT / 2 + 3, 158, 50, &Font_7x10);
	ST7735_setTextArea(&text_area2, 4,  15, 100, 50, &Font_7x10);
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
*/

void *main_menu(uint16_t number_new, uint8_t ID, uint8_t bat_level) {
	char buf[50];
	ST7735_FillScreen(ST7735_BLACK);

	//ID BLOCK
	text_area_t text;
	ST7735_setTextArea(&text, 1, 2, 80, 20, &Font_7x10);
	ST7735_fillRoundRect(0, 0, 80, 15, 2, 0x23ffd);
	ST7735_drawRoundRect(0, 0, 80, 15, 2, ST7735_WHITE);
	sprintf(buf, "my ID: %d",ID);
	ST7735_writeToTextArea(&text, buf, ST7735_BLACK, 0x23ffd);

	//BAT_BLOCK
	ST7735_drawRect(119, 2, 29, 12, ST7735_CYAN);
	ST7735_FillRectangle(148, 5, 4, 6, ST7735_CYAN);
	if (bat_level == 4) {
		ST7735_FillRectangle(141, 3, 6, 10, ST7735_GREEN);
	}
	if (bat_level >= 3){
		ST7735_FillRectangle(134, 3, 6, 10, ST7735_YELLOW);
	}
	if (bat_level >= 2){
		ST7735_FillRectangle(127, 3, 6, 10, ST7735_RED);
	}
	if (bat_level >= 1){
		ST7735_FillRectangle(120, 3, 6, 10, ST7735_RED);
	}

	//MESSAGES BLOCK
	ST7735_drawRoundRect(0, 20, 160, 40, 2, ST7735_RED);
	ST7735_setTextArea(&text, 1, 21, 158, 40, &Font_7x10);
	ST7735_writeToTextArea(&text, "[N] New message", ST7735_WHITE, ST7735_BLACK);

	ST7735_drawRoundRect(0, 62, 160, 40, 2, ST7735_GREEN);
	ST7735_setTextArea(&text, 1, 64, 158, 40, &Font_7x10);
	sprintf(buf, "[M] Messages: %d", number_new);
	ST7735_writeToTextArea(&text, buf, ST7735_WHITE, ST7735_BLACK);


	ST7735_setTextArea(&text, 80, 108, 70, 12, &Font_7x10);
	ST7735_writeToTextArea(&text, "[S] Setup", ST7735_WHITE, ST7735_BLACK);


	menu.updated = 1;
	return 0;
}

void *setup_menu(uint8_t freq, uint8_t spr, uint8_t power, uint8_t ID) {
	menu.updated = 1;
}
void *read_message_menu(text_FIFO_buf_t* text_buf_rx) {
	text_area_t text;
	ST7735_FillScreen(ST7735_BLACK);
	for (uint8_t i = 0; i < 10; i++) {
		ST7735_setTextArea(&text, 0, i * 10, 159, 10, &Font_7x10);
		ST7735_writeToTextArea(&text, text_buf_rx->texts[i], ST7735_WHITE, ST7735_BLACK);
	}
	menu.updated = 1;
}
void *write_message_menu(char* text1, _Bool shifted) {
	text_area_t text;
	ST7735_FillScreen(ST7735_BLACK);

	ST7735_setTextArea(&text, 0, 0, 159, 10, &Font_7x10);
	ST7735_writeToTextArea(&text, text1, ST7735_WHITE, ST7735_BLACK);
	if(shifted) {
		ST7735_setTextArea(&text, 100, 100, 59, 10, &Font_7x10);
		ST7735_writeToTextArea(&text, "Shift", ST7735_RED, ST7735_BLACK);
	}

	menu.updated = 1;
}
