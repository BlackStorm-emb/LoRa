/*
 * loop.c
 *
 *  Created on: 25 сент. 2021 г.
 *      Author: Тлехас Алий
 *  В данном файле содержится вся основная логика работы программы, которая
 *  сделана по шаблону StateMachine + EventHandler (Событийный автомат)
 */



#include "loop.h"
#include "config.h"
#include "menus.h"

static Keypad_t keypad;
static _Bool keypad_shifted;

extern SPI_HandleTypeDef hspi2;
static SX1278_hw_t SX1278_hw;
static SX1278_t SX1278;

#define BUFFER_LENGTH 100

typedef struct{
	uint8_t i;
	char buf[BUFFER_LENGTH];
} text_buf_t;

static text_buf_t text_buf;
static text_buf_t text_buf_rx;

//State Machine, здесь мы описываем возможные состояния
typedef enum {
	STATE_INIT,
	STATE_IDLE,
	STATE_SHOW_MAIN_MENU,
} STATE_t;

volatile STATE_t state = STATE_INIT;

//Events, здесь мы описываем возможные события
typedef enum {
	EVENT_NONE,
	EVENT_TIMEOUT,
	EVENT_KEY,
} EVENT_t;

volatile EVENT_t event = EVENT_NONE;

void keypadEvent(char key);

//функции состояний
void error(void);
void idle(void);
void show_main_menu(void);

//Ну и делаем таблицу переходов, ведь использовать switch case удел лохов
#define STATE_MAX  3
#define EVENT_MAX  3

typedef void (*TRANSITION_FUNC_PTR_t)(void);

TRANSITION_FUNC_PTR_t transition_table[STATE_MAX][EVENT_MAX] = {
	[STATE_INIT]			[EVENT_NONE]		=	show_main_menu,
	[STATE_INIT]			[EVENT_TIMEOUT]		=	error,
	[STATE_INIT]			[EVENT_KEY]			=	error,
	[STATE_IDLE]			[EVENT_NONE]		= 	idle,
	[STATE_IDLE]			[EVENT_TIMEOUT]		=	error,
	[STATE_IDLE]			[EVENT_KEY]			=	error,
	[STATE_SHOW_MAIN_MENU]	[EVENT_NONE]		= 	show_main_menu,
	[STATE_SHOW_MAIN_MENU]	[EVENT_TIMEOUT]		=	error,
	[STATE_SHOW_MAIN_MENU]	[EVENT_KEY]			=	show_main_menu
};


void setup(void) {
	ST7735_Init();
	ST7735_Start(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);


	BEEPER_SetVolume(2);
	Keypad_create(&keypad, makeKeymap(symbolKeys), rowPins, colPins, ROWS, COLS);
	Keypad_addEventListener(keypadEvent);


	SX1278_hw.dio0.port = LORA_DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = LORA_DIO0_Pin;
	SX1278_hw.nss.port = LORA_NSS_GPIO_Port;
	SX1278_hw.nss.pin = LORA_NSS_Pin;
	SX1278_hw.reset.port = LORA_RST_GPIO_Port;
	SX1278_hw.reset.pin = LORA_RST_Pin;
	SX1278_hw.spi = &hspi2;
	SX1278.hw = &SX1278_hw;

	SX1278.hw = &SX1278_hw;


	SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_14DBM, SX1278_LORA_SF_8, SX1278_LORA_BW_20_8KHZ, 20);
	SX1278_LoRaEntryRx(&SX1278, 50, 2000);

	init_menus(MENU_STATE_MAIN_MENU);
}

void loop(void) {
	while (1) {
		transition_table[state][event](); //ну это всё
	}
}

void error(void) {
	//Если мы находимся здесь, значит произошла ошибка во время работы программы
	while(1) {

	}
}

void idle(void) {

}

void show_main_menu(void) {
	set_current_menu(MENU_STATE_MAIN_MENU);

	uint16_t ret = SX1278_LoRaRxPacket(&SX1278);

	if (ret > 0) {
			BEEPER_Enable(1000, 20);
			SX1278_read(&SX1278, (uint8_t*) text_buf_rx.buf, ret);
			menu_set_need_update();
	}

	char c;
	c = Keypad_getKey(&keypad);
	if (c) {
		if (c == KEY_ENTER) {
			BEEPER_Enable(700, 10);
			SX1278_LoRaEntryTx(&SX1278, strlen(text_buf.buf), 2000);
			SX1278_LoRaTxPacket(&SX1278, (uint8_t *)text_buf.buf, strlen(text_buf.buf), 2000);
			SX1278_LoRaEntryRx(&SX1278, 10, 2000);
			for (int i = 0; i < BUFFER_LENGTH; i++) text_buf.buf[i] = 0;
			text_buf.i = 0;
		}
		else if (c == KEY_DOWN) {
			if (text_buf.i) {
				text_buf.buf[text_buf.i - 1] = 0;
				text_buf.i--;
			}
		}
		else if (c == KEY_SHIFT) {
			if (!keypad_shifted) {
				Keypad_delete(&keypad);
				Keypad_create(&keypad, makeKeymap(shiftKeys), rowPins, colPins, ROWS, COLS);
				keypad_shifted = 1;
			}
			else {
				Keypad_delete(&keypad);
				Keypad_create(&keypad, makeKeymap(symbolKeys), rowPins, colPins, ROWS, COLS);
				keypad_shifted = 0;
			}
		}
		else {
			text_buf.buf[text_buf.i] = c;
			text_buf.i++;
		}
		menu_set_need_update();
		event = EVENT_NONE;
	}

	if (menu_get_need_update()) {
		MENU_FUNC_PTR_t func = get_current_menu_func_ptr();
		(*func)(text_buf.buf, text_buf_rx.buf, keypad_shifted);
	}


	state = STATE_SHOW_MAIN_MENU;
}

void keypadEvent(char key) {
	//unused(key);
	event = EVENT_KEY;
}
