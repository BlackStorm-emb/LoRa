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
#include "testimg.h"

static Keypad_t keypad;
static _Bool keypad_shifted;

extern SPI_HandleTypeDef hspi2;
static SX1278_hw_t SX1278_hw;
static SX1278_t SX1278;

static text_buf_t text_buf;
static text_FIFO_buf_t text_buf_rx;
static char TX_BUF[BUFFER_LENGTH];

static uint8_t MyID = 13;

static uint8_t unread_mes = 0;

//State Machine, здесь мы описываем возможные состояния
typedef enum {
	STATE_INIT,
	STATE_IDLE,
	STATE_SHOW_CURRENT_MENU,
} STATE_t;

volatile STATE_t state = STATE_INIT;

//Events, здесь мы описываем возможные события
typedef enum {
	EVENT_NONE,
	EVENT_TIMEOUT,
	EVENT_KEY,
	EVENT_RX
} EVENT_t;

volatile EVENT_t event = EVENT_NONE;

void keypadEvent(char key);

//функции состояний
void error(void);
void idle(void);
void show_current_menu(void);


#define STATE_MAX  3
#define EVENT_MAX  4

typedef void (*TRANSITION_FUNC_PTR_t)(void);

TRANSITION_FUNC_PTR_t transition_table[STATE_MAX][EVENT_MAX] = {
	[STATE_INIT]			[EVENT_NONE]		=	show_current_menu,
	[STATE_INIT]			[EVENT_TIMEOUT]		=	error,
	[STATE_INIT]			[EVENT_KEY]			=	error,
	[STATE_INIT]			[EVENT_RX]			=	error,
	[STATE_IDLE]			[EVENT_NONE]		= 	idle,
	[STATE_IDLE]			[EVENT_TIMEOUT]		=	show_current_menu,
	[STATE_IDLE]			[EVENT_KEY]			=	show_current_menu,
	[STATE_IDLE]			[EVENT_RX]			=	show_current_menu,
	[STATE_SHOW_CURRENT_MENU]	[EVENT_NONE]		= 	idle,
	[STATE_SHOW_CURRENT_MENU]	[EVENT_TIMEOUT]		=	error,
	[STATE_SHOW_CURRENT_MENU]	[EVENT_KEY]			=	idle,
	[STATE_SHOW_CURRENT_MENU]	[EVENT_RX]			=	idle
};


void setup(void) {
	ST7735_Init();
	ST7735_Start(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);


	BEEPER_SetVolume(2);
	Keypad_create(&keypad, makeKeymap(symbolKeys), rowPins, colPins, ROWS, COLS);
	//Keypad_addEventListener(keypadEvent);


	SX1278_hw.dio0.port = LORA_DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = LORA_DIO0_Pin;
	SX1278_hw.nss.port = LORA_NSS_GPIO_Port;
	SX1278_hw.nss.pin = LORA_NSS_Pin;
	SX1278_hw.reset.port = LORA_RST_GPIO_Port;
	SX1278_hw.reset.pin = LORA_RST_Pin;
	SX1278_hw.spi = &hspi2;
	SX1278.hw = &SX1278_hw;

	SX1278.hw = &SX1278_hw;


	SX1278_begin(&SX1278, SX1278_433MHZ, SX1278_POWER_20DBM, SX1278_LORA_SF_6, SX1278_LORA_BW_10_4KHZ, 20);
	SX1278_LoRaEntryRx(&SX1278, BUFFER_LENGTH, 2000);
	ST7735_DrawImage(0, 0, 160, 128, test_img_160x128_radio);
	HAL_Delay(500);

	init_menus(MENU_STATE_MAIN_MENU);
	menu_set_need_update();
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
	const uint32_t _timeout = 10000;

	static uint32_t millis;
	state = STATE_IDLE;
	uint16_t ret = SX1278_LoRaRxPacket(&SX1278);

	if (HAL_GetTick() - millis > _timeout) {
		millis = HAL_GetTick();
		event = EVENT_TIMEOUT;
		menu_set_need_update();
	}

	if (ret > 0) {
			BEEPER_Enable(1000, 20);
			if (text_buf_rx.i == FIFO_LENGTH - 1 ) {
				for (uint8_t i = 0; i < FIFO_LENGTH - 1; i++) {
					memcpy(text_buf_rx.texts[i], text_buf_rx.texts[i + 1], strlen(text_buf_rx.texts[i + 1]));
				}
				SX1278_read(&SX1278, (uint8_t*) text_buf_rx.texts[text_buf_rx.i], ret);
			}
			else {
				SX1278_read(&SX1278, (uint8_t*) text_buf_rx.texts[text_buf_rx.i], ret);
				text_buf_rx.i++;
			}
			menu_set_need_update();
			event = EVENT_RX;
			unread_mes++;
	}

	char new_key;
	new_key = Keypad_getKey(&keypad);
	if (new_key) {
		if (get_current_menu() == MENU_STATE_MAIN_MENU) {
			if (new_key == 'N') {
				set_current_menu(MENU_STATE_WRITE_MESSAGE);
				menu_set_need_update();
			}
			else if (new_key == 'M') {
				set_current_menu(MENU_STATE_READ_MESSAGES);
				menu_set_need_update();
			}
			else if (new_key == 'S') {
				set_current_menu(MENU_STATE_SETUP);
				menu_set_need_update();
			}
		}
		else if (get_current_menu() == MENU_STATE_READ_MESSAGES) {
			if (new_key == 'X') {
				set_current_menu(MENU_STATE_MAIN_MENU);
				menu_set_need_update();
			}
		}
		else if (get_current_menu() == MENU_STATE_WRITE_MESSAGE) {
			menu_set_need_update();
			if (new_key == KEY_ENTER) {
				set_current_menu(MENU_STATE_MAIN_MENU);
				menu_set_need_update();

				sprintf(TX_BUF, "ID:%d->mes: ", MyID);
				strncat(TX_BUF, text_buf.buf, 100 - strlen(TX_BUF));
				BEEPER_Enable(700, 10);
				SX1278_LoRaEntryTx(&SX1278, strlen(TX_BUF), 2000);
				SX1278_LoRaTxPacket(&SX1278, (uint8_t *)TX_BUF, strlen(TX_BUF), 2000);
				SX1278_LoRaEntryRx(&SX1278, 10, 2000);
				for (int i = 0; i < BUFFER_LENGTH; i++) text_buf.buf[i] = 0;
				text_buf.i = 0;
				Keypad_delete(&keypad);
				Keypad_create(&keypad, makeKeymap(symbolKeys), rowPins, colPins, ROWS, COLS);
				keypad_shifted = 0;
			}
			else if (new_key == KEY_DOWN) {
				if (text_buf.i) {
					text_buf.buf[text_buf.i - 1] = 0;
					text_buf.i--;
				}
			}
			else if (new_key == KEY_SHIFT) {
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
				text_buf.buf[text_buf.i] = new_key;
				text_buf.i++;
			}
		}

		event = EVENT_KEY;
	}
	HAL_Delay(10);
}

void show_current_menu(void) {
	state = STATE_SHOW_CURRENT_MENU;
	event = EVENT_NONE;
	if (menu_get_need_update()) {
		MENU_FUNC_PTR_t func = get_current_menu_func_ptr();
		if (get_current_menu() == MENU_STATE_MAIN_MENU) {
			uint8_t level = getBatLevel();
			(*func)(unread_mes, MyID, level);
		}
		else if (get_current_menu() == MENU_STATE_READ_MESSAGES) {
			(*func)(&text_buf_rx);
		}
		else if (get_current_menu() == MENU_STATE_WRITE_MESSAGE) {
			(*func)(&text_buf.buf, keypad_shifted);
		}
	}
}

void keypadEvent(char key) {
	//unused(key);
	event = EVENT_KEY;
}
