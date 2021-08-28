/*
 * keyboard.c
 *
 *  Created on: Aug 16, 2021
 *      Author: Тлехас Алий
 */
#include "keyboard.h"

static struct KEYBOARD_struct {
	char keyboard_buffer_symb[KEYBOARD_BUFFER_SYMB_LENGTH];
	char keyboard_buffer_ctrl[KEYBOARD_BUFFER_CTRL_LENGTH];
	uint16_t end_ptr_symb;
	uint16_t end_ptr_ctrl;
	_Bool isShifted;
	_Bool isPressing;

} keyboard;

extern TIM_HandleTypeDef htim7;

void keyboard_INIT(void) {
	for(size_t i = 0; i < KEYBOARD_BUFFER_SYMB_LENGTH; i++) {
		keyboard.keyboard_buffer_symb[i] = 0;
	}
	for(size_t i = 0; i < KEYBOARD_BUFFER_CTRL_LENGTH; i++) {
		keyboard.keyboard_buffer_ctrl[i] = 0;
	}
	keyboard.end_ptr_symb = 0;
	keyboard.end_ptr_ctrl = 0;
	keyboard.isShifted = 0;
	keyboard.isPressing = 0;
	//HAL_TIM_Base_Start_IT(&htim7);
}

void keyboard_startHANDLE(void) {
	if (!keyboard.isPressing) {
		TIM7->ARR = SHORT_CLICK_DELAY - 1;
		keyboard.isPressing = 1;
		HAL_TIM_Base_Start_IT(&htim7);
	}
}

void keyboard_HANDLE(void) {
	if (keyboard.end_ptr_symb == KEYBOARD_BUFFER_SYMB_LENGTH - 2 || keyboard.end_ptr_ctrl > KEYBOARD_BUFFER_CTRL_LENGTH - 2)
		//overflow
		return;

	uint16_t this_end_ptr_symb = keyboard.end_ptr_symb;
	uint16_t this_end_ptr_ctrl = keyboard.end_ptr_ctrl;
	//A
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
	if (!HAL_GPIO_ReadPin(K_1_GPIO_Port, K_1_Pin)) {
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'Q';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'q';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_2_GPIO_Port, K_2_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'W';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'w';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_3_GPIO_Port, K_3_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'E';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'e';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_4_GPIO_Port, K_4_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'R';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'r';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_5_GPIO_Port, K_5_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'T';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 't';
		keyboard.end_ptr_symb++;
	}
	return;
	//B
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
	if (!HAL_GPIO_ReadPin(K_1_GPIO_Port, K_1_Pin)) {
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'Y';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'y';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_2_GPIO_Port, K_2_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'U';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'u';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_3_GPIO_Port, K_3_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'I';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'i';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_4_GPIO_Port, K_4_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'O';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'o';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_5_GPIO_Port, K_5_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'P';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'p';
		keyboard.end_ptr_symb++;
	}

	//C
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
	if (!HAL_GPIO_ReadPin(K_1_GPIO_Port, K_1_Pin)) {
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'A';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'a';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_2_GPIO_Port, K_2_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'S';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 's';
		keyboard.end_ptr_symb++;
	}
	if (!HAL_GPIO_ReadPin(K_3_GPIO_Port, K_3_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'D';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'd';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_4_GPIO_Port, K_4_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'F';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'f';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_5_GPIO_Port, K_5_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'G';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'g';
		keyboard.end_ptr_symb++;
	}

	//D
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
	if (!HAL_GPIO_ReadPin(K_1_GPIO_Port, K_1_Pin)) {
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'H';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'h';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_2_GPIO_Port, K_2_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'J';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'j';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_3_GPIO_Port, K_3_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'K';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'k';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_4_GPIO_Port, K_4_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'L';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'l';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_5_GPIO_Port, K_5_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'Z';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'z';
		keyboard.end_ptr_symb++;
	}

	//E
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
	if (!HAL_GPIO_ReadPin(K_1_GPIO_Port, K_1_Pin)) {
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'X';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'x';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_2_GPIO_Port, K_2_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'C';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'c';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_3_GPIO_Port, K_3_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'V';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'v';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_4_GPIO_Port, K_4_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'B';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'b';
		keyboard.end_ptr_symb++;
}
	else if (!HAL_GPIO_ReadPin(K_5_GPIO_Port, K_5_Pin)){
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'N';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'n';
		keyboard.end_ptr_symb++;
	}

	//F
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
	if (!HAL_GPIO_ReadPin(K_1_GPIO_Port, K_1_Pin)) {
		if (keyboard.isShifted)
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'M';
		else
			keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb] = 'm';
		keyboard.end_ptr_symb++;
	}
	else if (!HAL_GPIO_ReadPin(K_2_GPIO_Port, K_2_Pin)){
		keyboard.keyboard_buffer_ctrl[keyboard.end_ptr_ctrl] = 'S';
		keyboard.end_ptr_ctrl++;
	}
	else if (!HAL_GPIO_ReadPin(K_3_GPIO_Port, K_3_Pin)){
		keyboard.keyboard_buffer_ctrl[keyboard.end_ptr_ctrl] = 'E';
		keyboard.end_ptr_ctrl++;
	}
	else if (!HAL_GPIO_ReadPin(K_4_GPIO_Port, K_4_Pin)){
		keyboard.keyboard_buffer_ctrl[keyboard.end_ptr_ctrl] = 'U';
		keyboard.end_ptr_ctrl++;
	}
	else if (!HAL_GPIO_ReadPin(K_5_GPIO_Port, K_5_Pin)){
		keyboard.keyboard_buffer_ctrl[keyboard.end_ptr_ctrl] = 'D';
		keyboard.end_ptr_ctrl++;
	}

	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);



		keyboard.isPressing = 0;
}

char keyboard_ReadLast(_Bool isSymb) {
	if (isSymb)
		return keyboard.keyboard_buffer_symb[keyboard.end_ptr_symb - 1];
	else
		return keyboard.keyboard_buffer_ctrl[keyboard.end_ptr_ctrl - 1];
}

void keyboard_BufRead(char * buf) {
	if (keyboard.end_ptr_symb <= 1) return;
	memcpy(buf, keyboard.keyboard_buffer_symb, keyboard.end_ptr_symb + 1);
}
