/*
 * Display_GFX.h
 *
 *  Created on: 8 сент. 2021 г.
 *      Author: Тлехас Алий
 */

#ifndef DISPLAY_GFX_INC_DISPLAY_GFX_H_
#define DISPLAY_GFX_INC_DISPLAY_GFX_H_

#include "main.h"

void ST7735_drawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void ST7735_drawHLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);

void ST7735_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

void ST7735_drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void ST7735_drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t corners, uint16_t color);
void ST7735_fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void ST7735_fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void ST7735_drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7735_fillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7735_drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
void ST7735_fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
	FontDef *font;

} text_area_t;

_Bool ST7735_setTextArea(text_area_t *area, uint16_t x, uint16_t y, uint16_t w, uint16_t h, FontDef *font);
_Bool ST7735_writeToTextArea(text_area_t *area, char *text, uint16_t color, uint16_t bgcolor);

#endif /* DISPLAY_GFX_INC_DISPLAY_GFX_H_ */
