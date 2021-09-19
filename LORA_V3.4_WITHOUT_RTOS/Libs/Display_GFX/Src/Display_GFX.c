/*
 * Display_GFX.c
 *
 *  Created on: 8 сент. 2021 г.
 *      Author: Тлехас Алий
 */

#include "Display_GFX.h"

extern uint16_t window_x0, window_x1, window_y0, window_y1;

void ST7735_drawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
	uint16_t height = window_y1 - window_y0 + 1;
	uint16_t width = window_x1 - window_x0 + 1;

	// Edge rejection (no-draw if totally off canvas)
	if ((x < 0) || (x >= width) || (y >= height) || ((y + h - 1) < 0)) {
		return;
	}

	if (y < 0) { // Clip top
		h += y;
		y = 0;
	}
	if (y + h > height) { // Clip bottom
		h = height - y;
	}

	for (uint16_t i = 0; i < h; i++) {
		ST7735_DrawPixel(x, y + i, color);
	}
}

void ST7735_drawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
	uint16_t height = window_y1 - window_y0 + 1;
	uint16_t width = window_x1 - window_x0 + 1;

	// Edge rejection (no-draw if totally off canvas)
	if ((y < 0) || (y >= height) || (x >= width) || ((x + w - 1) < 0)) {
		return;
	}

	if (x < 0) { // Clip left
		w += x;
		x = 0;
	}
	if (x + w >= width) { // Clip right
		w = width - x;
	}

	for (uint16_t i = 0; i < w; i++) {
		ST7735_DrawPixel(x + i, y, color);
	}
}

void ST7735_drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {

	if (x0 == x1) {
		if (y0 > y1)
			_swap_uint16_t(&y0, &y1);
		ST7735_drawVLine(x0, y0, y1 - y0 + 1, color);
	}
	else if (y0 == y1) {
		if (x0 > x1)
			_swap_uint16_t(&x0, &x1);
		ST7735_drawHLine(x0, y0, x1 - x0 + 1, color);
	}
	else {
		//Ne prostoi variant
		int16_t steep = abs(y1 - y0) > abs(x1 - x0);
		if (steep) {
			_swap_uint16_t(&x0, &y0);
			_swap_uint16_t(&x1, &y1);
		}

		if (x0 > x1) {
			_swap_uint16_t(&x0, &x1);
			_swap_uint16_t(&y0, &y1);
		}

		int16_t dx, dy;
		dx = x1 - x0;
		dy = abs(y1 - y0);

		int16_t err = dx / 2;
		int16_t ystep;

		if (y0 < y1) {
			ystep = 1;
		}
		else {
			ystep = -1;
		}

		for (; x0 <= x1; x0++) {
			if (steep) {
				ST7735_DrawPixel(y0, x0, color);
			}
			else {
				ST7735_DrawPixel(x0, y0, color);
			}
			err -= dy;
			if (err < 0) {
			  y0 += ystep;
			  err += dx;
			}
		}
	}
}

void ST7735_drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	ST7735_drawHLine(x, y, w, color);
	ST7735_drawHLine(x, y + h - 1, w, color);
	ST7735_drawVLine(x, y, h, color);
	ST7735_drawVLine(x + w - 1, y, h, color);
}

void ST7735_drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color){
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7735_DrawPixel(x0, y0 + r, color);
	ST7735_DrawPixel(x0, y0 - r, color);
	ST7735_DrawPixel(x0 + r, y0, color);
	ST7735_DrawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
		  y--;
		  ddF_y += 2;
		  f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7735_DrawPixel(x0 + x, y0 + y, color);
		ST7735_DrawPixel(x0 - x, y0 + y, color);
		ST7735_DrawPixel(x0 + x, y0 - y, color);
		ST7735_DrawPixel(x0 - x, y0 - y, color);
		ST7735_DrawPixel(x0 + y, y0 + x, color);
		ST7735_DrawPixel(x0 - y, y0 + x, color);
		ST7735_DrawPixel(x0 + y, y0 - x, color);
		ST7735_DrawPixel(x0 - y, y0 - x, color);
	}
}

void ST7735_drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y) {
		if (f >= 0) {
		  y--;
		  ddF_y += 2;
		  f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		if (cornername & 0x4) {
		  ST7735_DrawPixel(x0 + x, y0 + y, color);
		  ST7735_DrawPixel(x0 + y, y0 + x, color);
		}
		if (cornername & 0x2) {
		  ST7735_DrawPixel(x0 + x, y0 - y, color);
		  ST7735_DrawPixel(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8) {
		  ST7735_DrawPixel(x0 - y, y0 + x, color);
		  ST7735_DrawPixel(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1) {
		  ST7735_DrawPixel(x0 - y, y0 - x, color);
		  ST7735_DrawPixel(x0 - x, y0 - y, color);
		}
	}
}

void ST7735_fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
	ST7735_drawVLine(x0, y0 - r, 2 * r + 1, color);
	ST7735_fillCircleHelper(x0, y0, r, 3, 0, color);
}

void ST7735_fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t corners, int16_t delta, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;
	int16_t px = x;
	int16_t py = y;

	delta++; // Avoid some +1's in the loop

	while (x < y) {
		if (f >= 0) {
		  y--;
		  ddF_y += 2;
		  f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		// These checks avoid double-drawing certain lines, important
		if (x < (y + 1)) {
		  if (corners & 1)
			  ST7735_drawVLine(x0 + x, y0 - y, 2 * y + delta, color);
		  if (corners & 2)
			  ST7735_drawVLine(x0 - x, y0 - y, 2 * y + delta, color);
		}
		if (y != py) {
		  if (corners & 1)
			  ST7735_drawVLine(x0 + py, y0 - px, 2 * px + delta, color);
		  if (corners & 2)
			  ST7735_drawVLine(x0 - py, y0 - px, 2 * px + delta, color);
		  py = y;
		}
		px = x;
	}
}

void ST7735_drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	ST7735_drawLine(x0, y0, x1, y1, color);
	ST7735_drawLine(x1, y1, x2, y2, color);
	ST7735_drawLine(x2, y2, x0, y0, color);
}

void ST7735_fillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1) {
		_swap_uint16_t(&y0, &y1);
		_swap_uint16_t(&x0, &x1);
	}
	if (y1 > y2) {
		_swap_uint16_t(&y2, &y1);
		_swap_uint16_t(&x2, &x1);
	}
	if (y0 > y1) {
		_swap_uint16_t(&y0, &y1);
		_swap_uint16_t(&x0, &x1);
	}

	if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if (x1 < a)
			a = x1;
		else if (x1 > b)
			b = x1;
		if (x2 < a)
			a = x2;
		else if (x2 > b)
			b = x2;
		ST7735_drawHLine(a, y0, b - a + 1, color);
		return;
	}

	int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 = x2 - x1, dy12 = y2 - y1;
	int32_t sa = 0, sb = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if (y1 == y2)
		last = y1; // Include y1 scanline
	else
		last = y1 - 1; // Skip it

	for (y = y0; y <= last; y++) {
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		if (a > b)
		  _swap_int16_t(&a, &b);
		ST7735_drawHLine(a, y, b - a + 1, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = (int32_t)dx12 * (y - y1);
	sb = (int32_t)dx02 * (y - y0);
	for (; y <= y2; y++) {
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if (a > b)
		  _swap_int16_t(&a, &b);
		ST7735_drawHLine(a, y, b - a + 1, color);
	}
}

void ST7735_drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
	int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
	if (r > max_radius)
		r = max_radius;
	ST7735_drawHLine(x + r, y, w - 2 * r, color);         // Top
	ST7735_drawHLine(x + r, y + h - 1, w - 2 * r, color); // Bottom
	ST7735_drawVLine(x, y + r, h - 2 * r, color);         // Left
	ST7735_drawVLine(x + w - 1, y + r, h - 2 * r, color); // Right
	// draw four corners
	ST7735_drawCircleHelper(x + r, y + r, r, 1, color);
	ST7735_drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
	ST7735_drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
	ST7735_drawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

void ST7735_fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
	int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
	if (r > max_radius)
		r = max_radius;
	ST7735_FillRectangle(x + r, y, w - 2 * r, h, color);
	// draw four corners
	ST7735_fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
	ST7735_fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

_Bool ST7735_setTextArea(text_area_t *area, uint16_t x, uint16_t y, uint16_t w, uint16_t h, FontDef *font) {
		if (x + w >= window_x1 - window_x0 + 1) return 0;
		if (y + h >= window_y1 - window_y0 + 1) return 0;

		area->x = x;
		area->y = y;
		area->w = w;
		area->h = h;
		area->font = font;
	return 1;
}

_Bool ST7735_writeToTextArea(text_area_t *area, char *text, uint16_t color, uint16_t bgcolor) {
	uint16_t x = area->x;
	uint16_t y = area->y;
	while(*text) {
		if(x + area->font->width >= area->x + area->w) {
			x = area->x;
			y += area->font->height;
			if(y + area->font->height >= area->y + area->h) {
				return 0;
			}

			if(*text == ' ') {
				// skip spaces in the beginning of the new line
				text++;
				continue;
			}
		}

		ST7735_WriteChar(x, y, *text, *(area->font), color, bgcolor);
		x += area->font->width;
		text++;
	}
	return 1;
}

