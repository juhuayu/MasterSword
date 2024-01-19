#ifndef _WS2812_H
#define _WS2812_H

#include "main.h"


#define PIXEL_NUM		33
#define NUM				(24*PIXEL_NUM + 300)        // Reset 280us / 1.25us = 224
#define WS1				55
#define WS0				24


typedef enum {
	DYN_RGB=1,
    DYN_RED,
    DYN_ORANGE,
    DYN_YELLOW,
    DYN_GREE,
    DYN_CYAN,
    DYN_BLUE,
    DYN_PURPLE,
    STATICK_RED , 
    STATICK_ORANGE,
    STATICK_YELLOW,
    STATICK_GREEN,
    STATICK_CYAN,
    STATICK_BLUE,
    STATICK_PURPLE,
    STATICK_WHITE,
    STATICK_BLACK
} RGB_LED_EFFECT;
extern RGB_LED_EFFECT RGB_Eff;




extern uint16_t send_Buf[NUM];

void WS_Load(void);
void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B);
void WS_CloseAll(void);

uint32_t WS2812_Color(uint8_t red, uint8_t green, uint8_t blue);
void WS2812_SetPixelColor(uint16_t n, uint32_t GRBColor);
void WS2812_SetPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue);

uint32_t Wheel(uint8_t WheelPos);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);

#endif

