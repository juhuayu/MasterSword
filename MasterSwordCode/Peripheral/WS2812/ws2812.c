/**
  ******************************************************************************
  * @file           : ws2812.c
  * @brief          : ws2812 LED Driver
  ******************************************************************************
  * @attention
  *
  * 未经作者许可，不得用于其它任何用途
  * 创建日期:2024/1/2
	* 作者：juhua_yu
  * 版本：V1.0
  *
  ******************************************************************************
  */

#include "ws2812.h"
#include <string.h>
#include <stdio.h>
#include "tim.h"

uint16_t send_Buf[NUM];

//#define PROCESS_DELAY 5
//#define SECTION_DELAY 100

void WS_Load1(void) { HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, (uint32_t*)send_Buf, NUM); }
void WS_Load2(void) { HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, (uint32_t*)send_Buf, NUM); }
void WS_CloseAll(void)
{
	uint16_t i;
	for (i = 0; i < PIXEL_NUM * 24; i++)
		send_Buf[i] = WS0;    // 写入逻辑0的占空比
	for (i = PIXEL_NUM * 24; i < NUM; i++)
		send_Buf[i] = 0;    // 占空比比为0，全为低电平
	WS_Load1();
	WS_Load2();
}

void WS_WriteAll_RGB(uint8_t n_R, uint8_t n_G, uint8_t n_B)
{
	uint16_t i, j;
	uint8_t dat[24];
	// 将RGB数据进行转换
	for (i = 0; i < 8; i++)
	{
		dat[i] = ((n_G & 0x80) ? WS1 : WS0);
		n_G <<= 1;
	}
	for (i = 0; i < 8; i++)
	{
		dat[i + 8] = ((n_R & 0x80) ? WS1 : WS0);
		n_R <<= 1;
	}
	for (i = 0; i < 8; i++)
	{
		dat[i + 16] = ((n_B & 0x80) ? WS1 : WS0);
		n_B <<= 1;
	}
	for (i = 0; i < PIXEL_NUM; i++)
	{
		for (j = 0; j < 24; j++)
		{
			send_Buf[i * 24 + j] = dat[j];
		}
	}
	for (i = PIXEL_NUM * 24; i < NUM; i++)
		send_Buf[i] = 0;    // 占空比比为0，全为低电平
	WS_Load1();
	WS_Load2();
}

uint32_t WS2812_Color(uint8_t red, uint8_t green, uint8_t blue) { return green << 16 | red << 8 | blue; }

// n代表第n颗灯珠，如果只让其中一颗显示，其他的灯珠先要清零
void WS2812_SetPixelColor(uint16_t n, uint32_t GRBColor)
{
	uint8_t i;
	if (n < PIXEL_NUM)
	{
		for (i = 0; i < 24; ++i)
			send_Buf[24 * n + i] = (((GRBColor << i) & 0X800000) ? WS1 : WS0);
	}
}

void WS2812_SetPixelRGB(uint16_t n, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t i;

	if (n < PIXEL_NUM)
	{
		for (i = 0; i < 24; ++i)
			send_Buf[24 * n + i] = (((WS2812_Color(red, green, blue) << i) & 0X800000) ? WS1 : WS0);
	}
}

// RGB颜色渐变算法
uint32_t Wheel(uint8_t WheelPos)
{
	WheelPos = 255 - WheelPos;
	if (WheelPos < 85)
	{
		return WS2812_Color(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if (WheelPos < 170)
	{
		WheelPos -= 85;
		return WS2812_Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return WS2812_Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbow(uint8_t wait)
{
	uint32_t timestamp = HAL_GetTick();
	uint16_t i;
	static uint8_t j;
	static uint32_t next_time = 0;

	uint32_t flag = 0;
	if (next_time < wait)
	{
		if ((uint64_t)timestamp + wait - next_time > 0)
			flag = 1;
	}
	else if (timestamp > next_time)
	{
		flag = 1;
	}
	if (flag)    // && (timestamp - next_time < wait*5))
	{
		j++;
		next_time = timestamp + wait;
		for (i = 0; i < PIXEL_NUM; i++)
		{
			WS2812_SetPixelColor(i, Wheel((i + j) & 255));
		}
	}
	WS_Load1();
	WS_Load2();
}

void rainbowCycle(uint8_t wait)
{
	uint32_t timestamp = HAL_GetTick();
	uint16_t i;
	static uint8_t j;
	static uint32_t next_time = 0;

	static uint8_t loop = 0;
	if (loop == 0)
		next_time = timestamp;
	loop = 1;    //首次调用初始化

	if ((timestamp > next_time))    // && (timestamp - next_time < wait*5))
	{
		j++;
		next_time = timestamp + wait;
		for (i = 0; i < PIXEL_NUM; i++)
		{
			WS2812_SetPixelColor(i, Wheel(((i * 256 / (PIXEL_NUM)) + j) & 255));
		}
	}
	WS_Load2();
	WS_Load1();
	
	
}

