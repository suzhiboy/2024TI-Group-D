#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"

// 严格匹配 ti_msp_dl_config.h 中的宏定义
#define OLED_SCL_HIGH()  DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_OLED_SCL_PIN)
#define OLED_SCL_LOW()   DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_OLED_SCL_PIN)
#define OLED_SDA_HIGH()  DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_OLED_SDA_PIN)
#define OLED_SDA_LOW()   DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_OLED_SDA_PIN)

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);

#endif