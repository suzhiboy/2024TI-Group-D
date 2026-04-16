#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"

// OLED 的 7 位 I2C 地址
#define OLED_ADDR           0x3C

// 【关键修改点】：PA0/PA1 对应的是 I2C_1_INST (即硬件 I2C0)
#define OLED_I2C_INSTANCE   I2C_1_INST 

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);

#endif