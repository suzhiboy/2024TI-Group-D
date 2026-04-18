#include "oled.h"
#include "Oled_Font.h"
#include "delay.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/* --- I2C 底层操作函数 --- */
static void I2C_Delay(void){
    delay_us(5); // 增加延时，提高软件 I2C 在 80MHz 主频下的稳定性
}

static void I2C_Start(void){
    OLED_SDA_OUT();
    OLED_SDA_Set();
    OLED_SCL_Set();
    I2C_Delay();
    OLED_SDA_Clr();
    I2C_Delay();
    OLED_SCL_Clr();
    I2C_Delay();
}

static void I2C_Stop(void){
    OLED_SDA_OUT();
    OLED_SCL_Clr();
    OLED_SDA_Clr();
    I2C_Delay();
    OLED_SCL_Set();
    I2C_Delay();
    OLED_SDA_Set();
    I2C_Delay();
}

static uint8_t I2C_WaitAck(void){
    OLED_SCL_Clr();
    OLED_SDA_IN();
    I2C_Delay();
    OLED_SCL_Set();
    I2C_Delay();
    uint8_t ack = OLED_READ_SDA();
    OLED_SCL_Clr();
    I2C_Delay();
    OLED_SDA_OUT();
    return ack;
}

static void I2C_Send_Byte(uint8_t dat){
    uint8_t i;
    OLED_SDA_OUT();
    for(i=0; i<8; i++)
    {
        OLED_SCL_Clr();
        I2C_Delay();
        if(dat & 0x80) OLED_SDA_Set();
        else OLED_SDA_Clr();
        dat <<= 1;
        I2C_Delay();
        OLED_SCL_Set();
        I2C_Delay();
    }
    OLED_SCL_Clr();
    I2C_Delay();
}

/* --- OLED 操作函数 --- */
uint8_t OLED_GRAM[144][8];

void OLED_WR_Byte(uint8_t dat, uint8_t mode){
    I2C_Start();
    I2C_Send_Byte(0x78); // OLED 地址
    I2C_WaitAck();
    if(mode) I2C_Send_Byte(0x40);
    else I2C_Send_Byte(0x00);
    I2C_WaitAck();
    I2C_Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

void OLED_Refresh(void){
    uint8_t i, n;
    for(i=0; i<8; i++)
    {
        OLED_WR_Byte(0xb0+i, OLED_CMD); // 设置页地址
        OLED_WR_Byte(0x00, OLED_CMD);   // 设置低列起始地址
        OLED_WR_Byte(0x10, OLED_CMD);   // 设置高列起始地址
        
        I2C_Start();
        I2C_Send_Byte(0x78);
        I2C_WaitAck();
        I2C_Send_Byte(0x40);
        I2C_WaitAck();
        for(n=0; n<128; n++)
        {
            I2C_Send_Byte(OLED_GRAM[n][i]);
            I2C_WaitAck(); // 每一字节后必须等待应答信号
        }
        I2C_Stop();
    }
}

void OLED_Clear(void){
    uint8_t i, n;
    for(i=0; i<8; i++)
        for(n=0; n<128; n++)
            OLED_GRAM[n][i] = 0;
    OLED_Refresh();
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t){
    if (x > 127 || y > 63) return;
    if(t) OLED_GRAM[x][y/8] |= (1 << (y % 8));
    else OLED_GRAM[x][y/8] &= ~(1 << (y % 8));
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode){
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0=x, y0=y;
    if(size1==16) size2=16;
    else return;
    chr1 = chr - ' ';
    for(i=0; i<size2; i++)
    {
        temp = OLED_F8x16[chr1][i];
        for(m=0; m<8; m++)
        {
            if(temp & 0x01) OLED_DrawPoint(x, y, mode);
            else OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        x++;
        if((x-x0) == 8){ x=x0; y0=y0+8; }
        y=y0;
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode){
    while((*chr >= ' ') && (*chr <= '~'))
    {
        OLED_ShowChar(x, y, *chr, size1, mode);
        x += 8;
        chr++;
    }
}

uint32_t OLED_Pow(uint8_t m, uint8_t n){
    uint32_t result = 1;
    while(n--) result *= m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode){
    uint8_t t, temp;
    for(t=0; t<len; t++)
    {
        temp = (num / OLED_Pow(10, len-t-1)) % 10;
        OLED_ShowChar(x + 8*t, y, temp + '0', size1, mode);
    }
}

void OLED_ShowFloatNum(uint8_t x, uint8_t y, double num, uint8_t intlen, uint8_t fralen, uint8_t size1, uint8_t mode){
    uint32_t PowNum, IntNum, FraNum;
    if (num < 0) { OLED_ShowChar(x, y, '-', size1, mode); num = -num; }
    else { OLED_ShowChar(x, y, '+', size1, mode); }
    
    IntNum = (uint32_t)num;
    num -= IntNum;
    PowNum = OLED_Pow(10, fralen);
    FraNum = round(num * PowNum);
    IntNum += FraNum / PowNum;
    FraNum %= PowNum;

    OLED_ShowNum(x + 8, y, IntNum, intlen, size1, mode);
    OLED_ShowChar(x + 8 + 8*intlen, y, '.', size1, mode);
    OLED_ShowNum(x + 16 + 8*intlen, y, FraNum, fralen, size1, mode);
}

void OLED_Init(void){
    // 初始化引脚：推挽输出
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_enableOutput(OLED_PORT, OLED_SCL_PIN);
    
    OLED_SDA_OUT();
    
    OLED_SCL_Set();
    OLED_SDA_Set();
    delay_ms(100);

    OLED_WR_Byte(0xAE, OLED_CMD); // 关闭显示
    OLED_WR_Byte(0x00, OLED_CMD); // 设置低列地址
    OLED_WR_Byte(0x10, OLED_CMD); // 设置高列地址
    OLED_WR_Byte(0x40, OLED_CMD); // 设置起始行
    OLED_WR_Byte(0x81, OLED_CMD); // 对比度
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD); // 段重映射
    OLED_WR_Byte(0xC8, OLED_CMD); // COM 扫描方向
    OLED_WR_Byte(0xA6, OLED_CMD); // 正常显示
    OLED_WR_Byte(0xA8, OLED_CMD); // 多路复用
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD); // 显示偏移
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD); // 时钟分频
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD); // 预充电
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD); // COM 引脚配置
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); // VCOMH
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD); // 寻址模式
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD); // 电荷泵
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); // 开启显示
}

void OLED_Update(void){
    OLED_Refresh();
}

void OLED_Show_String(uint8_t han, uint8_t lie, uint8_t *string){
    OLED_ShowString((lie-1)*8, (han-1)*16, string, 16, 1);
    OLED_Update();
}
