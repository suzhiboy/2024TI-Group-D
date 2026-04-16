#include "oled.h"
#include "OLED_Font.h"
#include "delay.h"

// --- 硬件 I2C 底层写函数 ---
static void I2C_Write_Byte(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};

    // 1. 将控制字节和数据填充到发送 FIFO
    // 【修复】：使用了正确的 MSPM0 官方库函数名 DL_I2C_fillControllerTXFIFO
    DL_I2C_fillControllerTXFIFO(OLED_I2C_INSTANCE, &buffer[0], 2);
    
    // 2. 发送开始信号并启动传输 (发送 2 个字节)
    DL_I2C_startControllerTransfer(OLED_I2C_INSTANCE, OLED_ADDR, 
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    
    // 3. 轮询等待控制器状态不再是 BUSY（即等待传输完全结束）
    while (DL_I2C_getControllerStatus(OLED_I2C_INSTANCE) & DL_I2C_CONTROLLER_STATUS_BUSY);
}

// --- 发送命令 ---
void OLED_Write_Cmd(unsigned char IIC_Cmd) {
    I2C_Write_Byte(0x00, IIC_Cmd); // 控制字节 0x00 表示接下来的字节是命令
}

// --- 发送数据 ---
void OLED_Write_Data(unsigned char IIC_Data) {
    I2C_Write_Byte(0x40, IIC_Data); // 控制字节 0x40 表示接下来的字节是数据
}

// --- 设置光标坐标 ---
void OLED_Set_Pos(unsigned char x, unsigned char y) {
    OLED_Write_Cmd(0xb0 + y);
    OLED_Write_Cmd(((x & 0xf0) >> 4) | 0x10);
    OLED_Write_Cmd((x & 0x0f) | 0x01);
}

// --- 显示 8x16 字符 ---
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr) {
    uint8_t c = chr - ' '; 
    if (x > 120) { x = 0; y = y + 2; }
    
    OLED_Set_Pos(x, y);
    for (uint8_t i = 0; i < 8; i++) OLED_Write_Data(OLED_F8x16[c][i]); 
    
    OLED_Set_Pos(x, y + 1);
    for (uint8_t i = 0; i < 8; i++) OLED_Write_Data(OLED_F8x16[c][i + 8]); 
}

// --- 显示字符串 ---
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr) {
    uint8_t j = 0;
    while (chr[j] != '\0') {
        OLED_ShowChar(x, y, chr[j]);
        x += 8;
        if (x > 120) { x = 0; y += 2; }
        j++;
    }
}

// --- 显示 16x16 中文字符 ---
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no) {      			    
	uint8_t t, adder = 0;
	OLED_Set_Pos(x, y);	 
	for(t = 0; t < 16; t++) {
		OLED_Write_Data(HZ16[no].Msk[adder]);
		adder++;
	}	
	OLED_Set_Pos(x, y + 1);
	for(t = 0; t < 16; t++) {
		OLED_Write_Data(HZ16[no].Msk[adder]);
		adder++;
	}					
}

// --- 清屏 ---
void OLED_Clear(void) {
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_Write_Cmd(0xb0 + i);
        OLED_Write_Cmd(0x00);
        OLED_Write_Cmd(0x10);
        for (n = 0; n < 128; n++) OLED_Write_Data(0);
    }
}

// --- OLED 初始化 ---
void OLED_Init(void) {
    // 替换为标准的毫秒延时，确保 OLED 内部控制器有足够的时间复位就绪
    delay_ms(100); 
    
    OLED_Write_Cmd(0xAE); // 关闭显示
    OLED_Write_Cmd(0x00); // 设置低列地址
    OLED_Write_Cmd(0x10); // 设置高列地址
    OLED_Write_Cmd(0x40); // 设置起始行地址
    OLED_Write_Cmd(0x81); // 设置对比度控制寄存器
    OLED_Write_Cmd(0xCF); // 对比度值
    OLED_Write_Cmd(0xA1); // 设置段重映射
    OLED_Write_Cmd(0xC8); // 设置 COM 输出扫描方向
    OLED_Write_Cmd(0xA6); // 设置正常显示 (A7为反显)
    OLED_Write_Cmd(0xA8); // 设置多路复用率
    OLED_Write_Cmd(0x3F); // 1/64 duty
    OLED_Write_Cmd(0xD3); // 设置显示偏移
    OLED_Write_Cmd(0x00); // 不偏移
    OLED_Write_Cmd(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_Write_Cmd(0x80); // 设置分频比
    OLED_Write_Cmd(0xD9); // 设置预充电周期
    OLED_Write_Cmd(0xF1); // 预充电配置
    OLED_Write_Cmd(0xDA); // 设置 COM 引脚硬件配置
    OLED_Write_Cmd(0x12);
    OLED_Write_Cmd(0xDB); // 设置 VCOMH 取消选择级别
    OLED_Write_Cmd(0x40); 
    OLED_Write_Cmd(0x8D); // 设置电荷泵
    OLED_Write_Cmd(0x14); // 开启电荷泵
    OLED_Write_Cmd(0xAF); // 开启 OLED 显示面板
    
    OLED_Clear();         // 初始清屏
}