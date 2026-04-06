#include "oled.h"
#include "OLED_Font.h" // 引用你的字库文件

// --- 模拟 IIC 底层函数 ---
void IIC_Start(void) {
    OLED_SDA_HIGH(); OLED_SCL_HIGH();
    OLED_SDA_LOW();  OLED_SCL_LOW();
}

void IIC_Stop(void) {
    OLED_SCL_LOW();  OLED_SDA_LOW();
    OLED_SCL_HIGH(); OLED_SDA_HIGH();
}

void Write_IIC_Byte(unsigned char IIC_Byte) {
    unsigned char i, m, temp;
    temp = IIC_Byte;
    OLED_SCL_LOW();
    for (i = 0; i < 8; i++) {
        m = temp;
        if (m & 0x80) OLED_SDA_HIGH();
        else OLED_SDA_LOW();
        temp = temp << 1;
        OLED_SCL_HIGH(); 
        OLED_SCL_LOW();
    }
}

void OLED_Write_Cmd(unsigned char IIC_Cmd) {
    IIC_Start();
    Write_IIC_Byte(0x78);
    Write_IIC_Byte(0x00);
    Write_IIC_Byte(IIC_Cmd);
    IIC_Stop();
}

void OLED_Write_Data(unsigned char IIC_Data) {
    IIC_Start();
    Write_IIC_Byte(0x78);
    Write_IIC_Byte(0x40);
    Write_IIC_Byte(IIC_Data);
    IIC_Stop();
}

void OLED_Set_Pos(unsigned char x, unsigned char y) {
    OLED_Write_Cmd(0xb0 + y);
    OLED_Write_Cmd(((x & 0xf0) >> 4) | 0x10);
    OLED_Write_Cmd((x & 0x0f) | 0x01);
}

// --- 核心修改：适配你的 OLED_F8x16 字库 ---
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr) {
    uint8_t c = chr - ' '; 
    if (x > 120) { x = 0; y = y + 2; }
    
    OLED_Set_Pos(x, y);
    for (uint8_t i = 0; i < 8; i++) OLED_Write_Data(OLED_F8x16[c][i]); //
    
    OLED_Set_Pos(x, y + 1);
    for (uint8_t i = 0; i < 8; i++) OLED_Write_Data(OLED_F8x16[c][i + 8]); //
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr) {
    uint8_t j = 0;
    while (chr[j] != '\0') {
        OLED_ShowChar(x, y, chr[j]);
        x += 8;
        if (x > 120) { x = 0; y += 2; }
        j++;
    }
}

// 显示中文字符
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

void OLED_Clear(void) {
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_Write_Cmd(0xb0 + i);
        OLED_Write_Cmd(0x00);
        OLED_Write_Cmd(0x10);
        for (n = 0; n < 128; n++) OLED_Write_Data(0);
    }
}

void OLED_Init(void) {
    for(volatile int i=0; i<50000; i++); 
    OLED_Write_Cmd(0xAE); OLED_Write_Cmd(0x00); OLED_Write_Cmd(0x10); 
    OLED_Write_Cmd(0x40); OLED_Write_Cmd(0x81); OLED_Write_Cmd(0xCF); 
    OLED_Write_Cmd(0xA1); OLED_Write_Cmd(0xC8); OLED_Write_Cmd(0xA6); 
    OLED_Write_Cmd(0xA8); OLED_Write_Cmd(0x3F); OLED_Write_Cmd(0xD3); 
    OLED_Write_Cmd(0x00); OLED_Write_Cmd(0xD5); OLED_Write_Cmd(0x80); 
    OLED_Write_Cmd(0xD9); OLED_Write_Cmd(0xF1); OLED_Write_Cmd(0xDA); 
    OLED_Write_Cmd(0x12); OLED_Write_Cmd(0xDB); OLED_Write_Cmd(0x40); 
    OLED_Write_Cmd(0x8D); OLED_Write_Cmd(0x14); OLED_Write_Cmd(0xAF); 
    OLED_Clear();
}