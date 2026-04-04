#ifndef __BT_UART_H
#define __BT_UART_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

// 蓝牙数据接收结构体（建议并入你之前的全局数据中心）
typedef struct {
    uint8_t  last_command;  // 存放最新收到的一个字节指令
    bool     is_updated;    // 标志位：告诉逻辑层有新指令来了
} BT_Data_t;

extern BT_Data_t g_Bluetooth;

void BT_UART_Init(void);
void BT_SendString(char *str);

#endif