#include "bt_uart.h"
#include <string.h>

BT_Data_t g_Bluetooth = {0, false};

// 初始化蓝牙串口中断
void BT_UART_Init(void)
{
    // 清除可能残留的接收中断标志
    DL_UART_Main_clearInterruptStatus(UART_BLUETOOTH_INST, DL_UART_MAIN_INTERRUPT_RX);
    // 开启 UART 全局中断
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

// 阻塞式发送字符串（供成员 B 打印 PID 数据用）
void BT_SendString(char *str)
{
    uint16_t len = strlen(str);
    for(uint16_t i = 0; i < len; i++) {
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, str[i]);
    }
}

// =========================================================
// 成员 A 的专属领域：串口接收中断服务函数
// （注意：函数名必须和 SysConfig 生成的保持完全一致）
// =========================================================
void UART_BLUETOOTH_INST_IRQHandler(void)
{
    // 检查是不是接收中断 (RX Interrupt)
    switch(DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
            // 1. 读取接收到的这个字节（硬件会自动清除中断标志）
            uint8_t received_byte = DL_UART_Main_receiveData(UART_BLUETOOTH_INST);
            
            // 2. 存入全局数据中心
            g_Bluetooth.last_command = received_byte;
            
            // 3. 立起旗帜，通知逻辑层有新数据
            g_Bluetooth.is_updated = true;
            
            // ⚠️ 绝不越权！你只负责把数据存进 g_Bluetooth，
            // 绝对不要在这里写 if(received_byte == 'A') motor_run();
            break;
        }
        default:
            break;
    }
}