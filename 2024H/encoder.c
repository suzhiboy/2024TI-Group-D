#include "encoder.h"

// 软件计数变量
volatile int32_t left_count = 0;
volatile int32_t right_count = 0;

// 处理所有 GPIO 中断
void GROUP1_IRQHandler(void) {
    // 获取中断源
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFF) | 
                      DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFF);

    // --- 左轮逻辑 (PB7 边沿触发) ---
    if (status & DL_GPIO_PIN_7) {
        // 读取 B 相 PB9
        bool phase_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7);
        bool phase_b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9);
        // 正交解码逻辑：A相跳变时，若A!=B则正向，A==B则反向（取决于物理接线）
        if (phase_a != phase_b) left_count++;
        else left_count--;
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
    }

    // --- 右轮逻辑 (PB6 边沿触发) ---
    if (status & DL_GPIO_PIN_6) {
        // 读取 B 相 PB8
        bool phase_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6);
        bool phase_b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8);
        if (phase_a != phase_b) right_count++;
        else right_count--;
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6);
    }
}

void Encoder_UpdateData_10ms(void) {
    // 成员 A 的搬运工作：将中断累加的脉冲数交给全局变量，然后清零
    g_Encoder.speed_left = left_count;
    g_Encoder.speed_right = right_count;
    
    left_count = 0;
    right_count = 0;

    g_Encoder.distance_left += g_Encoder.speed_left;
    g_Encoder.distance_right += g_Encoder.speed_right;
}