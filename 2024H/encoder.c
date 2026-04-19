#include "encoder.h"
#include <ti/devices/msp/msp.h>

// 实例化全局数据中心变量
Encoder_Data_t g_Encoder = {0};

// 内部软件计数器
static volatile int32_t left_pulse_count = 0;
static volatile int32_t right_pulse_count = 0;

/**
 * @brief 编码器底层初始化
 */
void Encoder_Init(void) {
    // 1. 显式清除 GPIOB 可能存在的残留中断标志
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9);
    
    // 2. 强制开启引脚的中断使能
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
    
    // 3. 开启 GPIOB 的 NVIC 中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/**
 * @brief GPIO 中断服务函数 (精简后的正交解码核心)
 */
void GROUP1_IRQHandler(void) {
    // 1. 获取分组中断挂起状态
    uint32_t pendingGroup = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    // 2. 判断是否为 GPIOB 触发
    if (pendingGroup & DL_INTERRUPT_GROUP1_IIDX_GPIOB) {
        // 获取 GPIOB 已触发的中断标志
        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);

        // --- 左轮正交解码 (PB7) ---
        if (gpio_status & DL_GPIO_PIN_7) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9);
            
            // 修正后的左轮极性
            if (phase_a != phase_b) left_pulse_count--;
            else left_pulse_count++;
            
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
        }

        // --- 右轮正交解码 (PB6) ---
        if (gpio_status & DL_GPIO_PIN_6) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8);
            
            // 修正后的右轮极性
            if (phase_a != phase_b) right_pulse_count++;
            else right_pulse_count--;
            
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6);
        }
    }
}

/**
 * @brief 10ms 数据搬运与解算函数
 */
void Encoder_UpdateData_10ms(void) {
    g_Encoder.speed_left = left_pulse_count;
    g_Encoder.speed_right = right_pulse_count;
    
    g_Encoder.pulses_left += g_Encoder.speed_left;
    g_Encoder.pulses_right += g_Encoder.speed_right;

    left_pulse_count = 0;
    right_pulse_count = 0;

    float avg_pulses = (float)(g_Encoder.pulses_left + g_Encoder.pulses_right) / 2.0f;
    g_Encoder.distance_cm = avg_pulses * PULSE_TO_CM;
}

void Encoder_Clear(void) {
    g_Encoder.pulses_left = 0;
    g_Encoder.pulses_right = 0;
    g_Encoder.distance_cm = 0.0f;
    left_pulse_count = 0;
    right_pulse_count = 0;
}
