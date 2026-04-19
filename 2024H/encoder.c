#include "encoder.h"
#include <ti/devices/msp/msp.h>

// 实例化全局数据中心变量
Encoder_Data_t g_Encoder = {0};

// 内部软件计数器 (volatile 防止编译器过度优化)
static volatile int32_t left_pulse_count = 0;
static volatile int32_t right_pulse_count = 0;

/**
 * @brief 编码器底层初始化
 */
void Encoder_Init(void) {
    // 使用 MSPM0 标准的 GPIOB 中断宏
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/**
 * @brief GPIO 中断服务函数 (正交解码核心)
 * @note  处理 PB7(左轮A) 和 PB6(右轮A) 的双边沿中断
 */
void GROUP1_IRQHandler(void) {
    // 获取中断状态寄存器 (同时检查 GPIOB)
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);

    // --- 左轮正交解码 (PB7 作为 A 相中断源) ---
    if (status & DL_GPIO_PIN_7) {
        // 使用 !! 确保将位掩码转换为逻辑 0 或 1，防止位位置不同导致的比较错误
        bool phase_a = !!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7));
        bool phase_b = !!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9));
        
        // 修正：对调 ++ 和 -- 以修正极性
        if (phase_a != phase_b) left_pulse_count--;
        else left_pulse_count++;
        
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
    }

    // --- 右轮正交解码 (PB6 作为 A 相中断源) ---
    if (status & DL_GPIO_PIN_6) {
        bool phase_a = !!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6));
        bool phase_b = !!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8));
        
        // 修正：对调 ++ 和 -- 以修正极性
        if (phase_a != phase_b) right_pulse_count++;
        else right_pulse_count--;
        
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6);
    }
}

/**
 * @brief 10ms 数据搬运与解算函数
 * @note  必须在 10ms 定时器中断中调用
 */
void Encoder_UpdateData_10ms(void) {
    // 1. 提取当前速度增量
    g_Encoder.speed_left = left_pulse_count;
    g_Encoder.speed_right = right_pulse_count;
    
    // 2. 累加总脉冲数 (在清零前累加)
    g_Encoder.pulses_left += g_Encoder.speed_left;
    g_Encoder.pulses_right += g_Encoder.speed_right;

    // 3. 清零计数器
    left_pulse_count = 0;
    right_pulse_count = 0;

    // 4. 物理量换算 (厘米解算)
    // 距离计算公式：(左脉冲 + 右脉冲) / 2.0 * 系数
    // 注意：右轮已经取反，所以这里用加法
    float avg_pulses = (float)(g_Encoder.pulses_left + g_Encoder.pulses_right) / 2.0f;
    g_Encoder.distance_cm = avg_pulses * PULSE_TO_CM;
}

/**
 * @brief 清除里程计 (用于盲走 100cm 任务开始前)
 */
void Encoder_Clear(void) {
    g_Encoder.pulses_left = 0;
    g_Encoder.pulses_right = 0;
    g_Encoder.distance_cm = 0.0f;
    left_pulse_count = 0;
    right_pulse_count = 0;
}