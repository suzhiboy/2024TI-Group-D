#include "encoder.h"

// 实例化全局数据中心变量
Encoder_Data_t g_Encoder = {0};

// 内部软件计数器 (volatile 防止编译器过度优化)
static volatile int32_t left_pulse_count = 0;
static volatile int32_t right_pulse_count = 0;

/**
 * @brief 编码器底层初始化
 */
void Encoder_Init(void) {
    // 开启 GPIOB 的中断（针对 PB6, PB7）
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/**
 * @brief GPIO 中断服务函数 (正交解码核心)
 * @note  处理 PB7(左轮A) 和 PB6(右轮A) 的双边沿中断
 */
void GROUP1_IRQHandler(void) {
    // 获取中断状态寄存器
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIOA, 0xFFFFFFFF) | 
                      DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFF);

    // --- 左轮正交解码 (PB7 作为 A 相中断源) ---
    if (status & DL_GPIO_PIN_7) {
        bool phase_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7);
        bool phase_b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9);
        
        // 正交解码算法：A相发生跳变时，若 A != B 则为正转，反之为反转
        if (phase_a != phase_b) left_pulse_count++;
        else left_pulse_count--;
        
        DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
    }

    // --- 右轮正交解码 (PB6 作为 A 相中断源) ---
    if (status & DL_GPIO_PIN_6) {
        bool phase_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6);
        bool phase_b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8);
        
        // 注意：由于左右电机镜像对称安装，通常有一边需要反向计数
        // 这里假设右轮逻辑与左轮相同，调试时若发现推车距离减小，请将下方的 ++/-- 互换
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
    // 1. 提取当前速度增量并清零计数器 (原子操作模拟)
    g_Encoder.speed_left = left_pulse_count;
    g_Encoder.speed_right = right_pulse_count;
    
    left_pulse_count = 0;
    right_pulse_count = 0;

    // 2. 累加总脉冲数
    g_Encoder.pulses_left += g_Encoder.speed_left;
    g_Encoder.pulses_right += g_Encoder.speed_right;

    // 3. 物理量换算 (厘米解算)
    // 距离 = 左右平均脉冲数 * 转换系数
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