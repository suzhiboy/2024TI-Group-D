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
    // 1. 显式清除 GPIOB 可能存在的残留中断标志
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9);
    
    // 2. 强制开启引脚的中断使能 (防止 SysConfig 没勾选使能)
    DL_GPIO_enableInterrupt(GPIOB, DL_GPIO_PIN_6 | DL_GPIO_PIN_7);
    
    // 3. 开启 GPIOB 的 NVIC 中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/**
 * @brief GPIO 中断服务函数 (正交解码核心)
 * @note  处理 PB7(左轮A) 和 PB6(右轮A) 的双边沿中断
 */
void GROUP1_IRQHandler(void) {
    // 1. 获取分组中断挂起状态
    uint32_t pendingGroup = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    // 2. 判断是否为 GPIOB 触发
    if (pendingGroup & DL_INTERRUPT_GROUP1_IIDX_GPIOB) {
        // 获取 GPIOB 所有已使能中断引脚的状态
        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, 0xFFFFFFFF);

        // --- 左轮正交解码 (PB7 作为 A 相中断源) ---
        if (gpio_status & DL_GPIO_PIN_7) {
            // 修正：确保读取结果转换为标准的 0 或 1
            uint8_t phase_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7) ? 1 : 0;
            uint8_t phase_b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9) ? 1 : 0;
            
            if (phase_a != phase_b) left_pulse_count++;
            else left_pulse_count--;
            
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
        }

        // --- 右轮正交解码 (PB6 作为 A 相中断源) ---
        if (gpio_status & DL_GPIO_PIN_6) {
            uint8_t phase_a = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6) ? 1 : 0;
            uint8_t phase_b = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8) ? 1 : 0;
            
            // 修正：如果你的小车前进时里程不增反减，请将这里的 ++ 和 -- 对换
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
    
    left_pulse_count = 0;
    right_pulse_count = 0;

    g_Encoder.pulses_left += g_Encoder.speed_left;
    g_Encoder.pulses_right += g_Encoder.speed_right;

    // 改进：取绝对值的平均值计算行驶路程，防止正负抵消导致里程为0
    // 注意：如果是为了做位移计算（倒车减里程），则去掉 fabsf
    float dist_l = (float)g_Encoder.pulses_left;
    float dist_r = (float)g_Encoder.pulses_right;
    
    // 如果左右轮安装方向相反，前进时一正一负，这里取绝对值累加
    float total_pulses = (float)(abs(g_Encoder.pulses_left) + abs(g_Encoder.pulses_right)) / 2.0f;
    g_Encoder.distance_cm = total_pulses * PULSE_TO_CM;
}

void Encoder_Clear(void) {
    g_Encoder.pulses_left = 0;
    g_Encoder.pulses_right = 0;
    g_Encoder.distance_cm = 0.0f;
    left_pulse_count = 0;
    right_pulse_count = 0;
}
