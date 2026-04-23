#include "encoder.h"
#include "control.h"
#include "mpu6050.h"
#include <ti/devices/msp/msp.h>

// 实例化全局数据中心变量
Encoder_Data_t g_Encoder = {0};

// 内部软件计数器
static volatile int32_t left_pulse_count = 0;
static volatile int32_t right_pulse_count = 0;

/**
 * @brief 编码器与按键统一初始化
 */
void Encoder_Init(void) {
    // 1. 显式清除 GPIOB 可能存在的残留中断标志 (编码器引脚 + 按键引脚)
    DL_GPIO_clearInterruptStatus(GPIOB, 
        DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_12 | DL_GPIO_PIN_13);
    
    // 2. 强制开启引脚的中断使能
    DL_GPIO_enableInterrupt(GPIOB, 
        DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | 
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_12 | DL_GPIO_PIN_13);
    
    // 3. 开启 GPIOB 的 NVIC 中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

/**
 * @brief GPIO 统一中断服务函数 (负责 GPIOB 组的所有硬件响应)
 */
void GROUP1_IRQHandler(void) {
    // 1. 获取分组中断挂起状态
    uint32_t pendingGroup = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    // 2. 判断是否为 GPIOB 触发
    if (pendingGroup & DL_INTERRUPT_GROUP1_IIDX_GPIOB) {
        // 获取 GPIOB 所有已触发且使能的中断标志
        uint32_t gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, 
            DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | 
            DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_12 | DL_GPIO_PIN_13);

        /* --- A. 编码器处理 (PB7, PB6) --- */
        if (gpio_status & DL_GPIO_PIN_7) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_7);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_9);
            if (phase_a != phase_b) left_pulse_count--; else left_pulse_count++;
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_7);
        }
        if (gpio_status & DL_GPIO_PIN_6) {
            uint8_t phase_a = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_6);
            uint8_t phase_b = !!DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8);
            if (phase_a != phase_b) right_pulse_count++; else right_pulse_count--;
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_6);
        }

        /* --- B. 按键处理 (PB3, PB13, PB12, PB2) --- */
        // 1. PB3 - 任务 1
        if (gpio_status & DL_GPIO_PIN_3) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset(); 
                g_target_task = TASK_1_AB_STRAIGHT; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_3);
        }
        // 2. PB13 - 任务 2
        else if (gpio_status & DL_GPIO_PIN_13) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();
                g_target_task = TASK_2_ABCD_CIRCLE; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_13);
        }
        // 3. PB12 - 任务 3
        else if (gpio_status & DL_GPIO_PIN_12) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();
                g_target_task = TASK_3_ACBD_DIAGONAL; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_12);
        }
        // 4. PB2 - 任务 4
        else if (gpio_status & DL_GPIO_PIN_2) {
            if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) {
                Control_Reset(); Yaw_Reset();
                g_target_task = TASK_4_FOUR_LAPS; Car_Mode = TASK_CALIBRATING;
            } else { Car_Mode = TASK_IDLE; Control_Reset(); }
            DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_PIN_2);
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
