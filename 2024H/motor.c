#include "motor.h"

// 记录左右电机上一次的速度，用于判断高速停机缓冲
static int16_t last_left = 0;
static int16_t last_right = 0;

void Motor_Init(void)
{
    // 启动定时器 PWM 输出 (TIMG0)
    DL_TimerG_startCounter(PWM_MOTOR_INST);
    
    // 原有的 STBY 拉高代码已移除，因为硬件已硬连 +5V
}

/**
 * @brief 设置左电机速度 (强制不倒退版)
 * @param speed 期望速度 (0 - 2000)
 */
void Set_Motor_Speed_Left(int16_t speed)
{
    // 1. 核心约束：严禁倒退 (电赛H题基本规则)
    if (speed < 0) speed = 0; 

    // 2. 高速停机缓冲
    if (last_left > 500 && speed == 0) {
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
        delay_cycles(16000); 
    }
    last_left = speed;

    if (speed > 2000) speed = 2000;

    if (speed > 0) {
        // 正转模式 (AIN1=1, AIN2=0)
        DL_GPIO_setPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_1_INDEX);
    } else {
        // 自由滑行停止
        DL_GPIO_clearPins(GPIO_MOTOR_AIN1_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_AIN2_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}

/**
 * @brief 设置右电机速度 (强制不倒退版)
 */
void Set_Motor_Speed_Right(int16_t speed)
{
    if (speed < 0) speed = 0;

    if (last_right > 500 && speed == 0) {
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
        delay_cycles(16000);
    }
    last_right = speed;

    if (speed > 2000) speed = 2000;

    if (speed > 0) {
        // 正转模式 (BIN1=0, BIN2=1)
        DL_GPIO_setPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_0_INDEX);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_BIN2_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_BIN1_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    }
}
