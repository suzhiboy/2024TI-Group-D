#include "motor.h"

void Motor_Init(void)
{
    // 启动定时器 PWM 输出 (TIMG0)
    DL_TimerG_startCounter(PWM_MOTOR_INST);
    
    // 使能 TB6612 (STBY 置高)
    DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_STBY_PIN);
}

void Set_Motor_Speed_Left(int16_t speed)
{
    if (speed > 1600) speed = 1600;
    if (speed < -1600) speed = -1600;

    if (speed > 0) {
        // 正转
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_0_INDEX);
    } else if (speed < 0) {
        // 反转
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)(-speed), DL_TIMER_CC_0_INDEX);
    } else {
        // 停止
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_AIN1_PIN | GPIO_MOTOR_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    }
}

void Set_Motor_Speed_Right(int16_t speed)
{
    if (speed > 1600) speed = 1600;
    if (speed < -1600) speed = -1600;

    if (speed > 0) {
        // 正转
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)speed, DL_TIMER_CC_1_INDEX);
    } else if (speed < 0) {
        // 反转
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, (uint32_t)(-speed), DL_TIMER_CC_1_INDEX);
    } else {
        // 停止
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_BIN1_PIN | GPIO_MOTOR_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
    }
}
