#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"

/**
 * @brief 按键扫描并启动对应任务
 * @note  Task 1: PB3, Task 2: PB13, Task 3: PB12, Task 4: PB2
 */
void Key_Scan_Proc(void)
{
    // 获取 GPIOB 的当前引脚电平状态
    uint32_t gpiob_state = DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3 | DL_GPIO_PIN_13 | DL_GPIO_PIN_12 | DL_GPIO_PIN_2);

    // --- 第一题: PB3 ---
    if (!(gpiob_state & DL_GPIO_PIN_3)) {
        delay_ms(20); // 消抖
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))) {
            Control_Reset();        
            mpu6050.Yaw = 0;        
            Reset_Encoder_Distance(); 
            Car_Mode = 1;           // TASK_1_AB_STRAIGHT
            while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))); 
        }
    }

    // --- 第二题: PB13 ---
    if (!(gpiob_state & DL_GPIO_PIN_13)) {
        delay_ms(20);
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13))) {
            Control_Reset();
            mpu6050.Yaw = 0;
            Reset_Encoder_Distance();
            Car_Mode = 2;           // TASK_2_ABCD_CIRCLE
            while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13)));
        }
    }

    // --- 第三题: PB12 ---
    if (!(gpiob_state & DL_GPIO_PIN_12)) {
        delay_ms(20);
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12))) {
            Control_Reset();
            mpu6050.Yaw = 0;
            Reset_Encoder_Distance();
            Car_Mode = 3;           // TASK_3_ACBD_DIAGONAL
            while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12)));
        }
    }

    // --- 第四题: PB2 ---
    if (!(gpiob_state & DL_GPIO_PIN_2)) {
        delay_ms(20);
        if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2))) {
            Control_Reset();
            mpu6050.Yaw = 0;
            Reset_Encoder_Distance();
            Car_Mode = 4;           // TASK_4_FOUR_LAPS
            while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)));
        }
    }
}

int main(void)
{
    // 1. 系统底层初始化
    SYSCFG_DL_init();

    // 2. 控制系统与传感器初始化
    Control_Init();
    mpu6050_init();

    // 3. 启动定时器中断 (10ms 闭环)
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) 
    {
        // --- 1. 更新陀螺仪姿态 ---
        AHRS_Geteuler();

        // --- 2. 扫描按键逻辑 ---
        Key_Scan_Proc();

        // --- 3. 循环延时 ---
        delay_ms(10);
    }
}

/**
 * @brief 定时器中断服务函数 (MSPM0G3507)
 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            Control_Loop(); // 调用 control.c 中的核心状态机
            break;
        default:
            break;
    }
}
