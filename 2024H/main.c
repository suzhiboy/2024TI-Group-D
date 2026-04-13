#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"

/**
 * @brief 按键扫描逻辑
 * @note  启动任务：PB3, PB13, PB12, PB2
 * @note  急停任务：运行中按上述任意键
 */
void Key_Scan_Proc(void)
{
    // 定义使用的所有功能键掩码
    uint32_t button_pins = DL_GPIO_PIN_3 | DL_GPIO_PIN_13 | DL_GPIO_PIN_12 | DL_GPIO_PIN_2;
    // 读取 GPIOB 状态
    uint32_t gpiob_state = DL_GPIO_readPins(GPIOB, button_pins);

    /* --- 1. 运行中按任意键急停 --- */
    if (Car_Mode >= TASK_1_AB_STRAIGHT && Car_Mode <= TASK_4_FOUR_LAPS) 
    {
        if ((gpiob_state & button_pins) != button_pins) 
        {
            delay_ms(20); // 硬件消抖
            if ((DL_GPIO_readPins(GPIOB, button_pins) & button_pins) != button_pins) 
            {
                Car_Mode = TASK_IDLE; // 切换到待机
                Control_Reset();      // 立即停止
                while((DL_GPIO_readPins(GPIOB, button_pins) & button_pins) != button_pins); 
                return; // 退出扫描逻辑
            }
        }
    }

    /* --- 2. 待机时启动任务 --- */
    if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) 
    {
        // 第一题: PB3
        if (!(gpiob_state & DL_GPIO_PIN_3)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))) {
                Control_Reset(); mpu6050.Yaw = 0; Reset_Encoder_Distance();
                Car_Mode = TASK_1_AB_STRAIGHT;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))); 
            }
        }
        // 第二题: PB13
        else if (!(gpiob_state & DL_GPIO_PIN_13)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13))) {
                Control_Reset(); mpu6050.Yaw = 0; Reset_Encoder_Distance();
                Car_Mode = TASK_2_ABCD_CIRCLE;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13)));
            }
        }
        // 第三题: PB12
        else if (!(gpiob_state & DL_GPIO_PIN_12)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12))) {
                Control_Reset(); mpu6050.Yaw = 0; Reset_Encoder_Distance();
                Car_Mode = TASK_3_ACBD_DIAGONAL;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12)));
            }
        }
        // 第四题: PB2
        else if (!(gpiob_state & DL_GPIO_PIN_2)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2))) {
                Control_Reset(); mpu6050.Yaw = 0; Reset_Encoder_Distance();
                Car_Mode = TASK_4_FOUR_LAPS;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)));
            }
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    Control_Init();
    mpu6050_init();

    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) 
    {
        AHRS_Geteuler();
        Key_Scan_Proc();
        delay_ms(10);
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            Control_Loop(); 
            break;
        default:
            break;
    }
}
