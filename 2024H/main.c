#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"
#include "sensor.h"
#include "oled.h"
#include <stdio.h>

// 全局通信标志
volatile uint8_t g_vofa_send_flag = 0;
static uint8_t g_imu_id = 0; 
static uint8_t g_oled_refresh_div = 0;

/* --- 0. 调试开关 --- */
#define DEBUG_SENSORS_OLED   1   

void Debug_Sensors_Display(void);
extern float pitch2, roll2, Yaw;

int main(void)
{
    SYSCFG_DL_init();
    
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"System Ready!", 16, 1);
    OLED_Update();
    
    Control_Init();
    mpu6050_init();
    
    extern uint8_t g_imu_addr;
    extern uint8_t Single_ReadI2C(unsigned char SlaveAddress, unsigned char REG_Address);
    g_imu_id = Single_ReadI2C(g_imu_addr, 0x75);

    DL_UART_Main_enable(UART_BLUETOOTH_INST);

    __enable_irq();
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    
    // GPIOB 端口中断（包含编码器和按键）在 Encoder_Init 中已统一开启，此处可选保留
    NVIC_EnableIRQ(GPIOB_INT_IRQn); 

    while (1) 
    {
        // 核心逻辑已移至中断：
        // 1. 10ms 中断：姿态计算 + PID 控制
        // 2. GPIO 中断：按键触发 + 编码器解码 (详见 encoder.c)

        if (g_vofa_send_flag) {
            g_vofa_send_flag = 0;
            Vofa_Send_Debug(); 
        }

#if DEBUG_SENSORS_OLED
        if (++g_oled_refresh_div >= 20) { // 约 200ms 刷新一次显示
            g_oled_refresh_div = 0;
            Debug_Sensors_Display();
        }
#endif
    }
}

/**
 * @brief OLED 实时看板
 */
void Debug_Sensors_Display(void)
{
    char disp_buf[32]; 
    sprintf(disp_buf, "Y:%.1f ID:%02X   ", mpu6050.Yaw, g_imu_id);
    OLED_ShowString(0, 0, (uint8_t *)disp_buf, 16, 1);

    extern float Gyro_Z_Measeure;
    sprintf(disp_buf, "G:%.1f deg/s   ", Gyro_Z_Measeure);
    OLED_ShowString(0, 16, (uint8_t *)disp_buf, 16, 1);

    sprintf(disp_buf, "L:%d R:%d      ", (int)g_Encoder.speed_left, (int)g_Encoder.speed_right);
    OLED_ShowString(0, 32, (uint8_t *)disp_buf, 16, 1);

    sprintf(disp_buf, "D:%.1f M:%d    ", g_Encoder.distance_cm, (int)Car_Mode);
    OLED_ShowString(0, 48, (uint8_t *)disp_buf, 16, 1); 

    OLED_Update();
}

/**
 * @brief 10ms 控制中断
 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            AHRS_Geteuler_WithDt(0.01f);
            Control_Loop(); 
            g_vofa_send_flag = 1; 
            break;
        default:
            break;
    }
}
