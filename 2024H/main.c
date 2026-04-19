#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"
#include "sensor.h"
#include "oled.h"
#include <stdio.h>

// 1. 定义一个全局标志位
volatile uint8_t g_vofa_send_flag = 0;
static uint8_t g_imu_id = 0; // 用于存储 MPU6050 的 ID，避免重复读取

/* --- 0. 调试开关 --- */
#define DEBUG_SENSORS_OLED   1   

void Debug_Sensors_Display(void);
extern float pitch2, roll2, Yaw;
static volatile uint8_t g_ahrs_update_flag = 0;

void Yaw_Reset(void)
{
    Yaw = 0.0f;
    pitch2 = 0.0f;
    roll2 = 0.0f;
    mpu6050.Yaw = 0.0f;
}

/**
 * @brief 按键扫描逻辑
 */
void Key_Scan_Proc(void)
{
    uint32_t button_pins = DL_GPIO_PIN_3 | DL_GPIO_PIN_13 | DL_GPIO_PIN_12 | DL_GPIO_PIN_2;
    uint32_t gpiob_state = DL_GPIO_readPins(GPIOB, button_pins);

    if (Car_Mode >= TASK_1_AB_STRAIGHT && Car_Mode <= TASK_4_FOUR_LAPS) 
    {
        if ((gpiob_state & button_pins) != button_pins) 
        {
            delay_ms(20); 
            if ((DL_GPIO_readPins(GPIOB, button_pins) & button_pins) != button_pins) 
            {
                Car_Mode = TASK_IDLE; 
                Control_Reset();      
                while((DL_GPIO_readPins(GPIOB, button_pins) & button_pins) != button_pins); 
                return; 
            }
        }
    }

    if (Car_Mode == TASK_IDLE || Car_Mode == TASK_FINISHED) 
    {
        if (!(gpiob_state & DL_GPIO_PIN_3)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))) {
                Control_Reset(); delay_ms(500); Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_1_AB_STRAIGHT;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))); 
            }
        }
        else if (!(gpiob_state & DL_GPIO_PIN_13)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13))) {
                Control_Reset(); delay_ms(500); Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_2_ABCD_CIRCLE;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13)));
            }
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"System Init...", 16, 1);
    OLED_Update();
    
    Control_Init();
    
    // 显式使能 I2C 并初始化
    DL_I2C_enableController(I2C_0_INST);
    mpu6050_init();
    
    // 获取一次 ID 即可
    extern uint8_t g_imu_addr;
    extern uint8_t Single_ReadI2C(unsigned char SlaveAddress, unsigned char REG_Address);
    g_imu_id = Single_ReadI2C(g_imu_addr, 0x75);

    // 显式开启串口发送功能 (仅一次)
    DL_UART_Main_enable(UART_BLUETOOTH_INST);

    __enable_irq();

    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIOB_INT_IRQn); 

    while (1) 
    {
        if (g_ahrs_update_flag) {
            g_ahrs_update_flag = 0;
            AHRS_Geteuler();
        }
        Key_Scan_Proc();

        if (g_vofa_send_flag) {
            g_vofa_send_flag = 0;
            Vofa_Send_Debug(); 
        }

#if DEBUG_SENSORS_OLED
        Debug_Sensors_Display();
#endif
        delay_ms(10);
    }
}

void Debug_Sensors_Display(void)
{
    char disp_buf[32]; 

    // 第一行：使用缓存的 ID，不再去敲 I2C 总线
    sprintf(disp_buf, "Y:%.1f ID:%02X   ", mpu6050.Yaw, g_imu_id);
    OLED_ShowString(0, 0, (uint8_t *)disp_buf, 16, 1);

    // 第二行：显示原始角速度
    extern float Gyro_Z_Measeure;
    sprintf(disp_buf, "G:%.1f deg/s   ", Gyro_Z_Measeure);
    OLED_ShowString(0, 16, (uint8_t *)disp_buf, 16, 1);

    // 第三行：显示编码器实时脉冲
    sprintf(disp_buf, "L:%d R:%d      ", (int)g_Encoder.speed_left, (int)g_Encoder.speed_right);
    OLED_ShowString(0, 32, (uint8_t *)disp_buf, 16, 1);

    // 第四行：显示累计里程 和 模式
    sprintf(disp_buf, "D:%.1f M:%d    ", g_Encoder.distance_cm, (int)Car_Mode);
    OLED_ShowString(0, 48, (uint8_t *)disp_buf, 16, 1); 

    OLED_Update();
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            g_ahrs_update_flag = 1;
            Control_Loop(); 
            g_vofa_send_flag = 1; 
            break;
        default:
            break;
    }
}
