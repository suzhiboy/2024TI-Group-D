#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"
#include "sensor.h"
#include "oled.h"
#include <stdio.h>

/* --- 0. 调试开关 --- */
#define DEBUG_SENSORS_OLED   1   // 设置为 1 开启 OLED 传感器实时调试

void Debug_Sensors_Display(void);
extern float pitch2, roll2, Yaw;
static volatile uint8_t g_ahrs_update_flag = 0;
static uint32_t g_heartbeat = 0;      // 心跳计数器
static uint32_t g_interrupt_cnt = 0;  // 中断计数器

void Yaw_Reset(void)
{
    Yaw = 0.0f;
    pitch2 = 0.0f;
    roll2 = 0.0f;
    mpu6050.Yaw = 0.0f;
}

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
                Control_Reset(); 
                delay_ms(500); // 关键修改：等待陀螺仪读数稳定
                Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_1_AB_STRAIGHT;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))); 
            }
        }
        // 第二题: PB13
        else if (!(gpiob_state & DL_GPIO_PIN_13)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13))) {
                Control_Reset();
                delay_ms(500); // 关键修改
                Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_2_ABCD_CIRCLE;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13)));
            }
        }
        // 第三题: PB12
        else if (!(gpiob_state & DL_GPIO_PIN_12)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12))) {
                Control_Reset();
                delay_ms(500); // 关键修改
                Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_3_ACBD_DIAGONAL;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12)));
            }
        }
        // 第四题: PB2
        else if (!(gpiob_state & DL_GPIO_PIN_2)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2))) {
                Control_Reset();
                delay_ms(500); // 关键修改
                Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_4_FOUR_LAPS;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2)));
            }
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    
    // 硬件基础初始化
    OLED_Init();
    OLED_Clear();
    
    // OLED 初始化测试：开机立刻显示信息并等待 1 秒
    OLED_ShowString(0, 0, (uint8_t *)"OLED OK!", 16, 1);
    OLED_Update();
    delay_ms(1000);
    
    Control_Init();
    mpu6050_init();

    // 显式使能全局中断
    __enable_irq();

    // 开启中断
    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIOB_INT_IRQn); // 开启编码器所在的端口中断
    NVIC_EnableIRQ(GPIOA_INT_IRQn); // 开启 MPU6050 所在的端口中断

    while (1) 
    {
        if (g_ahrs_update_flag) {
            g_ahrs_update_flag = 0;
            AHRS_Geteuler();
        }
        Key_Scan_Proc();

#if DEBUG_SENSORS_OLED
        // 解除限制：任务运行时也刷新显示，方便实时观察里程和 Yaw
        Debug_Sensors_Display();
#endif

        delay_ms(10);
    }
}

void Debug_Sensors_Display(void)
{
    char disp_buf[32]; 
    g_heartbeat++; 

    // 第一行：显示 Yaw 和 MPU6050 的身份 ID
    extern uint8_t g_imu_addr;
    uint8_t who_am_i = 0;
    extern void I2C_ReadReg(uint8_t DevAddr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
    I2C_ReadReg(g_imu_addr, 0x75, &who_am_i, 1); // 0x75 是 WHO_AM_I 寄存器

    sprintf(disp_buf, "Y:%.1f ID:%d    ", mpu6050.Yaw, (int)who_am_i);
    OLED_ShowString(0, 0, (uint8_t *)disp_buf, 16, 1);

    // 第二行：显示原始角速度 G 和 中断计数 iC
    extern float Gyro_Z_Measeure;
    sprintf(disp_buf, "G:%.1f iC:%d    ", Gyro_Z_Measeure, (int)(g_interrupt_cnt % 100));
    OLED_ShowString(0, 16, (uint8_t *)disp_buf, 16, 1);

    // 第三行：显示编码器实时脉冲 L / R
    sprintf(disp_buf, "L:%d R:%d      ", (int)g_Encoder.speed_left, (int)g_Encoder.speed_right);
    OLED_ShowString(0, 32, (uint8_t *)disp_buf, 16, 1);

    // 第四行：显示累计里程 和 模式
    sprintf(disp_buf, "D:%.1f M:%d    ", g_Encoder.distance_cm, (int)Car_Mode);
    OLED_ShowString(0, 48, (uint8_t *)disp_buf, 16, 1); 

    OLED_Update();
}

// 恢复为原始的中断函数名，确保中断能够进入
void TIMER_0_INST_IRQHandler(void)
{
    uint32_t pending = DL_Timer_getPendingInterrupt(TIMER_0_INST);
    if (pending & DL_TIMER_IIDX_ZERO) {
        g_interrupt_cnt++;
        g_ahrs_update_flag = 1;
        Control_Loop(); 
    }
}
