#include "ti_msp_dl_config.h"
#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "delay.h"
#include "main.h"
#include "sensor.h"
#include "oled.h"
#include <stdio.h>

// 1. 定义一个全局标志位 (在头部)
volatile uint8_t g_vofa_send_flag = 0;

/* --- 0. 调试开关 --- */
#define DEBUG_SENSORS_OLED   1   // 设置为 1 开启 OLED 传感器实时调试

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
                Control_Reset(); Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_1_AB_STRAIGHT;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_3))); 
            }
        }
        // 第二题: PB13
        else if (!(gpiob_state & DL_GPIO_PIN_13)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13))) {
                Control_Reset(); Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_2_ABCD_CIRCLE;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_13)));
            }
        }
        // 第三题: PB12
        else if (!(gpiob_state & DL_GPIO_PIN_12)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12))) {
                Control_Reset(); Yaw_Reset(); Reset_Encoder_Distance();
                Car_Mode = TASK_3_ACBD_DIAGONAL;
                while(!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_12)));
            }
        }
        // 第四题: PB2
        else if (!(gpiob_state & DL_GPIO_PIN_2)) {
            delay_ms(20);
            if (!(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_2))) {
                Control_Reset(); Yaw_Reset(); Reset_Encoder_Distance();
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
    
    Control_Init();
    
    // 显式使能 I2C 控制器，否则 MPU6050 无法通信
    DL_I2C_enableController(I2C_0_INST);
    mpu6050_init();

    // 显式开启串口发送功能，确保 VOFA+ 能收到数据
    DL_UART_Main_enable(UART_BLUETOOTH_INST);

    DL_Timer_startCounter(TIMER_0_INST);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) 
    {
        if (g_ahrs_update_flag) {
            g_ahrs_update_flag = 0;
            AHRS_Geteuler();
        }
        Key_Scan_Proc();

        // 【新增】：在主循环中发送串口数据，随时可以被编码器中断打断，不漏脉冲！
        if (g_vofa_send_flag) {
            g_vofa_send_flag = 0;
            Vofa_Send_Debug(); 
        }

        
#if DEBUG_SENSORS_OLED
        if (Car_Mode == TASK_IDLE) { // 仅在待机模式下刷新调试信息，以免干扰运行时的 OLED 显示
            Debug_Sensors_Display();
        }
#endif

        delay_ms(10);
    }
}

/**
 * @brief 灰度传感器 8 路调试可视化输出 (OLED 方案)
 */
void Debug_Sensors_Display(void)
{
    uint8_t sensor_data[8];
    char disp_buf[16];
    float current_err = 0;

    // 1. 读取传感器原始数据
    Sensor_Read_All(sensor_data);
    // 2. 获取计算出的 PID 误差量
    current_err = Sensor_Get_Error();

    // 3. 构建 8 路状态字符串 (例如 "11000000")
    for (int i = 0; i < 8; i++) {
        disp_buf[i] = (sensor_data[i] == 1) ? '1' : '0';
    }
    disp_buf[8] = '\0';

    // 4. OLED 刷新显示
    OLED_ShowString(0, 0, (uint8_t *)"Sensors:", 16, 1);
    OLED_ShowString(64, 0, (uint8_t *)disp_buf, 16, 1);  // 第一行右侧显示 0101

    sprintf(disp_buf, "Err: %+.2f  ", current_err);
    OLED_ShowString(0, 16, (uint8_t *)disp_buf, 16, 1);   // 第二行显示偏差量

    // 可以在第三行增加任务状态辅助监控
    sprintf(disp_buf, "Dist:%.1f cm  ", g_Encoder.distance_cm);
    OLED_ShowString(0, 32, (uint8_t *)disp_buf, 16, 1);   // 第三行显示当前里程

    OLED_Update(); // 必须调用更新函数，数据才会刷到屏幕上
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            g_ahrs_update_flag = 1;
            Control_Loop(); 
            g_vofa_send_flag = 1; // 触发串口发送标志
            break;
        default:
            break;
    }
}
