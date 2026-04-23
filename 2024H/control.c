#include "control.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "main.h"
#include <stdio.h>

/* --- 1. 任务模式定义已移至 control.h，此处直接使用 --- */

/* --- 2. 控制器与状态变量 --- */
PID_TypeDef pid_line;  
PID_TypeDef pid_yaw;   
PID_TypeDef pid_speed_L;
PID_TypeDef pid_speed_R;

uint8_t Car_Mode = TASK_IDLE;  
uint8_t g_target_task = TASK_IDLE; // 目标任务暂存变量
uint8_t Current_Step = 0;      
uint8_t Lap_Counter = 0;       
uint16_t Feedback_Timer = 0;   

// 滤波后的平滑速度变量
static float filtered_L = 0;
static float filtered_R = 0;

/* --- 3. 辅助功能函数 --- */

/**
 * @brief 重置里程计接口
 */
void Reset_Encoder_Distance(void) {
    Encoder_Clear(); 
}

/**
 * @brief 8路传感器特征识别：识别横线 (用于 B, C, D 点校准)
 */
bool Is_On_CrossLine(void) {
    uint8_t s[8];
    Sensor_Read_All(s);
    int count = 0;
    for(int i=0; i<8; i++) if(s[i] == 1) count++;
    return (count >= 6); 
}

/**
 * @brief 触发声光反馈
 */
void Trigger_Feedback(void) {
    Feedback_Timer = 50; 
}

/**
 * @brief 系统控制初始化
 */
void Control_Init(void)
{
    Motor_Init();
    Encoder_Init(); 
    
    // 1. 循迹环 (输出为速度偏差量)
    PID_Init(&pid_line, 0.5f, 0.0f, 0.1f, 3.0f, -3.0f, 1.0f);
    
    // 2. 角度环 (输出为速度偏差量)
    PID_Init(&pid_yaw, 5.0f, 0.05f, 1.5f, 15.0f, -15.0f, 3.0f);
    // PID_Init(&pid_yaw, 3.0f, 0.02f, 1.0f, 10.0f, -10.0f, 2.0f);

    // 3. 速度内环 (输出为 PWM 值 0-2000)
    PID_Init(&pid_speed_L, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    PID_Init(&pid_speed_R, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    
    Control_Reset();
}

/**
 * @brief 核心控制循环 (10ms)
 */
void Control_Loop(void)
{
    float turn_out = 0;
    float base_speed = 0;
    int16_t final_L_pwm = 0, final_R_pwm = 0;

    // --- 0. 更新传感器数据 ---
    Encoder_UpdateData_10ms(); 

    // --- 1. 软件一阶低通滤波 ---
    filtered_L = filtered_L * 0.7f + (float)g_Encoder.speed_left * 0.3f;
    filtered_R = filtered_R * 0.7f + (float)g_Encoder.speed_right * 0.3f;

    // --- 2. 声光反馈处理 ---
    if (Feedback_Timer > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7); Feedback_Timer--;
    } else if (Car_Mode != TASK_FINISHED) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_7);
    }

    // --- 3. 状态机逻辑：计算目标速度 ---
    switch (Car_Mode) 
    {
        case TASK_IDLE: 
            pid_speed_L.target = 0;
            pid_speed_R.target = 0;
            break;

        case TASK_CALIBRATING:
            pid_speed_L.target = 0;
            pid_speed_R.target = 0;
            // 检测陀螺仪是否完成 100 次采样校准 (10ms * 100 = 1秒)
            if (MPU6050_Is_Calibrated()) {
                Reset_Encoder_Distance();
                Car_Mode = g_target_task; // 自动跳转到目标任务
            }
            break;

        case TASK_1_AB_STRAIGHT:
            base_speed = 8.0f; // 略微调低默认速度
            pid_yaw.target = 0.0f; 
            turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
            pid_speed_L.target = base_speed + turn_out;
            pid_speed_R.target = base_speed - turn_out;
            if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                Trigger_Feedback(); Car_Mode = TASK_FINISHED;
            }
            break;

        case TASK_4_FOUR_LAPS:
        case TASK_2_ABCD_CIRCLE:
            if (Current_Step == 0) { // A -> B (直线 100cm)
                base_speed = 8.0f; pid_yaw.target = 0.0f;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            } 
            else if (Current_Step == 1) { // B -> C (弧线循迹)
                base_speed = 7.0f;
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (absFloat(mpu6050.Yaw) >= 170.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 2) { // C -> D (直线 100cm)
                base_speed = 8.0f; 
                float err = 180.0f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 3) { // D -> A (弧线循迹)
                base_speed = 6.5f;
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (absFloat(mpu6050.Yaw) <= 10.0f) { 
                    Trigger_Feedback(); 
                    if (Car_Mode == TASK_4_FOUR_LAPS) {
                        Lap_Counter++;
                        if (Lap_Counter >= 4) { Car_Mode = TASK_FINISHED; Lap_Counter = 0; }
                        else { Current_Step = 0; Reset_Encoder_Distance(); }
                    } else { Car_Mode = TASK_FINISHED; }
                }
            }
            break;

        case TASK_3_ACBD_DIAGONAL:
            if (Current_Step == 0) { // A -> C (对角线 128.1cm, 38.7°)
                base_speed = 8.0f; 
                float err = 38.7f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 128.1f || Is_On_CrossLine()) { 
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback(); 
                }
            }
            else if (Current_Step == 1) { // C -> B (垂直边 80cm, -90.0°)
                base_speed = 8.0f; 
                float err = -90.0f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 80.0f || Is_On_CrossLine()) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback(); 
                }
            }
            else if (Current_Step == 2) { // B -> D (对角线 128.1cm, 141.3°)
                base_speed = 8.0f; 
                float err = 141.3f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 128.1f || Is_On_CrossLine()) { 
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback(); 
                }
            }
            else if (Current_Step == 3) { // D -> A (垂直边 80cm, -90.0°)
                base_speed = 8.0f; 
                float err = -90.0f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 80.0f || Is_On_CrossLine()) { 
                    Trigger_Feedback(); Car_Mode = TASK_FINISHED; 
                }
            }
            break;

        case TASK_FINISHED:
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7);
            break;
    }

    // --- 4. 内环计算并输出 PWM ---
    final_L_pwm = (int16_t)PID_Calc_Positional(&pid_speed_L, filtered_L);
    final_R_pwm = (int16_t)PID_Calc_Positional(&pid_speed_R, filtered_R);

    Set_Motor_Speed_Left(final_L_pwm);
    Set_Motor_Speed_Right(final_R_pwm);
}

/**
 * @brief VOFA+ 数据外送
 */
void Vofa_Send_Debug(void)
{
    float data[5];
    data[0] = pid_speed_L.target;
    data[1] = filtered_L; 
    data[2] = pid_speed_R.target;
    data[3] = filtered_R; 
    data[4] = mpu6050.Yaw; // 恢复发送角度值，目标值为 0.0

    for(int i=0; i<5; i++) {
        uint8_t *p = (uint8_t *)&data[i];
        for(int j=0; j<4; j++) {
            DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, p[j]);
        }
    }
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    for(int i=0; i<4; i++) DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, tail[i]);
}

void Control_Reset(void)
{
    PID_Clear(&pid_line);
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_speed_L);
    PID_Clear(&pid_speed_R);
    Set_Motor_Speed_Left(0);
    Set_Motor_Speed_Right(0);
    Current_Step = 0;
    Reset_Encoder_Distance();
}
