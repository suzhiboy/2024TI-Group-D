#include "control.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "main.h"

/* --- 1. 任务模式定义 --- */
#define TASK_IDLE               0
#define TASK_1_AB_STRAIGHT      1   // 任务1：A->B 直线 (200cm)
#define TASK_2_ABCD_CIRCLE      2   // 任务2：A->B->C->D->A 矩形环
#define TASK_3_ACBD_DIAGONAL    3   // 任务3：A->C->B->D->A 对角线环
#define TASK_4_FOUR_LAPS        4   // 任务4：跑4圈
#define TASK_FINISHED           100 

/* --- 2. 控制器与状态变量 --- */
PID_TypeDef pid_line;  
PID_TypeDef pid_yaw;   

uint8_t Car_Mode = TASK_IDLE;  
uint8_t Current_Step = 0;      
uint8_t Lap_Counter = 0;       
uint16_t Feedback_Timer = 0;   

/**
 * @brief 重置里程计
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
    
    // 初始化 PID
    PID_Init(&pid_line, 4.8f, 0.01f, 1.2f, 400.0f, -400.0f, 20.0f);
    PID_Init(&pid_yaw, 25.0f, 0.0f, 5.0f, 350.0f, -350.0f, 0.0f);
    
    Control_Reset();
}

/**
 * @brief 核心控制循环 (10ms)
 */
void Control_Loop(void)
{
    float turn_out = 0;
    int16_t L_speed = 0, R_speed = 0;

    // --- 0. 接入真实的编码器数据更新 ---
    Encoder_UpdateData_10ms(); 

    // --- 1. 声光反馈处理 ---
    if (Feedback_Timer > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7); Feedback_Timer--;
    } else if (Car_Mode != TASK_FINISHED) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_7);
    }

    // --- 2. 状态机逻辑 ---
    switch (Car_Mode) 
    {
        case TASK_IDLE: Control_Reset(); return;

        /* 任务 1: A -> B 直线 (100cm) */
        case TASK_1_AB_STRAIGHT:
            pid_yaw.target = 0;
            turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
            L_speed = 450 - (int16_t)turn_out; 
            R_speed = 450 + (int16_t)turn_out;
            
            // 判定 B 点：距离达到 100cm 或 抓到横线
            if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                Trigger_Feedback(); Car_Mode = TASK_FINISHED;
            }
            break;

        /* 任务 2: A->B(直) -> C(弧) -> D(直) -> A(弧) */
        case TASK_2_ABCD_CIRCLE:
            if (Current_Step == 0) { // A -> B (直线 100cm)
                pid_yaw.target = 0;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 500 - turn_out; R_speed = 500 + turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            } 
            else if (Current_Step == 1) { // B -> C (弧线循迹)
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                L_speed = 400 + turn_out; R_speed = 400 - turn_out;
                if (absFloat(mpu6050.Yaw) >= 178.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 2) { // C -> D (直线 100cm)
                pid_yaw.target = 180.0f;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 500 - turn_out; R_speed = 500 + turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 3) { // D -> A (弧线循迹)
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                L_speed = 400 + turn_out; R_speed = 400 - turn_out;
                if (absFloat(mpu6050.Yaw) <= 5.0f) { 
                    Trigger_Feedback(); Car_Mode = TASK_FINISHED;
                }
            }
            break;

        /* 任务 3: 对角线路径 (A-C-B-D-A) */
        case TASK_3_ACBD_DIAGONAL:
            if (Current_Step == 0) { // A -> C (直线 128.1cm, 角度 38.7°)
                pid_yaw.target = 38.7f; 
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 550 - turn_out; R_speed = 550 + turn_out;
                if (g_Encoder.distance_cm >= 128.1f || Is_On_CrossLine()) { 
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            // ... 任务3其他阶段逻辑同理 ...
            break;

        case TASK_4_FOUR_LAPS:
            // 循环执行任务3，计数 Lap_Counter
            break;

        case TASK_FINISHED:
            Control_Reset();
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7);
            return;
    }

    Set_Motor_Speed_Left(L_speed);
    Set_Motor_Speed_Right(R_speed);
}

void Control_Reset(void)
{
    PID_Clear(&pid_line);
    PID_Clear(&pid_yaw);
    Set_Motor_Speed_Left(0);
    Set_Motor_Speed_Right(0);
    Current_Step = 0;
    Reset_Encoder_Distance();
}
