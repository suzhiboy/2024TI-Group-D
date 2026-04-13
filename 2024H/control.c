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
            // Yaw 角度归一化处理 (最短路径转向)
            {
                float target_yaw = 0.0f;
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;
            }
            
            turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
            L_speed = 450 - (int16_t)turn_out; 
            R_speed = 450 + (int16_t)turn_out;
            
            // 判定 B 点：距离达到 100cm 或 抓到横线
            if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                Trigger_Feedback(); Car_Mode = TASK_FINISHED;
            }
            break;

        /* 任务 4: 跑 4 圈 (复用任务 2 逻辑) */
        case TASK_4_FOUR_LAPS:
        /* 任务 2: A->B(直) -> C(弧) -> D(直) -> A(弧) */
        case TASK_2_ABCD_CIRCLE:
            if (Current_Step == 0) { // A -> B (直线 100cm)
                float target_yaw = 0.0f;
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;

                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 500 - turn_out; R_speed = 500 + turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            } 
            else if (Current_Step == 1) { // B -> C (弧线循迹)
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                L_speed = 400 + turn_out; R_speed = 400 - turn_out;
                if (absFloat(mpu6050.Yaw) >= 170.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 2) { // C -> D (直线 100cm)
                float target_yaw = 180.0f;
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;

                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 500 - turn_out; R_speed = 500 + turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 3) { // D -> A (弧线循迹)
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                L_speed = 400 + turn_out; R_speed = 400 - turn_out;
                if (absFloat(mpu6050.Yaw) <= 10.0f) { 
                    Trigger_Feedback(); 
                    if (Car_Mode == TASK_4_FOUR_LAPS) {
                        Lap_Counter++;
                        if (Lap_Counter >= 4) { Car_Mode = TASK_FINISHED; Lap_Counter = 0; }
                        else { Current_Step = 0; Reset_Encoder_Distance(); }
                    } else {
                        Car_Mode = TASK_FINISHED;
                    }
                }
            }
            break;

        /* 任务 3: 对角线路径 (A-C-B-D-A) */
        case TASK_3_ACBD_DIAGONAL:
            if (Current_Step == 0) { // A -> C (对角线 128.1cm, 角度 38.7°)
                float target_yaw = 38.7f; 
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;

                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 550 - turn_out; R_speed = 550 + turn_out;
                if (g_Encoder.distance_cm >= 128.1f || Is_On_CrossLine()) { 
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 1) { // C -> B (垂直边 80cm, 角度 -90.0°)
                float target_yaw = -90.0f; 
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;

                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 500 - turn_out; R_speed = 500 + turn_out;
                if (g_Encoder.distance_cm >= 80.0f || Is_On_CrossLine()) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 2) { // B -> D (对角线 128.1cm, 角度 141.3°)
                float target_yaw = 141.3f; 
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;

                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 550 - turn_out; R_speed = 550 + turn_out;
                if (g_Encoder.distance_cm >= 128.1f || Is_On_CrossLine()) { 
                    Current_Step = 3; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 3) { // D -> A (垂直边 80cm, 角度 -90.0°)
                float target_yaw = -90.0f; 
                float error_yaw = target_yaw - mpu6050.Yaw;
                if (error_yaw > 180.0f) error_yaw -= 360.0f;
                else if (error_yaw < -180.0f) error_yaw += 360.0f;
                pid_yaw.target = mpu6050.Yaw + error_yaw;

                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                L_speed = 500 - turn_out; R_speed = 500 + turn_out;
                if (g_Encoder.distance_cm >= 80.0f || Is_On_CrossLine()) { 
                    Trigger_Feedback(); Car_Mode = TASK_FINISHED;
                }
            }
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
