#include "control.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "main.h"
#include <stdio.h>

// 辅助宏：绝对值计算
//#define absFloat(x) ((x) > 0 ? (x) : -(x))

/* --- 控制器与状态变量 --- */
PID_TypeDef pid_line;  
PID_TypeDef pid_yaw;   
PID_TypeDef pid_speed_L;
PID_TypeDef pid_speed_R;

uint8_t Car_Mode = TASK_IDLE;  
uint8_t g_target_task = TASK_IDLE; 
uint8_t Current_Step = 0;      
uint8_t Lap_Counter = 0;       
uint16_t Feedback_Timer = 0;   

float filtered_L = 0;
float filtered_R = 0;

/* --- 辅助功能函数 --- */

void Reset_Encoder_Distance(void) {
    Encoder_Clear(); 
}

bool Is_On_CrossLine(void) {
    uint8_t s[8];
    Sensor_Read_All(s);
    int count = 0;
    for(int i=0; i<8; i++) if(s[i] == 1) count++;
    return (count >= 1); 
}

void Trigger_Feedback(void) {
    Feedback_Timer = 50; 
}

void Control_Init(void)
{
    Motor_Init();
    Encoder_Init(); 
    // 循迹 PID 优化参数 (增加积分项 Ki 解决不居中问题)
    //PID_Init(&pid_line, 0.8f, 0.05f, 2.5f, 8.0f, -8.0f, 1.0f);
    PID_Init(&pid_line, 1.5f, 0.05f, 3.5f, 8.0f,-8.0f, 4.0f);
    PID_Init(&pid_yaw, 3.0f, 0.02f, 1.0f, 10.0f, -10.0f, 2.0f);
    PID_Init(&pid_speed_L, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    PID_Init(&pid_speed_R, 80.0f, 5.0f, 0.5f, 2000.0f, 0.0f, 1000.0f);
    Control_Reset();
}

void Control_Loop(void)
{
    float turn_out = 0;
    float base_speed = 0;
    int16_t final_L_pwm = 0, final_R_pwm = 0;

    Encoder_UpdateData_10ms(); 

    filtered_L = filtered_L * 0.7f + (float)g_Encoder.speed_left * 0.3f;
    filtered_R = filtered_R * 0.7f + (float)g_Encoder.speed_right * 0.3f;

    if (Feedback_Timer > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7); Feedback_Timer--;
    } else if (Car_Mode != TASK_FINISHED) {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_7);
    }

    switch (Car_Mode) 
    {
        case TASK_IDLE: 
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            // 静态调参：在静止时也计算循迹 PID，使 VOFA data[2] 产生波形
            //PID_Calc_Positional(&pid_line, Sensor_Get_Error());
            break;

        case TASK_CALIBRATING:
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            if (MPU6050_Is_Calibrated()) {
                Reset_Encoder_Distance();
                Car_Mode = g_target_task; 
            }
            break;

        case TASK_1_AB_STRAIGHT:
            base_speed = 12.0f; // 恢复原始速度
            pid_yaw.target = 0.0f; 
            turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
            pid_speed_L.target = base_speed + turn_out;
            pid_speed_R.target = base_speed - turn_out;
            if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                Trigger_Feedback(); Car_Mode = TASK_FINISHED;
                PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
            }
            break;

        case TASK_2_ABCD_CIRCLE:
        case TASK_4_FOUR_LAPS:
            if (Current_Step == 0) { 
                base_speed = 15.0f; pid_yaw.target = 0.0f;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 100.0f || Is_On_CrossLine()) {
                    Current_Step = 1; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            } 
            else if (Current_Step == 1) { 
                base_speed = 12.0f;
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (absFloat(mpu6050.Yaw) >= 170.0f) { 
                    Current_Step = 2; Reset_Encoder_Distance(); Trigger_Feedback();
                }
            }
            else if (Current_Step == 2) { 
                base_speed = 15.0f; 
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
            else if (Current_Step == 3) { 
                base_speed = 12.0f;
                turn_out = PID_Calc_Positional(&pid_line, Sensor_Get_Error());
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (absFloat(mpu6050.Yaw) <= 10.0f) { 
                    Trigger_Feedback(); 
                    if (Car_Mode == TASK_4_FOUR_LAPS) {
                        Lap_Counter++;
                        if (Lap_Counter >= 4) { 
                            Car_Mode = TASK_FINISHED; Lap_Counter = 0; 
                            PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                        }
                        else { Current_Step = 0; Reset_Encoder_Distance(); }
                    } else { 
                        Car_Mode = TASK_FINISHED; 
                        PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                    }
                }
            }
            break;

        case TASK_3_ACBD_DIAGONAL:
            if (Current_Step == 0) { 
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
            else if (Current_Step == 1) { 
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
            else if (Current_Step == 2) { 
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
            else if (Current_Step == 3) { 
                base_speed = 8.0f; 
                float err = -90.0f - mpu6050.Yaw;
                if (err > 180.0f) err -= 360.0f; else if (err < -180.0f) err += 360.0f;
                pid_yaw.target = mpu6050.Yaw + err;
                turn_out = PID_Calc_Positional(&pid_yaw, mpu6050.Yaw);
                pid_speed_L.target = base_speed + turn_out;
                pid_speed_R.target = base_speed - turn_out;
                if (g_Encoder.distance_cm >= 80.0f || Is_On_CrossLine()) { 
                    Trigger_Feedback(); Car_Mode = TASK_FINISHED; 
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                }
            }
            break;

        case TASK_FINISHED:
            pid_speed_L.target = 0; pid_speed_R.target = 0;
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_7);
            break;
    }

    final_L_pwm = (int16_t)PID_Calc_Positional(&pid_speed_L, filtered_L);
    final_R_pwm = (int16_t)PID_Calc_Positional(&pid_speed_R, filtered_R);

    Set_Motor_Speed_Left(final_L_pwm);
    Set_Motor_Speed_Right(final_R_pwm);
}

void Vofa_Send_Debug(void)
{
    float data[6];
    data[0] = 0.0F; 
    data[1] = Sensor_Get_Error();        
    data[2] = pid_line.output; // 转向输出力度  
    data[3] = filtered_L;          
    data[4] = filtered_R;         
    data[5] = mpu6050.Yaw;    

    for(int i=0; i<6; i++) {
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
