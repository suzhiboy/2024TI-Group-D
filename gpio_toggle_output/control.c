#include "control.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"

// 1. 定义全局的 PID 控制器实例
PID_TypeDef Line_Tracking_PID;

// 2. 系统上电时的初始化函数
void Control_System_Init(void) 
{
    // 初始化转向 PID
    PID_Init(&Line_Tracking_PID, 15.0f, 0.0f, 0.0f, 1000, -1000, 10000);
}

// 3. 核心控制逻辑
void System_Control_Loop(void) 
{
    // 1. 眼睛看
    float line_error = Sensor_Get_Error();

    // 2. 脑子想
    Line_Tracking_PID.target = 0.0f;
    int32_t turn_adjust = (int16_t)PID_Calc_Positional(&Line_Tracking_PID, line_error);
    
    // 3. 腿去走
    int16_t base_speed = 600; 
    
    Set_Motor_Speed_Left(base_speed + turn_adjust);
    Set_Motor_Speed_Right(base_speed - turn_adjust);
}
