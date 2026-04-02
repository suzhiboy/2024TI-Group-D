#include "control.h"
#include "pid.h"
#include "motor.h"
#include "sensor.h"

/* --- 1. 定义 PID 控制器实例 --- */
// 巡线 PID (位置式)：控制小车转向，使偏差趋于 0
PID_TypeDef pid_line_track;

/* --- 2. 控制参数配置 --- */
#define BASE_SPEED      150     // 基础直线行驶速度 (PWM)
#define TRACK_TARGET    0.0f    // 理想巡线目标值 (正中心偏差为 0)

/**
 * @brief 系统控制初始化
 * @note  在 main 函数初始化阶段调用一次
 */
void Control_Init(void)
{
    // 硬件底层初始化
    Motor_Init();
    
    /* 
     * 初始化巡线 PID 参数 (根据 8 路灰度传感器偏差值范围调整)
     * P: 比例项，决定转向灵敏度 (建议先从 2.0 开始调)
     * I: 积分项，消除静态误差 (循迹通常给很小或 0)
     * D: 微分项，抑制摆动，增加系统稳定性 (建议 P 的 1/10 ~ 1/5)
     * 
     * 参数顺序: &pid_instance, P, I, D, out_max, out_min, i_max
     */
    PID_Init(&pid_line_track, 4.5f, 0.01f, 1.2f, 150.0f, -150.0f, 20.0f);
    
    // 初始状态清零
    PID_Clear(&pid_line_track);
}

/**
 * @brief 循迹闭环控制逻辑
 * @note  建议在 10ms ~ 20ms 的定时器中断中循环调用，保证控制周期恒定
 */
void Control_Loop(void)
{
    float current_error = 0;
    float turn_output = 0;
    int16_t left_speed = 0;
    int16_t right_speed = 0;

    // 1. 获取当前传感器偏差 (范围约 -30 到 30)
    current_error = Sensor_Get_Error();
    
    // 2. 设置目标值 (目标是让偏差回到 0)
    pid_line_track.target = TRACK_TARGET;
    
    // 3. 计算 PID 输出 (转向差速)
    // 输出 turn_output 为正表示偏右需要左转，为负表示偏左需要右转 (取决于传感器正负定义)
    turn_output = PID_Calc_Positional(&pid_line_track, current_error);
    
    // 4. 动力分配 (左右电机差速控制)
    // 注意：这里的加减号取决于你的电机安装方向和转向逻辑
    left_speed  = BASE_SPEED + (int16_t)turn_output;
    right_speed = BASE_SPEED - (int16_t)turn_output;
    
    // 5. 将计算结果作用于执行机构 (设置电机 PWM)
    Set_Motor_Speed_Left(left_speed);
    Set_Motor_Speed_Right(right_speed);
}

/**
 * @brief 紧急停止并重置控制状态
 */
void Control_Reset(void)
{
    PID_Clear(&pid_line_track);
    Set_Motor_Speed_Left(0);
    Set_Motor_Speed_Right(0);
}
