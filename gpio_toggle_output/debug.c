#include <stdio.h>
#include "pid.h"

float virtual_motor_speed = 0.0f;

void Set_Left_Motor_PWM(float pwm)
{
    // 假设电机的响应不是瞬间的：当前速度 = 惯性保持 + PWM驱动力
    // 0.95 是惯性系数(摩擦衰减)，0.02 是动力转换系数
    virtual_motor_speed = virtual_motor_speed * 0.95f + pwm * 0.02f;
}


// 模拟读取编码器 (将虚拟速度转化为脉冲数)
float Get_Left_Encoder(void) {
    // 假设速度1对应的脉冲是1 (这里简化处理，直接返回虚拟速度)
    return virtual_motor_speed; 
}

PID_TypeDef Motor_PID;

void Timer_10ms_Interrupt()
{
    float current_speed = Get_Left_Encoder();

    float new_pwm = PID_Calc_Incremental(&Motor_PID, current_speed);

    Set_Left_Motor_PWM(new_pwm);

}

int debug_test_main(){
    PID_Init(&Motor_PID, 10.0f, 1.0f, 0.0f, 1000.0f, 500.0f, 1000.0f);

    Motor_PID.target = 100.0f;

    printf("仿真开始！目标速度: 100\n");


    for(int i=0;i<=50;i++)
    {
        Timer_10ms_Interrupt();
        printf("时间: %3d ms | 实际速度: %6.2f | 输出PWM: %6.2f | 误差: %6.2f\n", 
               i * 10, Get_Left_Encoder(), Motor_PID.output, Motor_PID.err);
    }

    return 0;
}