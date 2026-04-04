#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

// 编码器数据结构体（作为全局数据中心的一部分）
typedef struct {
    int16_t  speed_left;       // 左轮当前速度 (单位: 脉冲数/10ms)
    int16_t  speed_right;      // 右轮当前速度 (单位: 脉冲数/10ms)
    
    int32_t  distance_left;    // 左轮累计行驶距离 (总脉冲数)
    int32_t  distance_right;   // 右轮累计行驶距离 (总脉冲数)
} Encoder_Data_t;

extern Encoder_Data_t g_Encoder; // 供成员 B (逻辑层) 读取

void Encoder_Init(void);
void Encoder_UpdateData_10ms(void);
void Encoder_Clear_Distance(void);

#endif