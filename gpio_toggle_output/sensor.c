#include "sensor.h"

// 快速延时约 50us
#define DELAY_50US  (1600)

void Sensor_Read_All(uint8_t results[8])
{
    for (int i = 0; i < 8; i++) {
        // 分别控制 S0, S1, S2 的电平，使用各自的 PORT 宏
        if (i & 0x01) DL_GPIO_setPins(GPIO_SENSOR_S0_PORT, GPIO_SENSOR_S0_PIN);
        else DL_GPIO_clearPins(GPIO_SENSOR_S0_PORT, GPIO_SENSOR_S0_PIN);
        
        if (i & 0x02) DL_GPIO_setPins(GPIO_SENSOR_S1_PORT, GPIO_SENSOR_S1_PIN);
        else DL_GPIO_clearPins(GPIO_SENSOR_S1_PORT, GPIO_SENSOR_S1_PIN);
        
        if (i & 0x04) DL_GPIO_setPins(GPIO_SENSOR_S2_PORT, GPIO_SENSOR_S2_PIN);
        else DL_GPIO_clearPins(GPIO_SENSOR_S2_PORT, GPIO_SENSOR_S2_PIN);
        
        delay_cycles(DELAY_50US);
        
        // 读取 OUT 引脚，使用 OUT 专门的端口宏
        results[i] = DL_GPIO_readPins(GPIO_SENSOR_OUT_PORT, GPIO_SENSOR_OUT_PIN) ? 1 : 0;
    }
}

float Sensor_Get_Error(void)
{
    uint8_t data[8];
    Sensor_Read_All(data);
    
    int weights[8] = {-30, -20, -10, -5, 5, 10, 20, 30};
    int weighted_sum = 0;
    int count = 0;
    
    for (int i = 0; i < 8; i++) {
        if (data[i] == 1) { 
            weighted_sum += weights[i];
            count++;
        }
    }
    
    if (count == 0) return 0; 
    
    return (float)weighted_sum / count;
}
