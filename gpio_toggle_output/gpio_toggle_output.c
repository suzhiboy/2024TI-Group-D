#include "ti_msp_dl_config.h"
#include "motor.h"

/**
 * @brief 2024电赛H题 - 硬件连线测试程序 (仅左电机)
 * 硬件分配：
 * PWMA (速度) -> PA12
 * AIN1 (方向1) -> PB13
 * AIN2 (方向2) -> PB14
 * STBY (使能) -> PB17
 */

int main(void)
{
    // 1. 硬件底层初始化 (由 SysConfig 生成，开启对应外设)
    SYSCFG_DL_init();

    // 2. 电机逻辑初始化 (启动 TIMG0, 设置 STBY 为高)
    Motor_Init();

    // 3. 点亮 LED1 表示程序已进入主逻辑
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    /* 
     * 4. 驱动左电机：
     * 参数 800 代表 50% 占空比 (1600 为满速)。
     * 如果转动方向反了，改为 -800。
     */
    Set_Motor_Speed_Left(800);

    // 5. 确保右电机停止
    Set_Motor_Speed_Right(0);

    while (1) {
        // 让 LED2 以 1Hz 频率闪烁，作为系统“心跳”指示
        delay_cycles(16000000); // 约 0.5 秒 (假设 32MHz)
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
    }
}
