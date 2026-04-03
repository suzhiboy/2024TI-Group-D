#include "ti_msp_dl_config.h"
#include "control.h"

int main(void)
{
    // 1. 系统底层初始化 (包含 SysConfig 生成的所有外设)
    SYSCFG_DL_init();

    // 2. 控制系统初始化 (PID、电机等)
    Control_Init();

    // 3. 启动定时器中断 (假设 SysConfig 中命名的定时器为 TIMER_0)
    // 注意：这里的函数名取决于你在 SysConfig 中给 Timer 命名的实例名
    DL_Timer_startCounter(TIMER_0_INST);

    // 4. 开启总中断 (必须调用，否则不会进中断函数)
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) 
    {
        // 主循环保持空闲，所有控制都在 10ms 中断中自动完成
        // 你也可以在这里写一些 OLED 显示逻辑，因为 OLED 刷新不需要太快
        __WFI(); // 进入低功耗睡眠等待中断，节省能耗
    }
}

/**
 * @brief 定时器中断服务函数 (MSPM0G3507)
 * @note  每 10ms 触发一次，用于执行 PID 闭环计算
 */
void TIMER_0_INST_IRQHandler(void)
{
    // 检查是否发生了 ZeroEvent 中断
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            // --- 核心控制逻辑 ---
            Control_Loop(); // 调用你 control.c 中的 10ms 闭环逻辑
            break;
        default:
            break;
    }
}

//主代码
