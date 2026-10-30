#include "Int_SR04.h"
#include "Delay_us.h"

#define SR04_TRIGGER_PULSE_US     12u  /* Trig 高电平持续时间。 */
#define SR04_TRIGGER_INTERVAL_MS  60u  /* 两次被接受的请求之间的最小间隔。 */
#define SR04_ECHO_TIMEOUT_MS      30u  /* 从接受请求起，等待完整回波的最长时间。 */

/* 前台在短临界区内读写，中断只更新回波时间和测量状态。 */
static volatile SR04_Status s_status = SR04_IDLE;
static volatile uint32_t s_trigger_ms;    /* 最近一次接受请求时的 HAL 毫秒计数。 */
static volatile uint32_t s_rise_cycles;   /* 本次 Echo 上升沿对应的 DWT 周期数。 */
static volatile uint32_t s_pulse_cycles;  /* 本次完整 Echo 高电平占用的 CPU 周期数。 */
static volatile uint8_t s_waiting_fall;   /* 1 表示已收到上升沿，接下来只接受下降沿。 */
static uint8_t s_has_triggered;           /* 区分首次触发，避免启动后额外等待 60 ms。 */
static uint32_t s_cycles_per_us;          /* 初始化时按 SystemCoreClock 计算的每微秒周期数。 */

/**
 * @brief 将超过等待上限的请求转为超时，阻止后续迟到边沿完成旧请求。
 * @param now_ms 当前 HAL 毫秒计数；无符号差值可跨越毫秒计数器回绕。
 * @return 无。
 * @note 仅在前台临界区或 Echo 中断内调用，不依赖新增的定时任务。
 */
static void SR04_CheckTimeout(uint32_t now_ms)
{
    if (s_status == SR04_BUSY &&
        (uint32_t)(now_ms - s_trigger_ms) >= SR04_ECHO_TIMEOUT_MS)
    {
        s_status = SR04_TIMEOUT;
        s_waiting_fall = 0u;
    }
}

void Int_SR04_Init(void)
{
    uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    HAL_GPIO_WritePin(TR_GPIO_Port, TR_Pin, GPIO_PIN_RESET);
    s_status = SR04_IDLE;
    s_trigger_ms = 0u;
    s_rise_cycles = 0u;
    s_pulse_cycles = 0u;
    s_waiting_fall = 0u;
    s_has_triggered = 0u;
    s_cycles_per_us = SystemCoreClock / 1000000u;
    __set_PRIMASK(saved_primask);
}

uint8_t Int_SR04_Trigger(void)
{
    uint32_t saved_primask = __get_PRIMASK();
    uint32_t now_ms;

    __disable_irq();
    now_ms = HAL_GetTick();
    SR04_CheckTimeout(now_ms);

    if (s_status == SR04_BUSY ||
        (s_has_triggered != 0u &&
         (uint32_t)(now_ms - s_trigger_ms) < SR04_TRIGGER_INTERVAL_MS) ||
        HAL_GPIO_ReadPin(EC_GPIO_Port, EC_Pin) != GPIO_PIN_RESET)
    {
        __set_PRIMASK(saved_primask);
        return 0u;
    }

    /* 先建立本次请求的状态，再发送脉冲，保证恢复中断后能立即接收回波。 */
    s_trigger_ms = now_ms;
    s_has_triggered = 1u;
    s_waiting_fall = 0u;
    s_rise_cycles = 0u;
    s_pulse_cycles = 0u;
    s_status = SR04_BUSY;

    /* 仅在约 12 us 的脉冲期间保持临界区，避免控制中断将 Trig 高电平拉长。 */
    HAL_GPIO_WritePin(TR_GPIO_Port, TR_Pin, GPIO_PIN_SET);
    Delay_us(SR04_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(TR_GPIO_Port, TR_Pin, GPIO_PIN_RESET);
    __set_PRIMASK(saved_primask);
    return 1u;
}

void Int_SR04_GetResult(SR04_Result *result)
{
    uint32_t saved_primask;
    uint32_t pulse_cycles;
    SR04_Status status;

    if (result == 0)
    {
        return;
    }

    saved_primask = __get_PRIMASK();
    __disable_irq();
    SR04_CheckTimeout(HAL_GetTick());
    status = s_status;
    pulse_cycles = s_pulse_cycles;
    __set_PRIMASK(saved_primask);

    /* 中断恢复后再换算，状态与周期数均来自同一次快照。 */
    result->status = status;
    result->pulse_us = 0u;
    result->distance_cm = 0.0f;
    if (status == SR04_OK)
    {
        result->pulse_us = pulse_cycles / s_cycles_per_us;
        result->distance_cm = (float)result->pulse_us * 0.017f;
    }
}

/**
 * @brief 承接现有 EXTI4 -> HAL_GPIO_EXTI_IRQHandler() 回调链路。
 * @param GPIO_Pin 产生中断的引脚编号；只处理超声波 EC_Pin。
 * @return 无。
 * @note 上升沿记录起点，下降沿记录宽度；重复或顺序错误的边沿不改变测量阶段。
 *       使用无符号周期差处理 DWT 回绕，不在中断中换算距离或操作电机、定时器。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_cycles;

    if (GPIO_Pin != EC_Pin)
    {
        return;
    }

    now_cycles = DWT->CYCCNT;
    SR04_CheckTimeout(HAL_GetTick());
    if (s_status != SR04_BUSY)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(EC_GPIO_Port, EC_Pin) == GPIO_PIN_SET)
    {
        if (s_waiting_fall == 0u)
        {
            s_rise_cycles = now_cycles;
            s_waiting_fall = 1u;
        }
    }
    else if (s_waiting_fall != 0u)
    {
        s_pulse_cycles = (uint32_t)(now_cycles - s_rise_cycles);
        s_waiting_fall = 0u;
        s_status = SR04_OK;
    }
}
