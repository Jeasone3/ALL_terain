#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "Int_SR04.h"
#include "Delay_us.h"

GPIO_TypeDef g_test_gpioa;
DWT_Type g_test_dwt;
uint32_t SystemCoreClock;

static uint32_t s_tick_ms;          /* 模拟 HAL 毫秒计数，可单独设置回绕起点。 */
static uint32_t s_sub_ms_us;        /* 尚未累积满一毫秒的模拟时间。 */
static uint32_t s_primask;          /* 模拟调用者原有的中断屏蔽状态。 */
static GPIO_PinState s_echo_level;
static GPIO_PinState s_trig_level;
static uint32_t s_trigger_count;    /* 实际输出 Trig 上升沿的次数。 */
static uint32_t s_delay_calls;
static uint32_t s_last_delay_us;
static uint8_t s_echo_on_restore;   /* 恢复中断时交付已挂起的 Echo 边沿。 */

/** 推进墙钟与 DWT；周期数自然截断为 32 位，用于验证计数器回绕。 */
static void advance_us(uint32_t us)
{
    uint32_t total_us = s_sub_ms_us + us;

    DWT->CYCCNT += us * (SystemCoreClock / 1000000u);
    s_tick_ms += total_us / 1000u;
    s_sub_ms_us = total_us % 1000u;
}

uint32_t __get_PRIMASK(void)
{
    return s_primask;
}

void __disable_irq(void)
{
    s_primask = 1u;
}

void __set_PRIMASK(uint32_t mask)
{
    assert(mask <= 1u);
    s_primask = mask;
    if (mask == 0u && s_echo_on_restore != 0u)
    {
        s_echo_on_restore = 0u;
        HAL_GPIO_EXTI_Callback(EC_Pin);
    }
}

uint32_t HAL_GetTick(void)
{
    return s_tick_ms;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    assert(port == EC_GPIO_Port);
    assert(pin == EC_Pin);
    return s_echo_level;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == TR_GPIO_Port);
    assert(pin == TR_Pin);
    assert(s_primask == 1u);
    if (state == GPIO_PIN_SET)
    {
        assert(s_trig_level == GPIO_PIN_RESET);
        ++s_trigger_count;
    }
    s_trig_level = state;
}

/** 用虚拟时间替代忙等待；验证触发脉冲期间保持短临界区。 */
void Delay_us(uint32_t us)
{
    assert(s_trig_level == GPIO_PIN_SET);
    assert(s_primask == 1u);
    ++s_delay_calls;
    s_last_delay_us = us;
    advance_us(us);
}

/** 模拟一个实际 Echo 边沿；错误顺序与重复回调由各用例主动构造。 */
static void echo_edge(GPIO_PinState level)
{
    assert(s_primask == 0u);
    s_echo_level = level;
    HAL_GPIO_EXTI_Callback(EC_Pin);
}

/** 清空宿主外设并调用真实驱动初始化，不重置真实驱动之外的生产模块。 */
static void reset_fixture(void)
{
    SystemCoreClock = 72000000u;
    DWT->CYCCNT = 123456u;
    s_tick_ms = 0u;
    s_sub_ms_us = 0u;
    s_primask = 0u;
    s_echo_level = GPIO_PIN_RESET;
    s_trig_level = GPIO_PIN_SET;
    s_trigger_count = 0u;
    s_delay_calls = 0u;
    s_last_delay_us = 0u;
    s_echo_on_restore = 0u;
    Int_SR04_Init();
}

/** 每次先填入旧数值，确认非成功状态不会向调用者泄漏上一帧距离。 */
static void expect_result(SR04_Status status, uint32_t pulse_us, float distance_cm)
{
    SR04_Result result = {SR04_OK, 999u, 999.0f};

    Int_SR04_GetResult(&result);
    assert(result.status == status);
    assert(result.pulse_us == pulse_us);
    assert(fabsf(result.distance_cm - distance_cm) < 0.0001f);
}

/** 验证首次可立即触发、Trig 脉宽、正常回波和重复读取同一结果。 */
static void test_normal_echo(void)
{
    reset_fixture();
    assert(DWT->CYCCNT == 123456u);
    assert(s_trig_level == GPIO_PIN_RESET);
    expect_result(SR04_IDLE, 0u, 0.0f);
    assert(Int_SR04_Trigger() == 1u);
    assert(s_trigger_count == 1u && s_delay_calls == 1u);
    assert(s_last_delay_us == 12u);
    assert(DWT->CYCCNT == 124320u);
    assert(s_trig_level == GPIO_PIN_RESET && s_primask == 0u);
    expect_result(SR04_BUSY, 0u, 0.0f);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(1000u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_OK, 1000u, 17.0f);
    expect_result(SR04_OK, 1000u, 17.0f);
    advance_us(1000u);
    echo_edge(GPIO_PIN_SET);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_OK, 1000u, 17.0f);
}

/** 验证忙时拒绝重入、60 ms 边界，以及新请求接受后清除旧距离。 */
static void test_trigger_interval(void)
{
    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    assert(Int_SR04_Trigger() == 0u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    assert(Int_SR04_Trigger() == 0u);
    advance_us(1000u);
    echo_edge(GPIO_PIN_RESET);
    assert(Int_SR04_Trigger() == 0u);
    expect_result(SR04_OK, 1000u, 17.0f);
    advance_us(58887u); /* 从首次触发起总计 59999 us。 */
    assert(s_tick_ms == 59u);
    assert(Int_SR04_Trigger() == 0u);
    advance_us(1u);
    assert(Int_SR04_Trigger() == 1u);
    assert(s_trigger_count == 2u);
    expect_result(SR04_BUSY, 0u, 0.0f);
}

/** 无上升沿时在 30 ms 超时；拒绝孤立下降沿，超时后可重新测量。 */
static void test_missing_rise_and_retry(void)
{
    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    echo_edge(GPIO_PIN_RESET);
    advance_us(29987u);
    expect_result(SR04_BUSY, 0u, 0.0f);
    advance_us(1u);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
    echo_edge(GPIO_PIN_SET);
    advance_us(1000u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
    assert(Int_SR04_Trigger() == 0u);
    advance_us(29000u);
    assert(Int_SR04_Trigger() == 1u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(1500u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_OK, 1500u, 25.5f);
}

/** 已有上升沿但没有下降沿时超时；迟到的下降沿不得发布旧回波。 */
static void test_missing_fall(void)
{
    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(29888u);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
    advance_us(1000u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
}

/** 前台没有读取结果时，回调自身仍必须先检查请求是否已经超时。 */
static void test_late_edges_check_timeout(void)
{
    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    advance_us(29988u);
    echo_edge(GPIO_PIN_SET);
    advance_us(100u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);

    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(29888u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
}

/** Echo 卡高时不触发；超过 60 ms 后 Trigger 自身清理旧请求并允许重试。 */
static void test_echo_high_and_trigger_timeout(void)
{
    reset_fixture();
    s_echo_level = GPIO_PIN_SET;
    assert(Int_SR04_Trigger() == 0u);
    assert(s_trigger_count == 0u && s_delay_calls == 0u);
    expect_result(SR04_IDLE, 0u, 0.0f);
    s_echo_level = GPIO_PIN_RESET;
    assert(Int_SR04_Trigger() == 1u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(59888u);
    assert(Int_SR04_Trigger() == 0u);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
    echo_edge(GPIO_PIN_RESET);
    assert(Int_SR04_Trigger() == 1u);
    expect_result(SR04_BUSY, 0u, 0.0f);

    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    advance_us(59988u);
    assert(Int_SR04_Trigger() == 1u);
    assert(s_trigger_count == 2u);
}

/** 无请求的边沿、MPU 引脚和重复上升沿均不得改变有效回波的起点。 */
static void test_unrelated_and_duplicate_edges(void)
{
    reset_fixture();
    echo_edge(GPIO_PIN_SET);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_IDLE, 0u, 0.0f);
    assert(Int_SR04_Trigger() == 1u);
    HAL_GPIO_EXTI_Callback(MPU_INT_Pin);
    echo_edge(GPIO_PIN_RESET);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(500u);
    echo_edge(GPIO_PIN_SET);
    advance_us(500u);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_OK, 1000u, 17.0f);
}

/** 回波跨过 DWT 最大值仍能正确求差，不清零已有的自由运行计数器。 */
static void test_dwt_wrap(void)
{
    uint32_t rise_cycles;

    reset_fixture();
    DWT->CYCCNT = UINT32_MAX - 10000u;
    assert(Int_SR04_Trigger() == 1u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    rise_cycles = DWT->CYCCNT;
    advance_us(1000u);
    echo_edge(GPIO_PIN_RESET);
    assert(DWT->CYCCNT < rise_cycles);
    expect_result(SR04_OK, 1000u, 17.0f);
}

/** 毫秒计数器回绕不绕过触发间隔，长时间未轮询也不能让旧请求重新变为有效。 */
static void test_tick_wrap_and_long_gap(void)
{
    reset_fixture();
    s_tick_ms = UINT32_MAX - 20u;
    assert(Int_SR04_Trigger() == 1u);
    advance_us(29988u);
    assert(s_tick_ms == 9u);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
    assert(Int_SR04_Trigger() == 0u);
    advance_us(30000u);
    assert(Int_SR04_Trigger() == 1u);

    reset_fixture();
    assert(Int_SR04_Trigger() == 1u);
    advance_us(60000000u); /* 超过 72 MHz 下的一次 DWT 回绕周期。 */
    echo_edge(GPIO_PIN_SET);
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_TIMEOUT, 0u, 0.0f);
}

/** 换算使用初始化时的系统频率；不足 1 us 的周期余数按接口约定向下取整。 */
static void test_clock_and_fractional_pulse(void)
{
    reset_fixture();
    SystemCoreClock = 48000000u;
    Int_SR04_Init();
    assert(Int_SR04_Trigger() == 1u);
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(1000u);
    DWT->CYCCNT += 47u;
    echo_edge(GPIO_PIN_RESET);
    expect_result(SR04_OK, 1000u, 17.0f);
}

/** 所有前台接口保留原中断状态；恢复中断时到来的回波不会撕裂已复制的快照。 */
static void test_critical_section_and_snapshot(void)
{
    reset_fixture();
    s_primask = 1u;
    Int_SR04_Init();
    assert(s_primask == 1u);
    assert(Int_SR04_Trigger() == 1u);
    assert(s_primask == 1u);
    assert(Int_SR04_Trigger() == 0u);
    expect_result(SR04_BUSY, 0u, 0.0f);
    Int_SR04_GetResult(0);
    assert(s_primask == 1u);
    s_primask = 0u;
    advance_us(100u);
    echo_edge(GPIO_PIN_SET);
    advance_us(1000u);
    s_echo_level = GPIO_PIN_RESET;
    s_echo_on_restore = 1u;
    expect_result(SR04_BUSY, 0u, 0.0f);
    expect_result(SR04_OK, 1000u, 17.0f);
    s_primask = 1u;
    expect_result(SR04_OK, 1000u, 17.0f);
    assert(s_primask == 1u);
}

int main(void)
{
    test_normal_echo();
    test_trigger_interval();
    test_missing_rise_and_retry();
    test_missing_fall();
    test_late_edges_check_timeout();
    test_echo_high_and_trigger_timeout();
    test_unrelated_and_duplicate_edges();
    test_dwt_wrap();
    test_tick_wrap_and_long_gap();
    test_clock_and_fractional_pulse();
    test_critical_section_and_snapshot();
    puts("SR04: 11 test groups passed.");
    return 0;
}
