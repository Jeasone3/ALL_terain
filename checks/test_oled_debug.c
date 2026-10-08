#include "OLED_Debug.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t SystemCoreClock = 72000000u;
static ModeFSM_Diagnostics s_diag;
static uint16_t s_released = 0xffffu;
static uint32_t s_now;
static OLED_Device s_screen, s_expected;
static const uint8_t s_init[] = {0xaf};
static const OLED_Config s_config = {128, 64, 0, 0x3c, s_init, 1, 20, 500, 100};
static OLED_Result send(void *user, uint8_t address, uint8_t control,
                         const uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)user; (void)address; (void)control; (void)data; (void)len; (void)timeout;
    return OLED_OK;
}
void ModeFSM_GetDiagnostics(ModeFSM_Diagnostics *out) { *out = s_diag; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port;
    return (s_released & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
static void advance(unsigned ms)
{
    unsigned i;
    for (i = 0; i < ms / 5u; ++i) { s_now += 5u; OLED_Debug_Tick(s_now); }
}
static void tap(uint16_t pin)
{
    s_released &= (uint16_t)~pin; advance(100);
    s_released |= pin; advance(100);
}
static void expect(uint8_t row, const char *text)
{
    OLED_Clear(&s_expected, 0);
    OLED_DrawText(&s_expected, 0, (int16_t)(row * 8), text, OLED_FONT_6X8);
    assert(strlen(text) <= 21);
    assert(memcmp(&s_screen.canvas[row * 128], &s_expected.canvas[row * 128], strlen(text) * 6) == 0);
}
static void frame(uint32_t tick, uint8_t state, uint32_t count, uint8_t event)
{
    ModeFSM_DebugFrame f = {0};
    f.tick_ms = tick; f.state = state; f.event_count = count; f.event = event;
    OLED_Debug_OnFrame(&f);
}
int main(void)
{
    OLED_Port port = {send, NULL};
    unsigned i;
    OLED_Init(&s_screen, &s_config, &port, 0);
    OLED_Init(&s_expected, &s_config, &port, 0);
    OLED_Debug_Init(&s_screen, 0);
    s_diag.fsm.raw_mask = 0x0f;
    s_diag.control_max_cycles = 144000u;
    s_diag.control_last_cycles = 72000u;
    advance(400);
    expect(0, "RUN 1/5 LIVE");
    expect(2, "G:11110000 IN1->8");
    expect(4, "IMU:0 Y:INVALID");
    s_released &= (uint16_t)~RIGHT_Pin; advance(10);
    s_released |= RIGHT_Pin; advance(100);
    expect(0, "RUN 1/5 LIVE");
    puts("PASS live layout, sensor order and invalid angle");

    tap(RIGHT_Pin);
    expect(0, "JUNCTION 2/5 LIVE");
    /* 长按只产生一个事件；释放后再按才切下一页。 */
    s_released &= (uint16_t)~RIGHT_Pin; advance(500);
    expect(0, "HISTORY 3/5 LIVE");
    s_released |= RIGHT_Pin; advance(100);
    frame(10, STATE_JUNCTION_PENDING, 0, JUNCTION_NONE);
    frame(20, STATE_CROSS, 1, JUNCTION_CROSS);
    frame(30, STATE_FORWARD, 1, JUNCTION_CROSS);
    advance(400);
    expect(1, "      3 EXIT");
    expect(2, "      2 CROSS");
    tap(UP_Pin);
    expect(1, "      2 CROSS");
    tap(DOWN_Pin);
    expect(1, "      3 EXIT");
    puts("PASS key debounce, latched CROSS and history navigation");

    tap(OK_Pin);
    expect(0, "HISTORY 3/5 HOLD");
    frame(40, STATE_NORMAL_TRACK, 1, JUNCTION_CROSS);
    advance(400);
    expect(1, "      3 EXIT");
    tap(OK_Pin); advance(400);
    expect(1, "      4 TRACK");
    frame(100, STATE_NORMAL_TRACK, 1, JUNCTION_CROSS);
    advance(400);
    expect(1, "     10 LOG_GAP");
    for (i = 0; i < 40; ++i) frame(110 + i * 10, (uint8_t)(i % 2 ? STATE_NORMAL_TRACK : STATE_FORWARD), 1, JUNCTION_CROSS);
    advance(400);
    expect(1, "     50 TRACK");
    puts("PASS frozen history, missing frames and ring wrap");

    tap(OK_Pin);
    s_diag.fsm.state = STATE_FAULT_STOP;
    s_diag.fsm.fault = MODE_FAULT_HEADING;
    s_diag.fault_capture_valid = s_diag.fault_yaw_valid = 1u;
    s_diag.fault_state = STATE_JUNCTION_PENDING;
    s_diag.fault_frames = 4u;
    s_diag.fault_mask = 0x0fu;
    s_diag.fault_yaw_ddeg = -160;
    s_diag.fault_target_ddeg = 0;
    s_diag.fault_pwm_left = 500;
    s_diag.fault_pwm_right = 0;
    advance(400);
    expect(0, "FAULT 4/5 LIVE");
    expect(1, "F:2 HEADING_ERROR");
    expect(2, "AT:PENDING N:4");
    expect(3, "G:11110000");
    expect(5, "Y:-016.0 T:+000.0");
    expect(7, "PRE: +500/   +0");
    s_diag.fsm.raw_mask = 0xf0u;
    s_diag.yaw_ddeg = 300;
    advance(400);
    expect(3, "G:11110000");
    expect(5, "Y:-016.0 T:+000.0");
    s_diag.fault_yaw_valid = 0u;
    advance(400);
    expect(5, "Y:-- T:+000.0");
    tap(RIGHT_Pin);
    expect(0, "PERF 5/5 LIVE");
    expect(2, "CTL:1000us");
    expect(3, "MAX:2000us");
    puts("PASS fault preemption and performance units");
    puts("All OLED debug checks passed");
    return 0;
}
