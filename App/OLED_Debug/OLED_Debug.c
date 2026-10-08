#include "OLED_Debug.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define HISTORY_SIZE 32u
#define PAGE_COUNT 5u
typedef struct {
    uint32_t tick_ms;
    uint8_t kind, state, event, fault, mask;
} History;
/* kind：0 状态变化，1 确认，2 取消，3 故障，4 日志缺口。 */
static History s_history[HISTORY_SIZE], s_frozen_history[HISTORY_SIZE];
static uint8_t s_head, s_count, s_frozen_head, s_frozen_count, s_scroll;
static uint8_t s_page, s_hold, s_redraw, s_have_frame, s_last_fault;
static uint32_t s_last_render;
static ModeFSM_DebugFrame s_previous;
static ModeFSM_Diagnostics s_view;
static OLED_Device *s_device;
typedef struct { uint8_t raw, stable; uint32_t changed_ms; } Key;
static Key s_keys[5];

static const char *state_name(uint8_t state)
{
    static const char *const names[] = {
        "TRACK", "CROSS", "LEFT", "RIGHT", "EXIT", "PENDING", "APPROACH", "STOP"
    };
    return state < 8u ? names[state] : "UNKNOWN";
}
static const char *event_name(uint8_t event)
{
    static const char *const names[] = {"NONE", "CROSS", "LEFT", "RIGHT"};
    return event < 4u ? names[event] : "UNKNOWN";
}
static const char *fault_name(uint8_t fault)
{
    static const char *const names[] = {
        "NONE", "IMU_INVALID", "HEADING_ERROR", "CENTER_LOST", "AMBIGUOUS",
        "TURN_TIMEOUT", "EXIT_TIMEOUT", "INVALID_STATE"
    };
    return fault < 8u ? names[fault] : "UNKNOWN";
}
/* 每行限制到 21 字符；先在画布清空，再写文字，短字符串不会留残影。 */
static void row(uint8_t line, const char *format, ...)
{
    char text[22];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    (void)OLED_DrawText(s_device, 0, (int16_t)(line * 8u), text, OLED_FONT_6X8);
}
static void gray_text(uint8_t mask, char *text)
{
    uint8_t i;
    for (i = 0u; i < 8u; ++i) text[i] = (mask & (1u << i)) ? '1' : '0';
    text[8] = '\0';
}
static void angle_text(int16_t angle, char *text, size_t size)
{
    int magnitude = angle < 0 ? -(int)angle : (int)angle;
    (void)snprintf(text, size, "%c%03d.%d", angle < 0 ? '-' : '+', magnitude / 10, magnitude % 10);
}
static void remember(const ModeFSM_DebugFrame *f, uint8_t kind)
{
    History *h = &s_history[s_head];
    h->tick_ms = f->tick_ms; h->kind = kind; h->state = f->state;
    h->event = f->event; h->fault = f->fault; h->mask = f->raw_mask;
    s_head = (uint8_t)((s_head + 1u) % HISTORY_SIZE);
    if (s_count < HISTORY_SIZE) ++s_count;
    /* 查看旧记录时保持其位置，避免新事件不停把阅读内容向下推走。 */
    if (!s_hold && s_scroll > 0u && s_scroll < s_count - 1u) ++s_scroll;
}

void OLED_Debug_OnFrame(const ModeFSM_DebugFrame *f)
{
    uint8_t gap;
    if (f == NULL) return;
    gap = s_have_frame && (uint32_t)(f->tick_ms - s_previous.tick_ms) != MODEFSM_TICK_MS;
    if (gap) remember(f, 4u);
    if (!s_have_frame || f->fault != s_previous.fault || f->event_count != s_previous.event_count ||
        f->state != s_previous.state) {
        uint8_t kind = 0u;
        if (f->fault != 0u && (!s_have_frame || f->fault != s_previous.fault)) kind = 3u;
        else if (s_have_frame && f->event_count != s_previous.event_count) kind = 1u;
        else if (!gap && s_have_frame && s_previous.state == STATE_JUNCTION_PENDING &&
                 f->state == STATE_NORMAL_TRACK) kind = 2u;
        remember(f, kind);
    }
    s_previous = *f;
    s_have_frame = 1u;
}

void OLED_Debug_Init(OLED_Device *device, uint32_t now)
{
    s_device = device;
    s_head = s_count = s_scroll = s_page = s_hold = s_have_frame = s_last_fault = 0u;
    s_frozen_head = s_frozen_count = 0u;
    s_redraw = 1u;
    s_last_render = now;
    memset(s_keys, 0, sizeof(s_keys));
    memset(&s_view, 0, sizeof(s_view));
}

/* 低电平按下；前台非阻塞消抖，不重复触发长按，不操作车辆状态。 */
static uint8_t key_pressed(uint8_t i, GPIO_TypeDef *port, uint16_t pin, uint32_t now)
{
    uint8_t pressed = HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET;
    Key *k = &s_keys[i];
    if (pressed != k->raw) { k->raw = pressed; k->changed_ms = now; }
    if (k->stable != k->raw && (uint32_t)(now - k->changed_ms) >= 30u) {
        k->stable = k->raw;
        return k->stable;
    }
    return 0u;
}
static void keys_tick(uint32_t now)
{
    uint8_t count;
    if (key_pressed(0, LEFT_GPIO_Port, LEFT_Pin, now)) {
        s_page = (uint8_t)((s_page + PAGE_COUNT - 1u) % PAGE_COUNT); s_redraw = 1u;
    }
    if (key_pressed(1, RIGHT_GPIO_Port, RIGHT_Pin, now)) {
        s_page = (uint8_t)((s_page + 1u) % PAGE_COUNT); s_redraw = 1u;
    }
    count = s_hold ? s_frozen_count : s_count;
    if (key_pressed(2, UP_GPIO_Port, UP_Pin, now) && s_page == 2u && count != 0u) {
        if (s_scroll + 1u < count) ++s_scroll;
        s_redraw = 1u;
    }
    if (key_pressed(3, DOWN_GPIO_Port, DOWN_Pin, now) && s_page == 2u) {
        if (s_scroll != 0u) --s_scroll;
        s_redraw = 1u;
    }
    if (key_pressed(4, OK_GPIO_Port, OK_Pin, now)) {
        s_hold = (uint8_t)!s_hold;
        if (s_hold) {
            ModeFSM_GetDiagnostics(&s_view);
            memcpy(s_frozen_history, s_history, sizeof(s_history));
            s_frozen_head = s_head; s_frozen_count = s_count;
        }
        s_redraw = 1u;
    }
}
static void render(void)
{
    char gray[9], yaw[12], target[12];
    const ModeFSM_t *f = &s_view.fsm;
    OLED_Status oled = OLED_GetStatus(s_device);
    uint32_t mhz = SystemCoreClock / 1000000u;
    if (mhz == 0u) mhz = 1u;
    gray_text(f->raw_mask, gray);
    if (s_view.yaw_valid) angle_text(s_view.yaw_ddeg, yaw, sizeof(yaw));
    else (void)snprintf(yaw, sizeof(yaw), "--");
    angle_text((int16_t)(f->target_yaw * 10.0f), target, sizeof(target));
    OLED_Clear(s_device, 0u);
    if (s_page == 0u) {
        row(0, "RUN 1/5 %s", s_hold ? "HOLD" : "LIVE");
        row(1, "S:%s F:%u", state_name((uint8_t)f->state), (unsigned)f->fault);
        row(2, "G:%s IN1->8", gray);
        row(3, "PWM:%+5d/%+5d", (int)s_view.pwm_left, (int)s_view.pwm_right);
        row(4, "IMU:%u Y:%s", (unsigned)s_view.imu_ready,
            !s_view.yaw_valid ? "INVALID" : (s_view.yaw_fresh ? "FRESH" : "STALE"));
        row(5, "Y:%s T:%s", yaw, target);
        row(6, "EV:%s #%lu", event_name((uint8_t)f->last_event), (unsigned long)f->event_count);
        row(7, "DROP:%lu", (unsigned long)f->debug_dropped);
    } else if (s_page == 1u) {
        row(0, "JUNCTION 2/5 %s", s_hold ? "HOLD" : "LIVE");
        row(1, "%s N:%lu", state_name((uint8_t)f->state), (unsigned long)f->state_frames);
        row(2, "G:%s", gray);
        row(3, "VOTE L:%u R:%u", (unsigned)f->left_votes, (unsigned)f->right_votes);
        row(4, "FRESH:%u/%u AGE:%u/%u", (unsigned)s_view.left_fresh, (unsigned)s_view.right_fresh,
            (unsigned)s_view.left_age, (unsigned)s_view.right_age);
        if (f->state == STATE_JUNCTION_PENDING)
            row(5, "CENTER:%u/%u", (unsigned)s_view.center_gap, (unsigned)JUNCTION_CENTER_GAP_FRAMES);
        else
            row(5, "CENTER:%u OBS", (unsigned)s_view.center_gap);
        row(6, "LINE:%u TURN:%u", (unsigned)s_view.narrow_frames, (unsigned)s_view.turn_done);
        row(7, "ADV:%lu/%u", (unsigned long)s_view.advance_frames, (unsigned)TURN_APPROACH_FRAMES);
    } else if (s_page == 2u) {
        uint8_t i, count = s_hold ? s_frozen_count : s_count;
        uint8_t head = s_hold ? s_frozen_head : s_head;
        const History *history = s_hold ? s_frozen_history : s_history;
        row(0, "HISTORY 3/5 %s", s_hold ? "HOLD" : "LIVE");
        for (i = 0u; i < 7u && (unsigned)s_scroll + i < count; ++i) {
            uint8_t index = (uint8_t)((head + HISTORY_SIZE - 1u - s_scroll - i) % HISTORY_SIZE);
            const History *h = &history[index];
            const char *label = state_name(h->state);
            if (h->kind == 1u) label = event_name(h->event);
            else if (h->kind == 2u) label = "CANCEL";
            else if (h->kind == 3u) label = fault_name(h->fault);
            else if (h->kind == 4u) label = "LOG_GAP";
            row((uint8_t)(i + 1u), "%7lu %s", (unsigned long)(h->tick_ms / 10u), label);
        }
        if (count == 0u) row(1, "NO EVENTS");
    } else if (s_page == 3u) {
        row(0, "FAULT 4/5 %s", s_hold ? "HOLD" : "LIVE");
        row(1, "F:%u %s", (unsigned)f->fault, fault_name((uint8_t)f->fault));
        if (s_view.fault_capture_valid) {
            /* 控制器在发停车命令前锁存现场；之后车体移动不会覆盖故障证据。
             * IMU 状态仍取当前快照，不能把通信恢复误当作故障样本有效。
             * PRE 是停车前最后一次 PWM 指令，不是车轮实际速度。 */
            gray_text(s_view.fault_mask, gray);
            if (s_view.fault_yaw_valid)
                angle_text(s_view.fault_yaw_ddeg, yaw, sizeof(yaw));
            else
                (void)snprintf(yaw, sizeof(yaw), "--");
            angle_text(s_view.fault_target_ddeg, target, sizeof(target));
            row(2, "AT:%s N:%lu", state_name(s_view.fault_state), (unsigned long)s_view.fault_frames);
            row(3, "G:%s", gray);
            row(4, "IMU:%u Y:%s", (unsigned)s_view.imu_ready,
                s_view.fault_yaw_valid ? "FAULT-SAMPLE" : "INVALID");
            row(5, "Y:%s T:%s", yaw, target);
            row(7, "PRE:%+5d/%+5d", (int)s_view.fault_pwm_left, (int)s_view.fault_pwm_right);
        } else {
            row(2, "S:%s N:%lu", state_name((uint8_t)f->state), (unsigned long)f->state_frames);
            row(3, "G:%s", gray);
            row(4, "IMU:%u Y:%s", (unsigned)s_view.imu_ready, s_view.yaw_fresh ? "FRESH" : "STALE");
            row(5, "Y:%s T:%s", yaw, target);
            row(7, "I2C:%lu DROP:%lu", (unsigned long)oled.error_count, (unsigned long)f->debug_dropped);
        }
        row(6, "EV:%s #%lu", event_name((uint8_t)f->last_event), (unsigned long)f->event_count);
    } else {
        row(0, "PERF 5/5 %s", s_hold ? "HOLD" : "LIVE");
        row(1, "CPU:%luMHz", (unsigned long)mhz);
        row(2, "CTL:%luus", (unsigned long)(s_view.control_last_cycles / mhz));
        row(3, "MAX:%luus", (unsigned long)(s_view.control_max_cycles / mhz));
        row(4, "OVER10ms:%lu", (unsigned long)s_view.control_overruns);
        row(5, "GAPMAX:%luus", (unsigned long)(s_view.interval_max_cycles / mhz));
        row(6, "LATE11ms:%lu", (unsigned long)s_view.late_intervals);
        row(7, "DROP:%lu I2C:%lu", (unsigned long)f->debug_dropped, (unsigned long)oled.error_count);
    }
}

void OLED_Debug_Tick(uint32_t now)
{
    ModeFSM_Diagnostics latest;
    OLED_Status status;
    if (s_device == NULL) return;
    keys_tick(now);
    ModeFSM_GetDiagnostics(&latest);
    if (latest.fsm.fault != MODE_FAULT_NONE && latest.fsm.fault != s_last_fault) {
        s_page = 3u; s_hold = 0u; s_redraw = 1u;
    }
    s_last_fault = (uint8_t)latest.fsm.fault;
    status = OLED_GetStatus(s_device);
    /* 等待前一画面传完；冻结只冻结观测，日志与车辆控制始终继续。 */
    if (!status.busy && (s_redraw || (!s_hold && (uint32_t)(now - s_last_render) >= 200u))) {
        if (!s_hold) s_view = latest;
        render();
        if (OLED_Present(s_device) == OLED_OK) {
            s_redraw = 0u; s_last_render = now;
        }
    }
    (void)OLED_Service(s_device, now);
}
