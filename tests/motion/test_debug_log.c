/**
 * @file test_debug_log.c
 * @brief 编译真实 Debug_Log.c，验证 OLED 日志内容、故障快照与显示并发边界。
 * @note 只替代 HAL 时间、OLED 总线与由控制层维护的全局数据，不复制日志实现。
 */
#include "Debug_Log.h"
#include "Route.h"
#include "Track_Task.h"
#include "Attitude.h"
#include "Int_MPU6050.h"
#include "motor.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL [%s] line %d: %s\n", s_case_name, __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

volatile ModeFSM_t g_mode_fsm;
volatile Route_t g_route;
Motor_Struct motorLeft;
Motor_Struct motorRight;
uint16_t g_sensor_data[GRAYSCALE_SENSOR_CHANNELS];
uint8_t g_imu_ready;
Euler_struct g_euler;

static const char *s_case_name;
static uint32_t s_milliseconds;
static uint32_t s_irq_mask;
static uint32_t s_expected_display_mask;
static unsigned s_rows_written;
static unsigned s_updates;
static uint8_t s_change_after_copy;
static OLED_Status s_update_status;
static char s_oled_rows[OLED_PAGE_COUNT][OLED_LINE_CHARS + 1u];

/**
 * @brief 返回测试设置的时间，允许检查开机秒数与 32 位最大时间边界。
 * @param 无。
 * @return 当前模拟的毫秒计数。
 * @note 不递增；每个场景自行设置精确时间，生产模块照常调用 HAL_GetTick。
 */
uint32_t HAL_GetTick(void)
{
    return s_milliseconds;
}

/**
 * @brief 返回测试维护的中断屏蔽状态。
 * @param 无。
 * @return 0 表示可中断，1 表示调用前已屏蔽。
 * @note 与真实 CMSIS 名称一致，供日志模块使用真实临界区逻辑。
 */
uint32_t __get_PRIMASK(void)
{
    return s_irq_mask;
}

/**
 * @brief 模拟短临界区进入，供快照和记录函数保护共享数据。
 * @param 无。
 * @return 无。
 * @note 仅修改测试状态，不锁住宿主线程。
 */
void __disable_irq(void)
{
    s_irq_mask = 1u;
}

/**
 * @brief 模拟恢复调用前的中断屏蔽状态。
 * @param mask 原中断屏蔽位，允许 0 或 1。
 * @return 无。
 * @note 所有记录与复制操作都必须保存调用者状态，不能无条件启用中断。
 */
void __set_PRIMASK(uint32_t mask)
{
    CHECK(mask <= 1u);
    s_irq_mask = mask;
}

/**
 * @brief 保存生产日志模块生成的实际 OLED 文本并检查每行显示宽度。
 * @param row 显示行号，0 至 7。
 * @param format 生产模块选择的标准 printf 格式字符串。
 * @param ... 与 format 对应的参数。
 * @return 无。
 * @note 格式化前检查已恢复原 PRIMASK；完整格式化后检查长度，避免 OLED 裁剪掩盖布局溢出。
 *       并发场景在首次绘制时模拟下一控制帧更新，当前整屏应仍使用同一份已复制快照。
 */
void OLED_PrintLine(uint8_t row, const char *format, ...)
{
    char rendered[128];
    int length;
    va_list arguments;
    CHECK(row < OLED_PAGE_COUNT && format != NULL);
    CHECK(s_irq_mask == s_expected_display_mask);
    if (s_change_after_copy != 0u) {
        s_change_after_copy = 0u;
        g_mode_fsm.target_yaw = -40.0f;
        g_mode_fsm.state_frames = 888u;
        g_euler.yaw = -35.0f;
        motorLeft.speed = -333;
        motorRight.speed = -444;
        DebugLog_Write(LOG_INFO, "NEXT_FRAME");
    }
    va_start(arguments, format);
    length = vsnprintf(rendered, sizeof(rendered), format, arguments);
    va_end(arguments);
    CHECK(length >= 0 && length <= (int)OLED_LINE_CHARS);
    memset(s_oled_rows[row], 0, sizeof(s_oled_rows[row]));
    memcpy(s_oled_rows[row], rendered, (size_t)length + 1u);
    ++s_rows_written;
}

/**
 * @brief 记录刷新调用，返回场景指定的 OLED 通信结果。
 * @param 无。
 * @return 本场景设置的 OLED_Status，可模拟断连和重新连接。
 * @note 只替代通信；日志记录、格式化与刷新调用全部来自真实生产模块。
 */
OLED_Status OLED_Update(void)
{
    CHECK(s_irq_mask == s_expected_display_mask);
    ++s_updates;
    return s_update_status;
}

/**
 * @brief 初始化独立场景的状态及显示记录。
 * @param name 场景名称，用于定位断言失败。
 * @return 无。
 * @note 先准备上层全局数据，再调用真实 DebugLog_Init；不会模拟 ModeFSM_Init 的 BOOT 自动记录。
 */
static void begin_case(const char *name)
{
    unsigned i;
    s_case_name = name;
    memset((void *)&g_mode_fsm, 0, sizeof(g_mode_fsm));
    memset((void *)&g_route, 0, sizeof(g_route));
    memset(&motorLeft, 0, sizeof(motorLeft));
    memset(&motorRight, 0, sizeof(motorRight));
    memset(&g_euler, 0, sizeof(g_euler));
    memset(s_oled_rows, 0, sizeof(s_oled_rows));
    for (i = 0u; i < GRAYSCALE_SENSOR_CHANNELS; ++i) {
        g_sensor_data[i] = (uint16_t)((i == 0u || i == 7u) ? LINE_RAW_VALUE : !LINE_RAW_VALUE);
    }
    g_imu_ready = 1u;
    s_milliseconds = s_irq_mask = s_expected_display_mask = 0u;
    s_rows_written = s_updates = 0u;
    s_change_after_copy = 0u;
    s_update_status = OLED_OK;
    DebugLog_Init();
}

/**
 * @brief 刷新整屏并检查一次调用完整绘制八行、刷新一次且恢复中断状态。
 * @param 无。
 * @return 实际 OLED 刷新返回值，便于通信失败场景继续断言。
 * @note OLED fake 保留完整行文本，不参与真实日志格式选择。
 */
static OLED_Status display(void)
{
    unsigned rows_before = s_rows_written;
    unsigned updates_before = s_updates;
    uint32_t mask_before = s_irq_mask;
    OLED_Status status;
    s_expected_display_mask = mask_before;
    status = DebugLog_Display();
    CHECK(s_rows_written == rows_before + OLED_PAGE_COUNT);
    CHECK(s_updates == updates_before + 1u);
    CHECK(s_irq_mask == mask_before);
    return status;
}

/**
 * @brief 检查日志三行是否出现指定消息。
 * @param message 需要寻找的普通 ASCII 文本。
 * @return 1 表示至少一条历史记录包含消息，0 表示没有。
 * @note 搜索仅限第 5 至 7 行，不把状态文本误认为日志事件。
 */
static int has_message(const char *message)
{
    unsigned row;
    for (row = 5u; row < OLED_PAGE_COUNT; ++row) {
        if (strstr(s_oled_rows[row], message) != NULL) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 验证三条容量、时间、等级、值复制、截断及非格式字符串输入。
 * @param 无。
 * @return 无。
 * @note 覆盖 NULL、空串、非法等级不插入，以及控制字符和非 ASCII 字节被替换为问号。
 */
static void test_records_and_text(void)
{
    char text[] = "COPY_ME";
    begin_case("three records, text limits and seconds");
    CHECK(display() == OLED_OK);
    CHECK(s_oled_rows[5][0] == '\0' && s_oled_rows[6][0] == '\0' && s_oled_rows[7][0] == '\0');
    s_milliseconds = 999u;
    DebugLog_Write(LOG_INFO, "FIRST");
    display();
    CHECK(s_oled_rows[5][0] == '\0' && s_oled_rows[6][0] == '\0');
    CHECK(strstr(s_oled_rows[7], "0 I FIRST") != NULL);
    s_milliseconds = 12001u;
    DebugLog_Write(LOG_WARN, text);
    display();
    CHECK(s_oled_rows[5][0] == '\0' && strstr(s_oled_rows[6], "0 I FIRST") != NULL);
    CHECK(strstr(s_oled_rows[7], "12 W COPY_ME") != NULL);
    strcpy(text, "CHANGED");
    s_milliseconds = 16099u;
    DebugLog_Write(LOG_ERROR, "THIRD");
    CHECK(display() == OLED_OK);
    CHECK(strstr(s_oled_rows[5], "0 I FIRST") != NULL);
    CHECK(strstr(s_oled_rows[6], "12 W COPY_ME") != NULL);
    CHECK(strstr(s_oled_rows[7], "16 E THIRD") != NULL);
    DebugLog_Write(LOG_INFO, NULL);
    DebugLog_Write(LOG_INFO, "");
    DebugLog_Write((LogLevel)99, "BAD_LEVEL");
    display();
    CHECK(has_message("FIRST") && !has_message("CHANGED") && !has_message("BAD_LEVEL"));
    DebugLog_Write(LOG_INFO, "12345678901EXCESS");
    DebugLog_Write(LOG_WARN, "%s%n%%");
    DebugLog_Write(LOG_INFO, "A\n\001\377Z");
    display();
    CHECK(!has_message("FIRST") && !has_message("THIRD"));
    CHECK(strstr(s_oled_rows[5], "12345678901") != NULL && !has_message("EXCESS"));
    CHECK(strstr(s_oled_rows[6], "%s%n%%") != NULL);
    CHECK(strstr(s_oled_rows[7], "A???Z") != NULL);
    puts("PASS three-record ordering, copy, 11-byte limit, levels, seconds and safe text");
}

/**
 * @brief 验证循迹旧角度隐藏、IMU 有效性、最短角差及极端字段不会溢出屏幕。
 * @param 无。
 * @return 无。
 * @note 分别输入 NaN、Inf、FLT_MAX、UINT32_MAX，检查无需读取真实外设的显示安全边界。
 */
static void test_status_and_numeric_limits(void)
{
    begin_case("mode, fresh yaw, shortest error and numeric bounds");
    g_euler.yaw = 25.0f;
    g_mode_fsm.target_yaw = 90.0f;
    g_mode_fsm.imu_valid = 1u;
    display();
    CHECK(strstr(s_oled_rows[1], "Y:--") != NULL && strstr(s_oled_rows[1], "T:--") != NULL);
    CHECK(strstr(s_oled_rows[2], "E:--") != NULL);
    CHECK(strstr(s_oled_rows[3], "10000001") != NULL);
    g_mode_fsm.mode = MODE_IMU;
    g_mode_fsm.target_yaw = 170.0f;
    g_euler.yaw = -170.0f;
    g_mode_fsm.state_frames = 299u;
    motorLeft.speed = -427;
    motorRight.speed = 427;
    display();
    CHECK(strstr(s_oled_rows[1], "Y:-170.0") != NULL && strstr(s_oled_rows[1], "T:+170.0") != NULL);
    CHECK(strstr(s_oled_rows[2], "E:-20.0") != NULL && strstr(s_oled_rows[2], "N:299") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:-427") != NULL && strstr(s_oled_rows[4], "R:+427") != NULL);
    g_mode_fsm.imu_valid = 0u;
    display();
    CHECK(strstr(s_oled_rows[1], "Y:--") != NULL && strstr(s_oled_rows[1], "T:+170.0") != NULL);
    CHECK(strstr(s_oled_rows[2], "E:--") != NULL);
    g_mode_fsm.imu_valid = 1u;
    g_euler.yaw = NAN;
    display();
    CHECK(strstr(s_oled_rows[1], "Y:--") != NULL && strstr(s_oled_rows[2], "E:--") != NULL);
    g_euler.yaw = INFINITY;
    g_mode_fsm.target_yaw = NAN;
    display();
    CHECK(strstr(s_oled_rows[1], "Y:--") != NULL && strstr(s_oled_rows[1], "T:--") != NULL);
    g_euler.yaw = 3.4028234e38f;
    g_mode_fsm.target_yaw = -3.4028234e38f;
    g_mode_fsm.state_frames = UINT32_MAX;
    motorLeft.speed = -1000;
    motorRight.speed = 1000;
    s_milliseconds = UINT32_MAX;
    DebugLog_Write(LOG_WARN, "LONGEST_LOG");
    display();
    CHECK(has_message("LONGEST_LOG"));
    CHECK(strstr(s_oled_rows[4], "L:-1000") != NULL && strstr(s_oled_rows[4], "R:+1000") != NULL);
    puts("PASS mode display, IMU validity, angular wrap and extreme field widths");
}

/**
 * @brief 验证首次故障锁存停车前数据，后续写入或故障不覆盖，显式清除保留历史。
 * @param 无。
 * @return 无。
 * @note 模拟控制层在 CaptureFault 后停车及更新灰度，显示必须保留故障时的原始现场。
 */
static void test_fault_snapshot(void)
{
    char first_screen[sizeof(s_oled_rows)];
    begin_case("first pre-stop fault snapshot and frozen history");
    g_mode_fsm.mode = MODE_IMU;
    g_mode_fsm.imu_state = IMU_TURN_LEFT;
    g_mode_fsm.imu_valid = 1u;
    g_mode_fsm.target_yaw = 90.0f;
    g_mode_fsm.state_frames = 300u;
    g_route.state = ROUTE_TURN;
    g_euler.yaw = 37.2f;
    motorLeft.speed = -427;
    motorRight.speed = 427;
    DebugLog_Write(LOG_INFO, "JUNC_L");
    DebugLog_Write(LOG_INFO, "L90>TRK");
    DebugLog_CaptureFault(CAR_FAULT_NONE);
    DebugLog_Write(LOG_INFO, "PRE_FAULT");
    s_irq_mask = 1u;
    DebugLog_CaptureFault(CAR_FAULT_TURN_TIMEOUT);
    CHECK(s_irq_mask == 1u);
    s_irq_mask = 0u;
    CHECK(g_mode_fsm.status == CAR_RUNNING && motorLeft.speed == -427);
    display();
    CHECK(strstr(s_oled_rows[0], "FAULT") != NULL && strstr(s_oled_rows[0], "L90") != NULL);
    CHECK(strstr(s_oled_rows[0], "R:TURN") != NULL);
    CHECK(strstr(s_oled_rows[1], "Y:+37.2") != NULL && strstr(s_oled_rows[1], "T:+90.0") != NULL);
    CHECK(strstr(s_oled_rows[2], "E:+52.8") != NULL && strstr(s_oled_rows[2], "N:300") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:-427") != NULL && strstr(s_oled_rows[4], "R:+427") != NULL);
    CHECK(has_message("TURN_TO"));
    CHECK(strstr(s_oled_rows[7], " E TURN_TO") != NULL);
    memcpy(first_screen, s_oled_rows, sizeof(s_oled_rows));
    motorLeft.speed = motorRight.speed = 0;
    g_mode_fsm.mode = MODE_TRACK;
    g_mode_fsm.status = CAR_STOPPED;
    g_mode_fsm.target_yaw = 0.0f;
    g_mode_fsm.imu_valid = 0u;
    g_euler.yaw = 0.0f;
    g_route.state = ROUTE_FOLLOW;
    memset(g_sensor_data, 0, sizeof(g_sensor_data));
    g_imu_ready = 0u;
    DebugLog_Write(LOG_INFO, "AFTER_FAULT");
    DebugLog_CaptureFault(CAR_FAULT_IMU);
    display();
    CHECK(memcmp(first_screen, s_oled_rows, sizeof(s_oled_rows)) == 0);
    s_irq_mask = 1u;
    DebugLog_ClearFault();
    CHECK(s_irq_mask == 1u);
    s_irq_mask = 0u;
    display();
    CHECK(strstr(s_oled_rows[0], "FAULT") == NULL && has_message("TURN_TO"));
    DebugLog_Write(LOG_INFO, "RESUMED");
    display();
    CHECK(has_message("RESUMED") && has_message("TURN_TO"));
    puts("PASS first fault, pre-stop values, frozen log and explicit unfreeze");
}

/**
 * @brief 验证快照一致性、已有中断屏蔽恢复及 OLED 断连后的历史重绘。
 * @param 无。
 * @return 无。
 * @note 首次绘制后修改真实全局数据，当前屏幕和下一屏幕分别反映修改前与修改后。
 */
static void test_snapshot_irq_and_reconnect(void)
{
    char failed_screen[sizeof(s_oled_rows)];
    begin_case("consistent foreground snapshot, IRQ restore and reconnect");
    g_mode_fsm.mode = MODE_IMU;
    g_mode_fsm.imu_valid = 1u;
    g_mode_fsm.target_yaw = 90.0f;
    g_mode_fsm.state_frames = 12u;
    g_euler.yaw = 30.0f;
    motorLeft.speed = 111;
    motorRight.speed = 222;
    DebugLog_Write(LOG_INFO, "BEFORE_COPY");
    s_change_after_copy = 1u;
    display();
    CHECK(strstr(s_oled_rows[1], "Y:+30.0") != NULL && strstr(s_oled_rows[1], "T:+90.0") != NULL);
    CHECK(strstr(s_oled_rows[2], "N:12") != NULL);
    CHECK(strstr(s_oled_rows[4], "L:+111") != NULL && strstr(s_oled_rows[4], "R:+222") != NULL);
    CHECK(has_message("BEFORE_COPY") && !has_message("NEXT_FRAME"));
    display();
    CHECK(strstr(s_oled_rows[1], "Y:-35.0") != NULL && strstr(s_oled_rows[2], "N:888") != NULL);
    CHECK(has_message("NEXT_FRAME"));
    s_irq_mask = 1u;
    DebugLog_Write(LOG_INFO, "MASKED");
    CHECK(s_irq_mask == 1u);
    display();
    CHECK(s_irq_mask == 1u && has_message("MASKED"));
    s_irq_mask = 0u;
    s_update_status = OLED_IO_ERROR;
    CHECK(display() == OLED_IO_ERROR);
    memcpy(failed_screen, s_oled_rows, sizeof(s_oled_rows));
    s_update_status = OLED_OK;
    memset(s_oled_rows, 0, sizeof(s_oled_rows));
    CHECK(display() == OLED_OK);
    CHECK(memcmp(failed_screen, s_oled_rows, sizeof(s_oled_rows)) == 0);
    puts("PASS one coherent snapshot, preserved PRIMASK and OLED reconnect history");
}

/**
 * @brief 运行真实日志模块的独立行为测试。
 * @param 无。
 * @return EXIT_SUCCESS 表示所有场景通过；断言失败由 CHECK 直接结束。
 * @note 软件测试不验证实际 OLED 亮度、接线与现场可读性。
 */
int main(void)
{
    test_records_and_text();
    test_status_and_numeric_limits();
    test_fault_snapshot();
    test_snapshot_irq_and_reconnect();
    puts("All debug-log scenarios passed.");
    return EXIT_SUCCESS;
}
