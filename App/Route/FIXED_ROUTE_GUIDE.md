# 固定路线接入指南

适用情形：起点和路线已知，在依次经过的路口执行左转、直行、右转或停车。

本文件提供接入模板，尚未修改或接入现有固件。下面的“左、直、右、停”只是示例，需要按实际路线修改。

## 1. 代码写在哪里

第一版直接写在 `App/Route/Route.c`：路线动作表负责选择动作，现有 Route 状态机负责前移、转弯、穿越和等待完成。保持 `Route_Enable(1u)`。

现有分工：

```text
灰度确认新路口
    → 路线动作表选择 LEFT / STRAIGHT / RIGHT / STOP
    → Route 安排前移、转弯或穿越
    → Car_* 提交一次动作请求
    → ModeFSM_Tick 选择控制器
    → PID 和电机输出
```

`junction` 是检测到的路口类型；`action` 是计划执行的动作。两者不能绑定：在十字路口也可以按路线选择左转或右转。

## 2. 在 Route.c 的静态变量区域加入动作表

放在现有 `s_turn_count_before` 定义之后、辅助函数之前：

```c
typedef enum {
    ROUTE_ACTION_LEFT = 0,
    ROUTE_ACTION_STRAIGHT,
    ROUTE_ACTION_RIGHT,
    ROUTE_ACTION_STOP
} RouteAction;

/* 示例：第一个新路口左转，第二个直行，第三个右转，第四个停车。
 * 请按真实路线修改；每个表项对应一个依次确认的新路口。 */
static const RouteAction s_route_plan[] = {
    ROUTE_ACTION_LEFT,
    ROUTE_ACTION_STRAIGHT,
    ROUTE_ACTION_RIGHT,
    ROUTE_ACTION_STOP
};

#define ROUTE_PLAN_COUNT \
    (sizeof(s_route_plan) / sizeof(s_route_plan[0]))

/* 只有已完成的动作才推进下标。 */
static uint32_t s_plan_index = 0u;
```

`STOP` 表示在该新路口被确认时发出停车请求，不表示自动定位到路口中心。若终点位于两路口之间，需要另行定义到达条件。

## 3. 在 Route_Init 中初始化下标

在 `Route_Init()` 内加入：

```c
s_plan_index = 0u;
```

`ModeFSM_Init()` 已经调用 `Route_Init()`，主程序不用重复初始化 Route。这个模板只支持从已知起点重新开始；重置下标不能代替车辆回到起点。

## 4. 替换 select_route_action 函数

用下面的完整函数替换当前 `select_route_action()`。现有 `ROUTE_APPROACH`、`ROUTE_TURN`、`ROUTE_CROSSING` 的执行机制继续复用。

```c
static void select_route_action(RouteJunction junction)
{
    RouteAction action;

    /* 表已用完：在本次新路口停车，避免越界。 */
    if (s_plan_index >= ROUTE_PLAN_COUNT) {
        DebugLog_Write(LOG_INFO, "PLAN_END");
        Car_Stop();
        return;
    }

    action = s_route_plan[s_plan_index];

    if (action == ROUTE_ACTION_STOP) {
        DebugLog_Write(LOG_INFO, "PLAN_END");
        Car_Stop();
        return;
    }

    /* 按当前粗分类检查转向是否匹配：十字可左转或右转。
     * 直行出口是否存在须由已知地图保证，当前检测不能完整识别所有出口。 */
    if ((action == ROUTE_ACTION_LEFT &&
         junction != JUNCTION_LEFT && junction != JUNCTION_CROSS) ||
        (action == ROUTE_ACTION_RIGHT &&
         junction != JUNCTION_RIGHT && junction != JUNCTION_CROSS)) {
        DebugLog_Write(LOG_ERROR, "PLAN_BAD");
        Car_Stop();
        return;
    }

    if (action == ROUTE_ACTION_LEFT || action == ROUTE_ACTION_RIGHT) {
        /* 转向由动作表决定，不能直接沿用检测结果。 */
        s_turn_direction = (action == ROUTE_ACTION_LEFT) ?
                           JUNCTION_LEFT : JUNCTION_RIGHT;
        g_route.state = ROUTE_APPROACH;
    } else if (action == ROUTE_ACTION_STRAIGHT) {
        g_route.state = ROUTE_CROSSING;
    } else {
        DebugLog_Write(LOG_ERROR, "PLAN_BAD");
        Car_Stop();
        return;
    }

    g_route.state_frames = 0u;
    Car_ImuForward();
}
```

确认左/右路口不一定代表所有出口都已正确识别。模板沿用当前灰度分类假设；路线动作必须与实际路口布局匹配。`PLAN_BAD` 表示模板发现转向与检测不匹配，并提交停车请求，未新增 `CarFault` 枚举。

## 5. 只在动作完成时推进下标

在 `cancel_route_step()` 定义之后、`select_route_action()` 之前新增：

```c
static void finish_route_step(void)
{
    if (s_plan_index < ROUTE_PLAN_COUNT) {
        ++s_plan_index;
    }
    cancel_route_step();
}
```

将 `Route_Tick()` 内的 `ROUTE_TURN` 分支替换为：

```c
case ROUTE_TURN:
    g_route.state_frames = g_mode_fsm.state_frames;
    if (g_mode_fsm.turn_count != s_turn_count_before) {
        /* 运动层已经自动切回循迹，这里只完成路线表中的当前动作。 */
        finish_route_step();
    }
    break;
```

将 `ROUTE_CROSSING` 分支替换为：

```c
case ROUTE_CROSSING:
    g_route.state_frames = g_mode_fsm.state_frames;
    if (g_route.state_frames >= ROUTE_CROSSING_FRAMES) {
        Car_Track();
        finish_route_step();
    }
    break;
```

其余用于外部接管、停车、故障和关闭自动路线的 `cancel_route_step()` 调用保持原样。

不要在 `cancel_route_step()` 内推进下标，因为取消动作不等于完成动作。不要在每次 `Route_Tick()` 或刚发请求时推进下标。

动作完成后只把下一动作设为待使用；下一次新路口事件到来才开始下一动作。转弯完成或穿越计时结束，不等于已到下一路口。

## 6. 启动和工程配置

本方案只修改已经参与构建的 `Route.c`，不需要新增 Keil Group 或 Include Path。

若要在主程序显式配置开关，在 `Core/Src/main.c` 的用户 include 区加入：

```c
#include "Route.h"
```

在 `ModeFSM_Init()` 之后、`HAL_TIM_Base_Start_IT(&htim4)` 之前加入：

```c
Route_Enable(1u);  /* 默认已经开启，此行明确保留 Route 的动作执行步骤。 */
```

本方案不使用 `Route_Enable(0)`。关闭后现有前移和转弯等待步骤会被取消，且不会自动停车。

不要在主循环重复调用 `Route_Tick()` 或 `ModeFSM_Tick()`。不要每帧重复发送 `Car_*()`，否则动作计时、PID 和角度基准会被不断重置。

## 7. 配置实际时间

当前源代码一帧约 10 ms：

| 配置 | 当前值 | 用途 |
| --- | --- | --- |
| `ROUTE_CONFIRM_FRAMES` | 3 | 左右路口和有线/无线的连续确认 |
| `ROUTE_CROSS_CONFIRM_FRAMES` | 2 | 十字路口的连续确认 |
| `ROUTE_APPROACH_FRAMES` | 80 | 转弯前移约 800 ms |
| `ROUTE_CROSSING_FRAMES` | 500 | 直行穿越约 5 s |

`ROUTE_CROSSING_FRAMES` 的“100 ms”注释与 500 的实际值不一致。请根据真实车速和路口尺寸设置穿越时间；若意图为 100 ms，对应 10 帧，但这不是车辆标定结果。

计时近似前移距离，没有编码器不能直接配置为准确的厘米数。确认顺序和转向正确之后，再在车辆上标定时间参数。

## 8. 固定路线执行的边界

- 必须从已知起点和朝向启动，路线表从下标 0 开始。
- 每个需要决策的位置都应能被当前路口检测识别；纯弯道不一定产生路口事件。
- 前移、转弯或穿越期间检测到的新路口事件不会保存到恢复循迹以后。当前检测在所有模式中更新并锁存事件，只有等待循迹时才消费动作表；应确保这些步骤在进入下一个路口前完成。密集路口需要额外设计事件保存和定位确认机制，不能只增加穿越时间。
- 漏检或误检路口会让动作表与实际位置错位。该模板按路口顺序执行，没有地图节点定位能力。
- 外部请求、故障或关闭路线中断步骤后，不把中断视为成功。车辆位置改变后应重新确认起点或另行设计恢复机制，再继续路线。
- `STOP` 表项在新路口确认时停车。若需要停在路口中心或某个终点标记，应增加独立的终点检测/接近步骤。

模板未接入现有固件，也未做 Keil 或车辆验证。接入后应先检查路线下标和动作日志，再验证车辆上的路口识别、转向和时间参数。
