#ifndef __INT_SR04_H__
#define __INT_SR04_H__

#include "main.h"

/** 测量状态；只有 SR04_OK 对应有效的回波宽度和距离。 */
typedef enum
{
    SR04_IDLE = 0,   /* 初始化完成，尚未接受测量请求。 */
    SR04_BUSY,       /* 已发送触发脉冲，正在等待 Echo 上升沿或下降沿。 */
    SR04_OK,         /* 最近一次测量完成，可重复读取，直到接受下一次请求。 */
    SR04_TIMEOUT    /* 本次请求超过 30 ms，仍未收到完整回波。 */
} SR04_Status;

/** 读取接口返回的一致快照；非 SR04_OK 时两个数值字段均为 0。 */
typedef struct
{
    SR04_Status status;  /* 本次读取时的测量状态。 */
    uint32_t pulse_us;   /* Echo 高电平宽度，单位 us，向下取整。 */
    float distance_cm;   /* 沿用旧工程的换算系数：pulse_us * 0.017，单位 cm。 */
} SR04_Result;

/**
 * @brief 启动时初始化 SR04 软件状态，并保持 Trig 为低电平。
 * @param 无。
 * @return 无。
 * @note 在系统时钟、Delay_Init() 和 MX_GPIO_Init() 完成后，由前台调用一次。
 *       复用 TR/EC 引脚和已运行的 DWT，不配置 GPIO、不清零 DWT、不操作 TIM4。
 */
void Int_SR04_Init(void);

/**
 * @brief 请求一次异步测距，只在发送约 12 us 的触发脉冲时短暂等待。
 * @param 无。
 * @return 1 表示本次请求已接受；0 表示正在测量、距上次触发不足 60 ms，或 Echo 尚未回低。
 * @note 初始化后由前台调用。接受请求后结果立即变为 SR04_BUSY，旧距离不再有效。
 *       回波通过 PA4 双边沿中断处理，调用者不需要等待回波或调用周期任务。
 */
uint8_t Int_SR04_Trigger(void);

/**
 * @brief 检查本次测量是否超时，并复制状态、回波宽度和距离。
 * @param result 接收结果的对象；传入空指针时直接返回。
 * @return 无。
 * @note 由前台调用；不等待回波，也不消耗已完成的结果。
 *       只有 status == SR04_OK 时才能使用距离；其余状态的数值字段均为 0。
 */
void Int_SR04_GetResult(SR04_Result *result);

#endif /* __INT_SR04_H__ */
