/**
 ******************************************************************************
 * @file       pid.h
 * @brief      优化版 PID 控制器头文件(位置式 + 增量式), 接口与原版兼容
 * 
 * 可移植性说明:
 *   - 本头文件自包含: 只依赖标准库 <stdint.h>, 不再 #include "main.h",
 *     与芯片无关, STM32 / 51 / DSP / Linux / Windows 均可直接编译
 *   - 原代码的 fp32 在大疆工程里就是 typedef float fp32(见其 main.h),
 *     这里统一用 pid_real_t 代替, 默认 float; 想要双精度时,
 *     在包含本头文件之前 #define PID_USE_DOUBLE 即可整体切换为 double
 *   - 纯 C89/C99 语法, 不依赖数学库(没用 math.h), 无浮点单元的 MCU 也能跑
 ******************************************************************************
 */

#ifndef __COM_PID_H__
#define __COM_PID_H__


// #include <stdint.h>   /* uint8_t */
#include "main.h"

//#define PID_USE_DOUBLE   /* 定义则整个 PID 使用双精度 double, 不定义则用单精度 float */
/* ==================== 可移植性配置区(按需修改) ==================== */
/* 定义 PID_USE_DOUBLE 则整个 PID 使用双精度浮点; 不定义则用单精度 float */
#ifdef PID_USE_DOUBLE
typedef double pid_real_t;   /* 双精度: 数值稳定性更好, 适合强算力平台 */
#else
typedef float  pid_real_t;   /* 单精度: 嵌入式默认, 速度快、RAM 占用小 */
#endif

/* PID 工作模式 */
typedef enum
{
    PID_POSITION = 0,   /* 位置式: 输出为最终控制量, 直接写入执行器 */
    PID_DELTA           /* 增量式: 每次输出为增量, 内部自动累加 */
} pid_mode_t;

/**
 * @brief PID结构体属性解释：
 * @param sep_threshold：积分分离阈值，一般填入目标值的的 5% ~ 20%    如果目标是电机转速 3000 RPM，阈值可能设为 150 RPM ~ 300 RPM
 *  
 */

/* PID 控制对象: 一个 PID 实例 = 一份结构体内存 */
typedef struct
{
    pid_mode_t mode;         /* 工作模式: PID_POSITION / PID_DELTA */

    /* ----- 参数(整定后一般不变, 可用 PID_set_param 在线改) ----- */
    pid_real_t Kp;           /* 比例系数 */
    pid_real_t Ki;           /* 积分系数(已含采样周期, 调用周期需固定) */
    pid_real_t Kd;           /* 微分系数(已含采样周期) */
    pid_real_t max_out;      /* 输出限幅 ±max_out */
    pid_real_t max_iout;     /* 积分限幅 ±max_iout(抗积分饱和) */

    /* ----- 可选优化项(初始化时默认关闭, 行为与原版一致) ----- */
    pid_real_t sep_threshold;    /* 积分分离阈值: |误差|>阈值时暂停积分, 0=禁用 ，用的是后直接填入值自动开启*/
    pid_real_t d_filter;         /* 微分一阶低通滤波系数 [0,1), 0=不滤波, 0.7~0.9 常用  值越大，滤波越强 */
    uint8_t    d_on_measurement; /* 微分作用在反馈上(抗微分突变, 仅位置式生效) 1=开启, 0=关闭 */

    /* ----- 运行状态(由 PID_calc 维护, 外部只读) ----- */
    pid_real_t set;          /* 本次设定值 */
    pid_real_t fdb;          /* 本次反馈值 */
    pid_real_t out;          /* 输出: 位置式=控制量, 增量式=累加后的控制量 */
    pid_real_t Pout;         /* 比例项 */
    pid_real_t Iout;         /* 积分项(位置式为累加值, 增量式为单次增量) */
    pid_real_t Dout;         /* 微分项(滤波后) */
    pid_real_t error[3];     /* 误差: [0]=本次 [1]=上次 [2]=上上次  这里记录3次是因为兼容增量式pid，一般位置式就用2个就够了*/ 
    pid_real_t Dprev;        /* 上一次滤波后的微分值(滤波用) */
    pid_real_t fdb_prev;     /* 上一次反馈值(微分作用于反馈时用) */
    uint8_t    initialized;  /* 内部标志: 首次调用时用当前值填充历史 */

} pid_type_def;

/* ==================== 函数声明 ==================== */

/* PID 结构体初始化(签名与原版一致, 原调用代码无需修改) */
void PID_init(pid_type_def *pid, uint8_t mode, const pid_real_t PID[3],
              pid_real_t max_out, pid_real_t max_iout);

/* PID 计算: 每个固定周期调用一次, ref=反馈值, set=设定值, 返回输出 */
pid_real_t PID_calc(pid_type_def *pid, pid_real_t ref, pid_real_t set);

/* 清空输出与历史状态(参数保留不清) */
void PID_clear(pid_type_def *pid);

/* 在线调参: 运行中修改 Kp/Ki/Kd */
void PID_set_param(pid_type_def *pid, pid_real_t Kp, pid_real_t Ki, pid_real_t Kd);

/* 微分滤波: alpha ∈ [0,1), 0=不滤波, 越大滤波越强(建议 0.7~0.9) */
void PID_set_deriv_filter(pid_type_def *pid, pid_real_t alpha);

/* 积分分离: |误差| > threshold 时暂停积分, 传 0 禁用 */
void PID_set_integral_separation(pid_type_def *pid, pid_real_t threshold);

/* 微分作用于反馈: enable=1 时设定值突变不会引起输出尖峰(仅位置式) */
void PID_set_deriv_on_measurement(pid_type_def *pid, uint8_t enable);


#endif /* __COM_PID_H__ */

