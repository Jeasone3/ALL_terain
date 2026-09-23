/**
 ******************************************************************************
 * @file       pid.c
 * @brief      PID 算法：(位置式 + 增量式)
 * @note       依赖: 仅本目录 pid.h，HAL库：main.h
 ******************************************************************************
 */
#include "Com_PID.h"


/* ==================== 内部工具函数 ==================== */

/**
 * @brief  限幅函数: 把 输出值 限制在 -（limit, +limit) 之间并返回
 *  {value} : 待限幅的值
 * {limit} : 限幅阈值, 输出范围为 [-limit, +limit]
 * @retval 限幅后的值
 *         
 */
static pid_real_t pid_limit(pid_real_t value, pid_real_t limit)
{
    if (value > limit)
        return limit;      /* 超上限 → 返回上限 */
    if (value < -limit)
        return -limit;     /* 超下限 → 返回下限 */
    return value;              /* 范围内 → 原样返回 */
}



/* ==================== 对外接口 ==================== */

/**
 * @brief  PID 初始化: 写入参数并把状态清零,防止上一阶段的PID的输出对这一阶段PID计算产生影响
 * @param  pid       PID 实例指针
 * @param  mode      PID_POSITION(位置式) / PID_DELTA(增量式)
 * @param  PID[3]    参数数组: [0]=Kp [1]=Ki [2]=Kd
 * @param  max_out   输出限幅(±)
 * @param  max_iout  积分限幅(±)
 */
void PID_init(pid_type_def *pid, uint8_t mode, const pid_real_t PID[3],
              pid_real_t max_out, pid_real_t max_iout)
{
    if (pid == NULL || PID == NULL)
        return;

    /* 写入模式与参数 */
    pid->mode = (pid_mode_t)mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;

    /* 可选优化项默认关闭 → 按需用 set 接口开启 0=关闭，填入值自动开启 */
    pid->d_filter = 0.0f; //微分滤波
    pid->sep_threshold = 0.0f; //积分分离
    pid->d_on_measurement = 0; //微分作用于反馈，防止微分突变，

    /* 状态清零(含 initialized 标志, 下次 calc 会重新填充历史) */
    //主要清楚上一次目标的误差历史, 避免误差累积
    PID_clear(pid);
}


//-----------------------------------set-------------------------------------------------//
/**
* @note :设置积分分离阈值，|误差| > threshold 时暂停积分, 传 0 禁用
*       设置微分一阶低通滤波系数，alpha ∈ [0,1), 0=不滤波, 越大滤波越强(建议 0.7~0.9)
*       开启/关闭"微分作用于反馈"(抗微分突变)，仅位置式生效
*/           


/**
 * @brief  设置微分一阶低通滤波系数
 * @param  alpha ∈ [0,1), 0=不滤波, 越大滤波越强(建议 0.7~0.9)
 */
void PID_set_deriv_filter(pid_type_def *pid, pid_real_t alpha)
{
    if (pid == NULL)
        return;
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha >= 1.0f)
        alpha = 0.95f;   /* 钳位上限, 防止微分项完全不更新 */
    pid->d_filter = alpha;
}

/**
 * @brief  设置积分分离阈值  设定积分分离阈值，|误差| > threshold 时暂停积分, 传 0 禁用
 * @param  threshold > 0: |误差| 超过它时暂停积分; 传 0 禁用
 */
void PID_set_integral_separation(pid_type_def *pid, pid_real_t threshold)
{
    if (pid == NULL)
        return;
    pid->sep_threshold = (threshold > 0.0f) ? threshold : 0.0f;
}

/**
 * @brief  开启/关闭"微分作用于反馈"(抗微分突变)
 *          解释：微分项直接作用到PID_out上，防止因反馈突变导致的微分突变
 * @param  enable: 1=开启, 0=关闭(与原版一致)
 * @note   仅位置式生效; 增量式仍用误差二阶差分
 */
void PID_set_deriv_on_measurement(pid_type_def *pid, uint8_t enable)
{
    if (pid == NULL)
        return;
    pid->d_on_measurement = (enable != 0) ? 1 : 0;
    /* 切换后用当前反馈填充历史, 避免用旧反馈算差分产生跳变 */
    pid->fdb_prev = pid->fdb;
}
//-----------------------------------end set-------------------------------------------------//




/**
 * @brief  PID 计算: 每个固定控制周期调用一次
 * @param  pid   PID 实例指针
 * @param  ref   反馈值(传感器实测)
 * @param  set   设定值(目标)
 * @retval 本次输出 out
 * @note   采样周期约定: Ki/Kd 已把采样周期乘进去了,
 *         所以必须在固定周期(如 1ms 定时中断)里调用;
 *         若周期改变, Ki/Kd 需要按比例重调
 */
pid_real_t PID_calc(pid_type_def *pid, pid_real_t ref, pid_real_t set)
{
    pid_real_t Dout_raw;   /* 未滤波的微分原始值 */

    if (pid == NULL)
        return 0.0f;

    /* 1. 误差历史后移: error[2]←error[1]←error[0], 再写入本次误差 */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    /* 2. 首次调用(或 PID_clear 之后): 用当前值填充历史。使平滑启动、防止积分饱和和防止微分冲击
     *    避免 e[1]、fdb_prev 初始为 0 造成的第一个周期输出尖峰 */
    if (pid->initialized == 0)
    {
        pid->error[1] = pid->error[0];
        pid->error[2] = pid->error[0];
        pid->fdb_prev = pid->fdb;
        pid->initialized = 1;
    }

    ///-------------------------- 位置式与增量式 PID 的核心计算逻辑 --------------------------------------/
    if (pid->mode == PID_POSITION)
    {
        /* ---- 位置式: out = P + I + D, 直接是最终控制量 ---- */

        /* 3. 比例项: Kp × 当前误差 */
        pid->Pout = pid->Kp * pid->error[0];


        /* 4. 积分项: 累加 Ki × 误差
         *    积分分离: |误差| 超过阈值时暂停累加, 避免大误差期间积分过度累积 */
        if ((pid->sep_threshold <= 0.0f) || // 积分分离开关，填写 0 禁用
            (pid->error[0] <= pid->sep_threshold && //将积分时机控制在误差小的时候，避免大误差时积分过度累积
             pid->error[0] >= -pid->sep_threshold))
        {
            pid->Iout += pid->Ki * pid->error[0];
        }
        /* 积分限幅: 抗积分饱和(积分项永远在 ±max_iout 内) */
        pid->Iout = pid_limit(pid->Iout, pid->max_iout);


        /* 5. 微分项
         *    默认: 作用在误差变化上(与原版一致)
         *    可选: 作用在反馈变化上 —— 设定值阶跃时误差突变不会进入微分,
         *          从而消除"微分突变(derivative kick)" */
        if (pid->d_on_measurement)
            Dout_raw = -pid->Kd * (pid->fdb - pid->fdb_prev); /* 反馈减小→输出为正   只看pid的反馈值 */
        else
            Dout_raw = pid->Kd * (pid->error[0] - pid->error[1]); //只看传感器返回的误差变化

        /* 6. 微分一阶低通滤波: y[n] = α·y[n-1] + (1-α)·x[n]
         *    α=0 时等价于不滤波; α 越大微分越"钝", 抗噪越强 */
        pid->Dout = pid->d_filter * pid->Dprev +
                    (1.0f - pid->d_filter) * Dout_raw;
        pid->Dprev = pid->Dout;

        /* 7. 合并输出并限幅 */
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = pid_limit(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        /* ---- 增量式: 每项都是增量, out 内部累加 ---- */

        /* 8. 比例增量: Kp × (e[n] - e[n-1]) */
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);

        /* 9. 积分增量: Ki × e[n](大误差时同样支持积分分离) */
        if ((pid->sep_threshold <= 0.0f) ||
            (pid->error[0] <= pid->sep_threshold &&
             pid->error[0] >= -pid->sep_threshold))
        {
            pid->Iout = pid->Ki * pid->error[0];
        }
        else
        {
            pid->Iout = 0.0f;
        }

        /* 10. 微分增量: Kd × (e[n] - 2e[n-1] + e[n-2]) 误差的二阶差分 */
        Dout_raw = pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        /* 11. 微分滤波(同位置式) */
        pid->Dout = pid->d_filter * pid->Dprev +
                    (1.0f - pid->d_filter) * Dout_raw;
        pid->Dprev = pid->Dout;

        /* 12. 增量叠加 + 输出限幅
         *     增量式没有独立的积分累加器, 天然不易积分饱和，所以没有积分限幅这个过程 */
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        pid->out = pid_limit(pid->out, pid->max_out);
    }

    /* 13. 保存本次反馈, 供下次"微分作用于反馈"使用 */
    pid->fdb_prev = pid->fdb;

    return pid->out;
}

/**
 * @brief  清空输出与历史状态(参数与配置保留, 不清除)
 * @note   急停后、切换目标/模式前调用, 防止旧历史干扰下一次控制
 *          一般就是在一次目标之后调用, 让 PID 重新用当前值填充历史
 *          清楚上一次目标的误差历史, 避免误差累积
 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
        return;

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->Dprev = 0.0f;
    pid->fdb_prev = 0.0f;
    pid->fdb = pid->set = 0.0f;
    pid->initialized = 0;   /* 下次 calc 会重新用当前值填充历史 */
}

/**
 * @brief  在线调参: 运行中修改 Kp/Ki/Kd(配合上位机/串口调试)
 */
void PID_set_param(pid_type_def *pid, pid_real_t Kp, pid_real_t Ki, pid_real_t Kd)
{
    if (pid == NULL)
        return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}


