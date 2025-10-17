/**
 * @file motor_pid.h
 * @brief 高级PID电机控制器（支持积分限幅+微分先行）
 * @author Your Name
 * @date 2024.01-2024.06
 * 
 * 特性：
 * - 积分限幅抗饱和
 * - 微分先行减少设定值突变冲击
 * - 输出限幅保护
 * - 手动/自动模式
 * - 抗积分饱和机制
 */

#ifndef __MOTOR_PID_H
#define __MOTOR_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID控制器结构体
 */
typedef struct {
    /* 基本PID参数 */
    float Kp;               ///< 比例增益
    float Ki;               ///< 积分增益  
    float Kd;               ///< 微分增益
    
    /* 高级控制参数 */
    float integral_max;     ///< 积分限幅值（抗饱和）
    float output_max;       ///< 输出最大值
    float output_min;       ///< 输出最小值
    
    /* 微分先行参数 */
    float derivative_alpha; ///< 微分滤波系数(0-1)
    bool derivative_on_measurement; ///< 微分先行使能
    
    /* 运行状态 */
    float integral;         ///< 积分项累加值
    float last_error;      ///< 上次误差（标准PID）
    float last_measurement; ///< 上次测量值（微分先行）
    float last_derivative;  ///< 上次微分值（滤波用）
    
    /* 模式控制 */
    bool auto_mode;        ///< 自动/手动模式
    float manual_output;   ///< 手动模式输出值
    
    /* 性能监控 */
    uint32_t update_count;  ///< 更新次数统计
    float cumulative_error; ///< 累计误差（用于性能评估）
} PID_Controller;

/**
 * @brief PID初始化函数
 * @param pid PID控制器指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd);

/**
 * @brief 高级PID初始化（完整参数）
 * @param pid PID控制器指针
 * @param Kp 比例增益
 * @param Ki 积分增益  
 * @param Kd 微分增益
 * @param integral_max 积分限幅
 * @param output_min 输出最小值
 * @param output_max 输出最大值
 * @param derivative_alpha 微分滤波系数
 * @param derivative_on_measurement 是否启用微分先行
 */
void PID_InitAdvanced(PID_Controller* pid, 
                     float Kp, float Ki, float Kd,
                     float integral_max,
                     float output_min, float output_max,
                     float derivative_alpha,
                     bool derivative_on_measurement);

/**
 * @brief PID控制器更新函数（核心算法）
 * @param pid PID控制器指针
 * @param setpoint 设定值（目标速度/位置）
 * @param measurement 测量值（当前速度/位置）
 * @param dt 时间步长（秒）
 * @return 控制器输出
 */
float PID_Update(PID_Controller* pid, float setpoint, float measurement, float dt);

/**
 * @brief 重置PID控制器状态
 * @param pid PID控制器指针
 */
void PID_Reset(PID_Controller* pid);

/**
 * @brief 设置PID参数
 * @param pid PID控制器指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void PID_SetTunings(PID_Controller* pid, float Kp, float Ki, float Kd);

/**
 * @brief 设置输出限幅
 * @param pid PID控制器指针
 * @param min 最小值
 * @param max 最大值
 */
void PID_SetOutputLimits(PID_Controller* pid, float min, float max);

/**
 * @brief 设置积分限幅
 * @param pid PID控制器指针
 * @param max 积分最大值
 */
void PID_SetIntegralLimit(PID_Controller* pid, float max);

/**
 * @brief 设置控制器模式
 * @param pid PID控制器指针
 * @param auto_mode true=自动, false=手动
 * @param manual_output 手动模式时的输出值
 */
void PID_SetMode(PID_Controller* pid, bool auto_mode, float manual_output);

/**
 * @brief 获取控制器状态信息
 * @param pid PID控制器指针
 * @param[out] p_term 比例项输出
 * @param[out] i_term 积分项输出
 * @param[out] d_term 微分项输出
 */
void PID_GetComponents(PID_Controller* pid, float* p_term, float* i_term, float* d_term);

/**
 * @brief 获取性能统计
 * @param pid PID控制器指针
 * @param[out] avg_error 平均误差
 * @param[out] update_count 更新次数
 */
void PID_GetStatistics(PID_Controller* pid, float* avg_error, uint32_t* update_count);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PID_H */