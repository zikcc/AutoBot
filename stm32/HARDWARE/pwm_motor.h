/**
 * @file pwm_motor.h
 * @brief 高级PWM电机控制器（支持多通道/死区控制/紧急制动）
 * @author Your Name
 * @date 2024.01-2024.06
 * 
 * 特性：
 * - 多通道电机独立控制
 * - 硬件死区时间配置
 * - 紧急制动功能
 * - 软启动/软停止
 * - 电机状态监控
 * - 支持H桥和三相电机
 */

#ifndef __PWM_MOTOR_H
#define __PWM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"  // 根据实际使用的STM32系列调整

/**
 * @brief 电机通道枚举
 */
typedef enum {
    MOTOR_CHANNEL_LEFT = 0,     ///< 左轮电机
    MOTOR_CHANNEL_RIGHT = 1,    ///< 右轮电机
    MOTOR_CHANNEL_REAR = 2,     ///< 后轮电机（可选）
    MOTOR_CHANNEL_MAX           ///< 最大通道数
} Motor_Channel;

/**
 * @brief 电机工作模式
 */
typedef enum {
    MOTOR_MODE_FORWARD = 0,     ///< 正转模式
    MOTOR_MODE_REVERSE = 1,     ///< 反转模式
    MOTOR_MODE_BRAKE = 2,       ///< 制动模式
    MOTOR_MODE_COAST = 3        ///< 滑行模式（自由停止）
} Motor_Mode;

/**
 * @brief 电机状态结构体
 */
typedef struct {
    uint32_t pwm_value;         ///< 当前PWM占空比（0-1000）
    int32_t actual_rpm;         ///< 实际转速（RPM）
    int32_t target_rpm;         ///< 目标转速（RPM）
    Motor_Mode mode;            ///< 当前工作模式
    bool enabled;               ///< 使能状态
    uint32_t error_count;       ///< 错误计数
    uint32_t run_time;          ///< 运行时间（毫秒）
} Motor_Status;

/**
 * @brief PWM配置结构体
 */
typedef struct {
    TIM_TypeDef* timer;         ///< 定时器实例
    uint32_t channel;           ///< 定时器通道
    uint32_t frequency;         ///< PWM频率（Hz）
    uint32_t dead_time;         ///< 死区时间（纳秒）
    uint32_t max_duty;          ///< 最大占空比限制
    uint32_t min_duty;          ///< 最小占空比限制
} PWM_Config;

/**
 * @brief 电机控制器结构体
 */
typedef struct {
    PWM_Config pwm_config;      ///< PWM硬件配置
    Motor_Status status;        ///< 电机状态
    GPIO_TypeDef* enable_port;  ///< 使能引脚端口
    uint16_t enable_pin;       ///< 使能引脚
    GPIO_TypeDef* mode1_port;   ///< 模式控制引脚1端口
    uint16_t mode1_pin;         ///< 模式控制引脚1
    GPIO_TypeDef* mode2_port;   ///< 模式控制引脚2端口
    uint16_t mode2_pin;         ///< 模式控制引脚2
} Motor_Controller;

/* 函数声明 */

/**
 * @brief 初始化PWM电机控制器
 * @param controllers 电机控制器数组
 * @param num_motors 电机数量
 */
void PWM_Motor_Init(Motor_Controller* controllers, uint8_t num_motors);

/**
 * @brief 设置单个电机PWM输出
 * @param channel 电机通道
 * @param pwm_value PWM值（0-1000，对应0%-100%）
 * @param mode 电机工作模式
 */
void PWM_Motor_SetOutput(Motor_Channel channel, uint32_t pwm_value, Motor_Mode mode);

/**
 * @brief 设置电机转速（RPM）
 * @param channel 电机通道
 * @param rpm 目标转速（正负表示方向）
 */
void PWM_Motor_SetRPM(Motor_Channel channel, int32_t rpm);

/**
 * @brief 紧急制动（所有电机）
 */
void PWM_Motor_EmergencyBrake(void);

/**
 * @brief 软启动（平滑加速到目标速度）
 * @param channel 电机通道
 * @param target_rpm 目标转速
 * @param ramp_time 加速时间（毫秒）
 */
void PWM_Motor_SoftStart(Motor_Channel channel, int32_t target_rpm, uint32_t ramp_time);

/**
 * @brief 软停止（平滑减速停止）
 * @param channel 电机通道
 * @param ramp_time 减速时间（毫秒）
 */
void PWM_Motor_SoftStop(Motor_Channel channel, uint32_t ramp_time);

/**
 * @brief 使能/禁用电机
 * @param channel 电机通道
 * @param enable true=使能, false=禁用
 */
void PWM_Motor_Enable(Motor_Channel channel, bool enable);

/**
 * @brief 获取电机状态
 * @param channel 电机通道
 * @return Motor_Status 电机状态结构体
 */
Motor_Status PWM_Motor_GetStatus(Motor_Channel channel);

/**
 * @brief 更新电机实际转速（用于闭环控制）
 * @param channel 电机通道
 * @param actual_rpm 实际转速（从编码器获取）
 */
void PWM_Motor_UpdateFeedback(Motor_Channel channel, int32_t actual_rpm);

/**
 * @brief 设置死区时间
 * @param channel 电机通道
 * @param dead_time_ns 死区时间（纳秒）
 */
void PWM_Motor_SetDeadTime(Motor_Channel channel, uint32_t dead_time_ns);

/**
 * @brief 安全检查（看门狗调用）
 * @return true=系统正常, false=存在故障
 */
bool PWM_Motor_SafetyCheck(void);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_MOTOR_H */