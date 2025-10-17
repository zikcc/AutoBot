/**
 * @file encoder.h
 * @brief 高级正交编码器接口（支持多通道/速度测量/位置跟踪）
 * @author Your Name
 * @date 2024.01-2024.06
 * 
 * 特性：
 * - 多通道正交编码器支持
 * - 32位位置计数器（防溢出）
 * - 实时速度测量（RPM和脉冲频率）
 * - 方向检测
 * - 零位校准
 * - 抗抖动滤波
 * - 溢出保护
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

/**
 * @brief 编码器通道枚举
 */
typedef enum {
    ENCODER_CHANNEL_LEFT = 0,    ///< 左轮编码器
    ENCODER_CHANNEL_RIGHT = 1,    ///< 右轮编码器
    ENCODER_CHANNEL_REAR = 2,     ///< 后轮编码器（可选）
    ENCODER_CHANNEL_MAX          ///< 最大通道数
} Encoder_Channel;

/**
 * @brief 编码器方向枚举
 */
typedef enum {
    ENCODER_DIR_FORWARD = 0,     ///< 正转方向
    ENCODER_DIR_REVERSE = 1,     ///< 反转方向
    ENCODER_DIR_STOPPED = 2      ///< 停止状态
} Encoder_Direction;

/**
 * @brief 编码器配置结构体
 */
typedef struct {
    TIM_TypeDef* timer;          ///< 定时器实例（编码器模式）
    uint32_t counts_per_rev;     ///< 每转脉冲数（PPR × 4）
    uint32_t filter_value;       ///< 输入滤波值
    uint32_t measurement_interval_ms; ///< 速度测量间隔（毫秒）
} Encoder_Config;

/**
 * @brief 编码器状态结构体
 */
typedef struct {
    int32_t raw_count;           ///< 原始计数值（有符号）
    int32_t total_count;         ///< 累计绝对位置（防溢出）
    int32_t speed_rpm;           ///< 当前转速（RPM）
    int32_t speed_pps;           ///< 当前速度（脉冲/秒）
    Encoder_Direction direction;  ///< 当前转动方向
    uint32_t last_update_time;   ///< 最后更新时间戳
    uint32_t error_count;        ///< 错误计数
    bool is_calibrated;          ///< 校准状态
    int32_t zero_offset;         ///< 零位偏移
} Encoder_Status;

/**
 * @brief 编码器控制器结构体
 */
typedef struct {
    Encoder_Config config;       ///< 硬件配置
    Encoder_Status status;       ///< 运行状态
    uint32_t capture_time;       ///< 捕获时间（用于速度计算）
    int32_t last_capture_count;  ///< 上次捕获计数值
} Encoder_Controller;

/* 函数声明 */

/**
 * @brief 初始化编码器系统
 * @param controllers 编码器控制器数组
 * @param num_encoders 编码器数量
 */
void Encoder_Init(Encoder_Controller* controllers, uint8_t num_encoders);

/**
 * @brief 更新编码器状态（定期调用）
 * @param channel 编码器通道
 */
void Encoder_Update(Encoder_Channel channel);

/**
 * @brief 获取编码器当前位置（32位防溢出）
 * @param channel 编码器通道
 * @return int32_t 绝对位置计数
 */
int32_t Encoder_GetPosition(Encoder_Channel channel);

/**
 * @brief 获取编码器当前转速（RPM）
 * @param channel 编码器通道
 * @return int32_t 转速（转/分钟）
 */
int32_t Encoder_GetSpeedRPM(Encoder_Channel channel);

/**
 * @brief 获取编码器当前速度（脉冲/秒）
 * @param channel 编码器通道
 * @return int32_t 速度（脉冲/秒）
 */
int32_t Encoder_GetSpeedPPS(Encoder_Channel channel);

/**
 * @brief 获取编码器转动方向
 * @param channel 编码器通道
 * @return Encoder_Direction 转动方向
 */
Encoder_Direction Encoder_GetDirection(Encoder_Channel channel);

/**
 * @brief 重置编码器位置计数器
 * @param channel 编码器通道
 * @param position 要设置的位置值
 */
void Encoder_SetPosition(Encoder_Channel channel, int32_t position);

/**
 * @brief 编码器零位校准
 * @param channel 编码器通道
 */
void Encoder_CalibrateZero(Encoder_Channel channel);

/**
 * @brief 获取编码器状态信息
 * @param channel 编码器通道
 * @return Encoder_Status 状态结构体
 */
Encoder_Status Encoder_GetStatus(Encoder_Channel channel);

/**
 * @brief 使能编码器中断
 * @param channel 编码器通道
 * @param enable 是否使能
 */
void Encoder_EnableInterrupt(Encoder_Channel channel, bool enable);

/**
 * @brief 检查编码器是否正常工作
 * @param channel 编码器通道
 * @return true=正常, false=故障
 */
bool Encoder_IsHealthy(Encoder_Channel channel);

/**
 * @brief 获取所有编码器总位移
 * @return int32_t 总位移（可用于里程计算）
 */
int32_t Encoder_GetTotalDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */