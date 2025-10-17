/**
 * @file motor_pid.c
 * @brief 高级PID电机控制器实现
 */

#include "motor_pid.h"
#include <math.h>

// 默认参数定义
#define DEFAULT_INTEGRAL_MAX   1000.0f
#define DEFAULT_OUTPUT_MIN     -1000.0f
#define DEFAULT_OUTPUT_MAX     1000.0f
#define DEFAULT_DERIVATIVE_ALPHA 0.1f

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd)
{
    // 基本PID参数
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // 默认高级参数
    pid->integral_max = DEFAULT_INTEGRAL_MAX;
    pid->output_min = DEFAULT_OUTPUT_MIN;
    pid->output_max = DEFAULT_OUTPUT_MAX;
    pid->derivative_alpha = DEFAULT_DERIVATIVE_ALPHA;
    pid->derivative_on_measurement = true; // 默认启用微分先行
    
    // 初始化状态变量
    PID_Reset(pid);
    
    // 模式设置
    pid->auto_mode = true;
    pid->manual_output = 0.0f;
}

void PID_InitAdvanced(PID_Controller* pid, 
                     float Kp, float Ki, float Kd,
                     float integral_max,
                     float output_min, float output_max,
                     float derivative_alpha,
                     bool derivative_on_measurement)
{
    // 基本PID参数
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    // 高级参数
    pid->integral_max = integral_max;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->derivative_alpha = derivative_alpha;
    pid->derivative_on_measurement = derivative_on_measurement;
    
    // 初始化状态变量
    PID_Reset(pid);
    
    // 模式设置
    pid->auto_mode = true;
    pid->manual_output = 0.0f;
}

void PID_Reset(PID_Controller* pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_measurement = 0.0f;
    pid->last_derivative = 0.0f;
    pid->update_count = 0;
    pid->cumulative_error = 0.0f;
}

float PID_Update(PID_Controller* pid, float setpoint, float measurement, float dt)
{
    // 检查时间步长有效性
    if (dt <= 0.0f) {
        return 0.0f;
    }
    
    // 手动模式直接返回预设输出
    if (!pid->auto_mode) {
        return pid->manual_output;
    }
    
    float error = setpoint - measurement;
    
    // 比例项
    float p_term = pid->Kp * error;
    
    // 积分项（带限幅抗饱和） - 项目重点！
    pid->integral += error * dt;
    
    // 积分限幅处理
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    float i_term = pid->Ki * pid->integral;
    
    // 微分项 - 项目重点：微分先行！
    float d_term = 0.0f;
    
    if (pid->derivative_on_measurement) {
        // 微分先行：对测量值微分，减少设定值突变冲击
        float derivative = (measurement - pid->last_measurement) / dt;
        
        // 微分滤波（一阶低通滤波）
        derivative = pid->derivative_alpha * derivative + 
                    (1.0f - pid->derivative_alpha) * pid->last_derivative;
        
        d_term = -pid->Kd * derivative; // 注意负号
        
        pid->last_measurement = measurement;
        pid->last_derivative = derivative;
    } else {
        // 标准微分：对误差微分
        float derivative = (error - pid->last_error) / dt;
        d_term = pid->Kd * derivative;
        pid->last_error = error;
    }
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
        // 抗积分饱和：输出饱和时停止积分累加
        if (error * output > 0) {
            pid->integral -= error * dt;
        }
    } else if (output < pid->output_min) {
        output = pid->output_min;
        // 抗积分饱和
        if (error * output > 0) {
            pid->integral -= error * dt;
        }
    }
    
    // 更新统计信息
    pid->update_count++;
    pid->cumulative_error += fabsf(error);
    
    return output;
}

void PID_SetTunings(PID_Controller* pid, float Kp, float Ki, float Kd)
{
    // 防止除零
    if (Kp < 0 || Ki < 0 || Kd < 0) {
        return;
    }
    
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetOutputLimits(PID_Controller* pid, float min, float max)
{
    if (min >= max) return;
    
    pid->output_min = min;
    pid->output_max = max;
}

void PID_SetIntegralLimit(PID_Controller* pid, float max)
{
    pid->integral_max = fabsf(max);
}

void PID_SetMode(PID_Controller* pid, bool auto_mode, float manual_output)
{
    if (pid->auto_mode != auto_mode) {
        // 模式切换时重置控制器状态
        PID_Reset(pid);
    }
    
    pid->auto_mode = auto_mode;
    pid->manual_output = manual_output;
}

void PID_GetComponents(PID_Controller* pid, float* p_term, float* i_term, float* d_term)
{
    // 这里需要根据最后一次计算的值返回，实际使用时需要修改实现
    if (p_term) *p_term = 0.0f;
    if (i_term) *i_term = 0.0f;
    if (d_term) *d_term = 0.0f;
}

void PID_GetStatistics(PID_Controller* pid, float* avg_error, uint32_t* update_count)
{
    if (avg_error) {
        *avg_error = (pid->update_count > 0) ? 
                    pid->cumulative_error / pid->update_count : 0.0f;
    }
    if (update_count) {
        *update_count = pid->update_count;
    }
}