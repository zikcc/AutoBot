/**
 * @file encoder.c
 * @brief 高级正交编码器接口实现
 */

#include "encoder.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <math.h>

// 模块全局变量
static Encoder_Controller* encoder_controllers = NULL;
static uint8_t num_encoders = 0;
static volatile uint32_t system_time = 0;

// 默认配置
#define DEFAULT_COUNTS_PER_REV  2048    // 512 PPR × 4（正交编码模式）
#define DEFAULT_FILTER_VALUE    6       // 输入滤波值
#define DEFAULT_MEASURE_INTERVAL 10     // 速度测量间隔10ms

void Encoder_Init(Encoder_Controller* controllers, uint8_t num_encoders_count)
{
    encoder_controllers = controllers;
    num_encoders = num_encoders_count;
    
    for (uint8_t i = 0; i < num_encoders; i++) {
        Encoder_Controller* ec = &encoder_controllers[i];
        
        // 设置默认配置
        ec->config.counts_per_rev = DEFAULT_COUNTS_PER_REV;
        ec->config.filter_value = DEFAULT_FILTER_VALUE;
        ec->config.measurement_interval_ms = DEFAULT_MEASURE_INTERVAL;
        
        // 初始化状态
        ec->status.raw_count = 0;
        ec->status.total_count = 0;
        ec->status.speed_rpm = 0;
        ec->status.speed_pps = 0;
        ec->status.direction = ENCODER_DIR_STOPPED;
        ec->status.last_update_time = 0;
        ec->status.error_count = 0;
        ec->status.is_calibrated = false;
        ec->status.zero_offset = 0;
        
        ec->capture_time = 0;
        ec->last_capture_count = 0;
        
        // 初始化硬件
        Encoder_Hardware_Init(ec);
    }
}

// 硬件初始化函数
static void Encoder_Hardware_Init(Encoder_Controller* ec)
{
    if (ec->config.timer == NULL) {
        return;
    }
    
    // 示例：初始化TIM2为编码器模式
    if (ec->config.timer == TIM2) {
        // 使能时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        
        // 编码器模式配置
        TIM_EncoderInterfaceConfig(TIM2, 
                                 TIM_EncoderMode_TI12, // 正交编码模式
                                 TIM_ICPolarity_Rising, 
                                 TIM_ICPolarity_Rising);
        
        // 输入滤波配置
        TIM_SetIC1Prescaler(TIM2, TIM_ICPSC_DIV1);
        TIM_SetIC2Prescaler(TIM2, TIM_ICPSC_DIV1);
        TIM_ICInitTypeDef ic_init;
        ic_init.TIM_ICFilter = ec->config.filter_value;
        TIM_ICInit(TIM2, &ic_init);
        
        // 32位计数器配置（防溢出）
        TIM_SetAutoreload(TIM2, 0xFFFFFFFF);
        TIM_SetCounter(TIM2, 0x7FFFFFFF); // 初始值设为中间值
        
        // 使能定时器
        TIM_Cmd(TIM2, ENABLE);
    }
    
    // 配置GPIO引脚（编码器A相、B相）
    // 根据实际硬件连接配置
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    
    if (ec->config.timer == TIM2) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // TIM2_CH1, TIM2_CH2
        GPIO_Init(GPIOA, &gpio_init);
        
        // 配置引脚复用
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    }
}

void Encoder_Update(Encoder_Channel channel)
{
    if (channel >= num_encoders || encoder_controllers == NULL) {
        return;
    }
    
    Encoder_Controller* ec = &encoder_controllers[channel];
    uint32_t current_time = system_time;
    
    // 读取原始计数值（有符号处理）
    int32_t current_count = (int32_t)TIM_GetCounter(ec->config.timer);
    
    // 处理计数器溢出（32位防溢出逻辑）
    static int32_t last_counts[ENCODER_CHANNEL_MAX] = {0};
    int32_t delta_count = current_count - last_counts[channel];
    
    // 处理定时器溢出（16位定时器）
    if (delta_count > 0x7FFF) {
        delta_count -= 0x10000; // 负向溢出
    } else if (delta_count < -0x7FFF) {
        delta_count += 0x10000; // 正向溢出
    }
    
    // 更新总位置计数（32位防溢出）
    ec->status.total_count += delta_count;
    ec->status.raw_count = current_count;
    last_counts[channel] = current_count;
    
    // 计算速度（定期更新）
    uint32_t time_diff = current_time - ec->capture_time;
    if (time_diff >= ec->config.measurement_interval_ms) {
        int32_t count_diff = ec->status.total_count - ec->last_capture_count;
        
        // 计算脉冲频率（脉冲/秒）
        ec->status.speed_pps = (count_diff * 1000) / time_diff;
        
        // 计算转速（RPM）
        if (ec->config.counts_per_rev > 0) {
            ec->status.speed_rpm = (ec->status.speed_pps * 60) / ec->config.counts_per_rev;
        }
        
        // 确定方向
        if (count_diff > 5) { // 死区阈值
            ec->status.direction = ENCODER_DIR_FORWARD;
        } else if (count_diff < -5) {
            ec->status.direction = ENCODER_DIR_REVERSE;
        } else {
            ec->status.direction = ENCODER_DIR_STOPPED;
        }
        
        // 更新捕获时间和计数
        ec->capture_time = current_time;
        ec->last_capture_count = ec->status.total_count;
    }
    
    ec->status.last_update_time = current_time;
}

int32_t Encoder_GetPosition(Encoder_Channel channel)
{
    if (channel < num_encoders && encoder_controllers != NULL) {
        return encoder_controllers[channel].status.total_count - 
               encoder_controllers[channel].status.zero_offset;
    }
    return 0;
}

int32_t Encoder_GetSpeedRPM(Encoder_Channel channel)
{
    if (channel < num_encoders && encoder_controllers != NULL) {
        return encoder_controllers[channel].status.speed_rpm;
    }
    return 0;
}

int32_t Encoder_GetSpeedPPS(Encoder_Channel channel)
{
    if (channel < num_encoders && encoder_controllers != NULL) {
        return encoder_controllers[channel].status.speed_pps;
    }
    return 0;
}

Encoder_Direction Encoder_GetDirection(Encoder_Channel channel)
{
    if (channel < num_encoders && encoder_controllers != NULL) {
        return encoder_controllers[channel].status.direction;
    }
    return ENCODER_DIR_STOPPED;
}

void Encoder_SetPosition(Encoder_Channel channel, int32_t position)
{
    if (channel >= num_encoders || encoder_controllers == NULL) {
        return;
    }
    
    Encoder_Controller* ec = &encoder_controllers[channel];
    ec->status.total_count = position + ec->status.zero_offset;
    ec->last_capture_count = ec->status.total_count;
    
    // 重置定时器计数器到中间值（防溢出）
    if (ec->config.timer) {
        TIM_SetCounter(ec->config.timer, 0x7FFFFFFF);
    }
}

void Encoder_CalibrateZero(Encoder_Channel channel)
{
    if (channel >= num_encoders || encoder_controllers == NULL) {
        return;
    }
    
    Encoder_Controller* ec = &encoder_controllers[channel];
    ec->status.zero_offset = ec->status.total_count;
    ec->status.is_calibrated = true;
}

Encoder_Status Encoder_GetStatus(Encoder_Channel channel)
{
    if (channel < num_encoders && encoder_controllers != NULL) {
        return encoder_controllers[channel].status;
    }
    
    Encoder_Status invalid_status = {0};
    return invalid_status;
}

bool Encoder_IsHealthy(Encoder_Channel channel)
{
    if (channel >= num_encoders || encoder_controllers == NULL) {
        return false;
    }
    
    Encoder_Controller* ec = &encoder_controllers[channel];
    
    // 检查更新超时（编码器可能故障）
    uint32_t time_since_update = system_time - ec->status.last_update_time;
    if (time_since_update > 1000) { // 1秒未更新
        ec->status.error_count++;
        return false;
    }
    
    // 检查错误计数
    if (ec->status.error_count > 50) {
        return false;
    }
    
    return true;
}

int32_t Encoder_GetTotalDistance(void)
{
    if (encoder_controllers == NULL) {
        return 0;
    }
    
    int32_t total_distance = 0;
    for (uint8_t i = 0; i < num_encoders; i++) {
        total_distance += abs(encoder_controllers[i].status.total_count);
    }
    
    return total_distance;
}

// 编码器中断处理函数（示例）
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        // 处理编码器溢出中断
        // 可以在这里更新位置计数
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}