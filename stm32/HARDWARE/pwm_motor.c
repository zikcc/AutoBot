/**
 * @file pwm_motor.c
 * @brief 高级PWM电机控制器实现
 */

#include "pwm_motor.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <math.h>

// 模块全局变量
static Motor_Controller* motor_controllers = NULL;
static uint8_t num_motors = 0;
static volatile uint32_t system_time = 0;

// 默认配置
#define DEFAULT_PWM_FREQUENCY   20000   // 20kHz PWM频率
#define DEFAULT_DEAD_TIME       100     // 100ns死区时间
#define DEFAULT_MAX_DUTY        950     // 最大95%占空比（防止过载）
#define DEFAULT_MIN_DUTY        50      // 最小5%占空比（维持运转）

void PWM_Motor_Init(Motor_Controller* controllers, uint8_t num_motors_count)
{
    motor_controllers = controllers;
    num_motors = num_motors_count;
    
    // 初始化每个电机控制器
    for (uint8_t i = 0; i < num_motors; i++) {
        Motor_Controller* mc = &motor_controllers[i];
        
        // 设置默认PWM配置
        mc->pwm_config.frequency = DEFAULT_PWM_FREQUENCY;
        mc->pwm_config.dead_time = DEFAULT_DEAD_TIME;
        mc->pwm_config.max_duty = DEFAULT_MAX_DUTY;
        mc->pwm_config.min_duty = DEFAULT_MIN_DUTY;
        
        // 初始化电机状态
        mc->status.pwm_value = 0;
        mc->status.actual_rpm = 0;
        mc->status.target_rpm = 0;
        mc->status.mode = MOTOR_MODE_COAST;
        mc->status.enabled = false;
        mc->status.error_count = 0;
        mc->status.run_time = 0;
        
        // 初始化硬件（根据实际硬件配置修改）
        PWM_Hardware_Init(mc);
    }
}

// 硬件初始化函数（需要根据实际硬件修改）
static void PWM_Hardware_Init(Motor_Controller* mc)
{
    // 示例：初始化TIM1 PWM输出
    if (mc->pwm_config.timer == TIM1) {
        // 使能时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
        
        // 定时器配置
        TIM_TimeBaseInitTypeDef tim_init;
        tim_init.TIM_Prescaler = SystemCoreClock / (mc->pwm_config.frequency * 1000) - 1;
        tim_init.TIM_CounterMode = TIM_CounterMode_Up;
        tim_init.TIM_Period = 1000 - 1; // 10位分辨率
        tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
        tim_init.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM1, &tim_init);
        
        // PWM输出配置
        TIM_OCInitTypeDef oc_init;
        oc_init.TIM_OCMode = TIM_OCMode_PWM1;
        oc_init.TIM_OutputState = TIM_OutputState_Enable;
        oc_init.TIM_OutputNState = TIM_OutputNState_Enable;
        oc_init.TIM_Pulse = 0; // 初始占空比0
        oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
        oc_init.TIM_OCNPolarity = TIM_OCNPolarity_High;
        oc_init.TIM_OCIdleState = TIM_OCIdleState_Set;
        oc_init.TIM_OCNIdleState = TIM_OCNIdleState_Set;
        
        // 配置通道
        switch (mc->pwm_config.channel) {
            case TIM_Channel_1:
                TIM_OC1Init(TIM1, &oc_init);
                TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
                break;
            case TIM_Channel_2:
                TIM_OC2Init(TIM1, &oc_init);
                TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
                break;
            case TIM_Channel_3:
                TIM_OC3Init(TIM1, &oc_init);
                TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
                break;
            case TIM_Channel_4:
                TIM_OC4Init(TIM1, &oc_init);
                TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
                break;
        }
        
        // 配置死区时间（防止H桥直通）
        TIM_BDTRInitTypeDef bdtr_init;
        bdtr_init.TIM_OSSRState = TIM_OSSRState_Enable;
        bdtr_init.TIM_OSSIState = TIM_OSSIState_Enable;
        bdtr_init.TIM_LOCKLevel = TIM_LOCKLevel_1;
        bdtr_init.TIM_DeadTime = mc->pwm_config.dead_time; // 死区时间
        bdtr_init.TIM_Break = TIM_Break_Disable;
        bdtr_init.TIM_BreakPolarity = TIM_BreakPolarity_High;
        bdtr_init.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
        TIM_BDTRConfig(TIM1, &bdtr_init);
        
        // 使能定时器
        TIM_Cmd(TIM1, ENABLE);
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
    }
    
    // 初始化GPIO控制引脚（使能、模式等）
    if (mc->enable_port) {
        GPIO_InitTypeDef gpio_init;
        gpio_init.GPIO_Pin = mc->enable_pin;
        gpio_init.GPIO_Mode = GPIO_Mode_OUT;
        gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
        gpio_init.GPIO_OType = GPIO_OType_PP;
        gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(mc->enable_port, &gpio_init);
        GPIO_ResetBits(mc->enable_port, mc->enable_pin); // 初始禁用
    }
}

void PWM_Motor_SetOutput(Motor_Channel channel, uint32_t pwm_value, Motor_Mode mode)
{
    if (channel >= num_motors || motor_controllers == NULL) {
        return;
    }
    
    Motor_Controller* mc = &motor_controllers[channel];
    
    // 安全检查：电机未使能时忽略设置
    if (!mc->status.enabled) {
        return;
    }
    
    // 占空比限幅
    if (pwm_value > mc->pwm_config.max_duty) {
        pwm_value = mc->pwm_config.max_duty;
        mc->status.error_count++;
    } else if (pwm_value < mc->pwm_config.min_duty) {
        pwm_value = mc->pwm_config.min_duty;
    }
    
    // 设置电机模式
    Set_Motor_Mode(mc, mode);
    
    // 更新PWM输出
    Set_PWM_Output(mc, pwm_value);
    
    // 更新状态
    mc->status.pwm_value = pwm_value;
    mc->status.mode = mode;
    mc->status.run_time++;
}

void PWM_Motor_SetRPM(Motor_Channel channel, int32_t rpm)
{
    if (channel >= num_motors) {
        return;
    }
    
    Motor_Controller* mc = &motor_controllers[channel];
    mc->status.target_rpm = rpm;
    
    // 根据转速方向设置模式
    Motor_Mode mode = (rpm >= 0) ? MOTOR_MODE_FORWARD : MOTOR_MODE_REVERSE;
    
    // 转换为PWM值（这里需要根据电机特性校准）
    uint32_t pwm_value = (uint32_t)fabsf(rpm) * 1000 / 3000; // 假设最大转速3000RPM
    
    PWM_Motor_SetOutput(channel, pwm_value, mode);
}

void PWM_Motor_EmergencyBrake(void)
{
    for (uint8_t i = 0; i < num_motors; i++) {
        // 快速设置制动模式
        Motor_Controller* mc = &motor_controllers[i];
        Set_Motor_Mode(mc, MOTOR_MODE_BRAKE);
        Set_PWM_Output(mc, 0); // 停止PWM输出
        
        // 更新状态
        mc->status.pwm_value = 0;
        mc->status.mode = MOTOR_MODE_BRAKE;
        mc->status.target_rpm = 0;
    }
}

// 设置电机模式（控制H桥方向）
static void Set_Motor_Mode(Motor_Controller* mc, Motor_Mode mode)
{
    if (!mc->mode1_port || !mc->mode2_port) {
        return;
    }
    
    switch (mode) {
        case MOTOR_MODE_FORWARD:
            GPIO_SetBits(mc->mode1_port, mc->mode1_pin);
            GPIO_ResetBits(mc->mode2_port, mc->mode2_pin);
            break;
            
        case MOTOR_MODE_REVERSE:
            GPIO_ResetBits(mc->mode1_port, mc->mode1_pin);
            GPIO_SetBits(mc->mode2_port, mc->mode2_pin);
            break;
            
        case MOTOR_MODE_BRAKE:
            GPIO_SetBits(mc->mode1_port, mc->mode1_pin);
            GPIO_SetBits(mc->mode2_port, mc->mode2_pin);
            break;
            
        case MOTOR_MODE_COAST:
            GPIO_ResetBits(mc->mode1_port, mc->mode1_pin);
            GPIO_ResetBits(mc->mode2_port, mc->mode2_pin);
            break;
    }
}

// 设置PWM输出
static void Set_PWM_Output(Motor_Controller* mc, uint32_t pwm_value)
{
    if (mc->pwm_config.timer == NULL) {
        return;
    }
    
    // 根据定时器通道设置比较值
    switch (mc->pwm_config.channel) {
        case TIM_Channel_1:
            TIM1->CCR1 = pwm_value;
            break;
        case TIM_Channel_2:
            TIM1->CCR2 = pwm_value;
            break;
        case TIM_Channel_3:
            TIM1->CCR3 = pwm_value;
            break;
        case TIM_Channel_4:
            TIM1->CCR4 = pwm_value;
            break;
    }
}

void PWM_Motor_Enable(Motor_Channel channel, bool enable)
{
    if (channel >= num_motors || motor_controllers == NULL) {
        return;
    }
    
    Motor_Controller* mc = &motor_controllers[channel];
    
    if (mc->enable_port) {
        if (enable) {
            GPIO_SetBits(mc->enable_port, mc->enable_pin);
        } else {
            GPIO_ResetBits(mc->enable_port, mc->enable_pin);
        }
        mc->status.enabled = enable;
    }
}

Motor_Status PWM_Motor_GetStatus(Motor_Channel channel)
{
    if (channel < num_motors && motor_controllers != NULL) {
        return motor_controllers[channel].status;
    }
    
    Motor_Status invalid_status = {0};
    return invalid_status;
}

bool PWM_Motor_SafetyCheck(void)
{
    if (motor_controllers == NULL) {
        return false;
    }
    
    for (uint8_t i = 0; i < num_motors; i++) {
        Motor_Controller* mc = &motor_controllers[i];
        
        // 检查错误计数
        if (mc->status.error_count > 100) {
            return false;
        }
        
        // 检查运行时间（防止过热）
        if (mc->status.run_time > 3600000) { // 1小时
            return false;
        }
    }
    
    return true;
}