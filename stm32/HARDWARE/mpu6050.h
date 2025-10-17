/**
 * @file mpu6050.h
 * @brief MPU6050 6轴陀螺仪加速度计驱动
 * @author Your Name
 * @date 2024.01-2024.06
 * 
 * 特性：
 * - I2C接口通信
 * - 6轴数据读取（3轴加速度 + 3轴陀螺仪 + 温度）
 * - 传感器自检功能
 * - 数据校准和滤波
 * - 姿态角计算（俯仰角、横滚角）
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* MPU6050 I2C地址定义 */
#define MPU6050_I2C_ADDR        0x68 << 1    // AD0引脚接地时的地址
#define MPU6050_I2C_ADDR_ALT    0x69 << 1    // AD0引脚接VCC时的地址

/* MPU6050寄存器地址定义 */
#define MPU6050_REG_SELF_TEST_X  0x0D
#define MPU6050_REG_SELF_TEST_Y  0x0E  
#define MPU6050_REG_SELF_TEST_Z  0x0F
#define MPU6050_REG_SELF_TEST_A  0x10
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_FIFO_EN      0x23
#define MPU6050_REG_I2C_MST_CTRL 0x24
#define MPU6050_REG_I2C_SLV0_ADDR 0x25
#define MPU6050_REG_I2C_SLV0_REG 0x26
#define MPU6050_REG_I2C_SLV0_CTRL 0x27
#define MPU6050_REG_I2C_MST_STATUS 0x36
#define MPU6050_REG_INT_PIN_CFG  0x37
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_INT_STATUS   0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_TEMP_OUT_L   0x42
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_GYRO_XOUT_L  0x44
#define MPU6050_REG_GYRO_YOUT_H  0x45
#define MPU6050_REG_GYRO_YOUT_L  0x46
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_GYRO_ZOUT_L  0x48
#define MPU6050_REG_I2C_SLV0_DO  0x63
#define MPU6050_REG_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_USER_CTRL    0x6A
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_PWR_MGMT_2   0x6C
#define MPU6050_REG_FIFO_COUNTH  0x72
#define MPU6050_REG_FIFO_COUNTL  0x73
#define MPU6050_REG_FIFO_R_W     0x74
#define MPU6050_REG_WHO_AM_I     0x75

/* 量程定义 */
typedef enum {
    ACCEL_FS_2G = 0,   // ±2g
    ACCEL_FS_4G = 1,   // ±4g  
    ACCEL_FS_8G = 2,   // ±8g
    ACCEL_FS_16G = 3   // ±16g
} Accel_FS_t;

typedef enum {
    GYRO_FS_250DPS = 0,  // ±250°/s
    GYRO_FS_500DPS = 1,  // ±500°/s
    GYRO_FS_1000DPS = 2, // ±1000°/s  
    GYRO_FS_2000DPS = 3  // ±2000°/s
} Gyro_FS_t;

/* 数字低通滤波器带宽 */
typedef enum {
    DLPF_260HZ = 0,  // 加速度计260Hz，陀螺仪256Hz
    DLPF_184HZ = 1,  // 加速度计184Hz，陀螺仪188Hz
    DLPF_94HZ = 2,   // 加速度计94Hz，陀螺仪98Hz
    DLPF_44HZ = 3,   // 加速度计44Hz，陀螺仪42Hz
    DLPF_21HZ = 4,   // 加速度计21Hz，陀螺仪20Hz
    DLPF_10HZ = 5,   // 加速度计10Hz，陀螺仪10Hz
    DLPF_5HZ = 6     // 加速度计5Hz，陀螺仪5Hz
} DLPF_BW_t;

/**
 * @brief MPU6050原始数据结构体
 */
typedef struct {
    int16_t accel_x;    // 加速度计X轴原始数据
    int16_t accel_y;    // 加速度计Y轴原始数据  
    int16_t accel_z;    // 加速度计Z轴原始数据
    int16_t temp;       // 温度原始数据
    int16_t gyro_x;     // 陀螺仪X轴原始数据
    int16_t gyro_y;     // 陀螺仪Y轴原始数据
    int16_t gyro_z;     // 陀螺仪Z轴原始数据
} MPU6050_RawData_t;

/**
 * @brief MPU6050校准数据结构体
 */
typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y; 
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
} MPU6050_Calibration_t;

/**
 * @brief MPU6050处理后的数据结构体
 */
typedef struct {
    float accel_x;      // 加速度X (g)
    float accel_y;      // 加速度Y (g)
    float accel_z;      // 加速度Z (g)
    float temp;         // 温度 (°C)
    float gyro_x;       // 角速度X (°/s)
    float gyro_y;       // 角速度Y (°/s) 
    float gyro_z;       // 角速度Z (°/s)
    float pitch;        // 俯仰角 (度)
    float roll;         // 横滚角 (度)
} MPU6050_Data_t;

/**
 * @brief MPU6050配置结构体
 */
typedef struct {
    I2C_HandleTypeDef* hi2c;        // I2C句柄
    uint8_t i2c_addr;               // I2C地址
    Accel_FS_t accel_fs;            // 加速度计量程
    Gyro_FS_t gyro_fs;              // 陀螺仪量程  
    DLPF_BW_t dlpf_bw;              // 低通滤波器带宽
    MPU6050_Calibration_t calib;    // 校准参数
} MPU6050_Handle_t;

/* 函数声明 */

/**
 * @brief 初始化MPU6050传感器
 * @param hmpu MPU6050句柄指针
 * @param hi2c I2C句柄
 * @return HAL状态
 */
uint8_t MPU6050_Init(MPU6050_Handle_t* hmpu, I2C_HandleTypeDef* hi2c);

/**
 * @brief 读取MPU6050原始数据
 * @param hmpu MPU6050句柄指针
 * @param raw_data 原始数据存储指针
 * @return HAL状态
 */
uint8_t MPU6050_ReadRawData(MPU6050_Handle_t* hmpu, MPU6050_RawData_t* raw_data);

/**
 * @brief 读取处理后的MPU6050数据（带单位转换）
 * @param hmpu MPU6050句柄指针  
 * @param data 处理后的数据存储指针
 * @return HAL状态
 */
uint8_t MPU6050_ReadData(MPU6050_Handle_t* hmpu, MPU6050_Data_t* data);

/**
 * @brief 自动校准MPU6050传感器
 * @param hmpu MPU6050句柄指针
 * @param samples 采样次数
 * @return HAL状态
 */
uint8_t MPU6050_AutoCalibrate(MPU6050_Handle_t* hmpu, uint16_t samples);

/**
 * @brief 设置加速度计量程
 * @param hmpu MPU6050句柄指针
 * @param fs 量程枚举
 * @return HAL状态
 */
uint8_t MPU6050_SetAccelRange(MPU6050_Handle_t* hmpu, Accel_FS_t fs);

/**
 * @brief 设置陀螺仪量程
 * @param hmpu MPU6050句柄指针
 * @param fs 量程枚举  
 * @return HAL状态
 */
uint8_t MPU6050_SetGyroRange(MPU6050_Handle_t* hmpu, Gyro_FS_t fs);

/**
 * @brief 设置数字低通滤波器带宽
 * @param hmpu MPU6050句柄指针
 * @param bw 带宽枚举
 * @return HAL状态
 */
uint8_t MPU6050_SetDLPF(MPU6050_Handle_t* hmpu, DLPF_BW_t bw);

/**
 * @brief 计算姿态角（俯仰角和横滚角）
 * @param data MPU6050数据指针
 */
void MPU6050_CalculateAttitude(MPU6050_Data_t* data);

/**
 * @brief 传感器自检
 * @param hmpu MPU6050句柄指针
 * @return true=自检通过, false=自检失败
 */
bool MPU6050_SelfTest(MPU6050_Handle_t* hmpu);

/**
 * @brief 重置MPU6050芯片
 * @param hmpu MPU6050句柄指针
 * @return HAL状态
 */
uint8_t MPU6050_Reset(MPU6050_Handle_t* hmpu);

/**
 * @brief 检查MPU6050是否连接
 * @param hmpu MPU6050句柄指针
 * @return true=连接正常, false=连接异常
 */
bool MPU6050_IsConnected(MPU6050_Handle_t* hmpu);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */