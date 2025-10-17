/**
 * @file mpu6050.c
 * @brief MPU6050 6轴陀螺仪加速度计驱动实现
 */

#include "mpu6050.h"

// 灵敏度系数（根据量程确定）
static const float ACCEL_SENSITIVITY[] = {
    16384.0f,  // ±2g
    8192.0f,   // ±4g  
    4096.0f,   // ±8g
    2048.0f    // ±16g
};

static const float GYRO_SENSITIVITY[] = {
    131.0f,    // ±250°/s
    65.5f,     // ±500°/s
    32.8f,     // ±1000°/s
    16.4f      // ±2000°/s
};

/**
 * @brief I2C写寄存器函数
 */
static uint8_t MPU6050_WriteReg(MPU6050_Handle_t* hmpu, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(hmpu->hi2c, hmpu->i2c_addr, reg, 
                            I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief I2C读寄存器函数  
 */
static uint8_t MPU6050_ReadReg(MPU6050_Handle_t* hmpu, uint8_t reg, uint8_t* data, uint16_t size) {
    return HAL_I2C_Mem_Read(hmpu->hi2c, hmpu->i2c_addr, reg,
                           I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

uint8_t MPU6050_Init(MPU6050_Handle_t* hmpu, I2C_HandleTypeDef* hi2c) {
    uint8_t status = HAL_OK;
    
    // 初始化句柄
    hmpu->hi2c = hi2c;
    hmpu->i2c_addr = MPU6050_I2C_ADDR;
    hmpu->accel_fs = ACCEL_FS_2G;
    hmpu->gyro_fs = GYRO_FS_250DPS;
    hmpu->dlpf_bw = DLPF_44HZ;
    
    // 清零校准参数
    memset(&hmpu->calib, 0, sizeof(MPU6050_Calibration_t));
    
    // 检查设备是否连接
    if (!MPU6050_IsConnected(hmpu)) {
        return HAL_ERROR;
    }
    
    // 重置设备
    status |= MPU6050_Reset(hmpu);
    HAL_Delay(100);
    
    // 唤醒设备（退出睡眠模式）
    status |= MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(10);
    
    // 设置采样率分频器（1kHz采样率）
    status |= MPU6050_WriteReg(hmpu, MPU6050_REG_SMPLRT_DIV, 0x00);
    
    // 设置数字低通滤波器
    status |= MPU6050_SetDLPF(hmpu, hmpu->dlpf_bw);
    
    // 设置加速度计量程
    status |= MPU6050_SetAccelRange(hmpu, hmpu->accel_fs);
    
    // 设置陀螺仪量程
    status |= MPU6050_SetGyroRange(hmpu, hmpu->gyro_fs);
    
    // 禁用FIFO
    status |= MPU6050_WriteReg(hmpu, MPU6050_REG_FIFO_EN, 0x00);
    
    // 禁用所有中断
    status |= MPU6050_WriteReg(hmpu, MPU6050_REG_INT_ENABLE, 0x00);
    
    return status;
}

uint8_t MPU6050_ReadRawData(MPU6050_Handle_t* hmpu, MPU6050_RawData_t* raw_data) {
    uint8_t buffer[14];
    uint8_t status;
    
    // 一次性读取14个字节的传感器数据
    status = MPU6050_ReadReg(hmpu, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    
    if (status == HAL_OK) {
        // 解析加速度计数据
        raw_data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        raw_data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);  
        raw_data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
        
        // 解析温度数据
        raw_data->temp = (int16_t)((buffer[6] << 8) | buffer[7]);
        
        // 解析陀螺仪数据
        raw_data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
        raw_data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
        raw_data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
        
        // 应用校准偏移
        raw_data->accel_x -= hmpu->calib.accel_offset_x;
        raw_data->accel_y -= hmpu->calib.accel_offset_y;
        raw_data->accel_z -= hmpu->calib.accel_offset_z;
        raw_data->gyro_x -= hmpu->calib.gyro_offset_x;
        raw_data->gyro_y -= hmpu->calib.gyro_offset_y;
        raw_data->gyro_z -= hmpu->calib.gyro_offset_z;
    }
    
    return status;
}

uint8_t MPU6050_ReadData(MPU6050_Handle_t* hmpu, MPU6050_Data_t* data) {
    MPU6050_RawData_t raw_data;
    uint8_t status;
    
    status = MPU6050_ReadRawData(hmpu, &raw_data);
    
    if (status == HAL_OK) {
        // 加速度计数据转换（g单位）
        data->accel_x = (float)raw_data.accel_x / ACCEL_SENSITIVITY[hmpu->accel_fs];
        data->accel_y = (float)raw_data.accel_y / ACCEL_SENSITIVITY[hmpu->accel_fs];
        data->accel_z = (float)raw_data.accel_z / ACCEL_SENSITIVITY[hmpu->accel_fs];
        
        // 温度转换（°C单位）
        data->temp = (float)raw_data.temp / 340.0f + 36.53f;
        
        // 陀螺仪数据转换（°/s单位）
        data->gyro_x = (float)raw_data.gyro_x / GYRO_SENSITIVITY[hmpu->gyro_fs];
        data->gyro_y = (float)raw_data.gyro_y / GYRO_SENSITIVITY[hmpu->gyro_fs]; 
        data->gyro_z = (float)raw_data.gyro_z / GYRO_SENSITIVITY[hmpu->gyro_fs];
        
        // 计算姿态角
        MPU6050_CalculateAttitude(data);
    }
    
    return status;
}

uint8_t MPU6050_AutoCalibrate(MPU6050_Handle_t* hmpu, uint16_t samples) {
    MPU6050_RawData_t raw_data;
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    uint16_t i;
    
    if (samples == 0) samples = 100;
    
    // 采集样本数据
    for (i = 0; i < samples; i++) {
        if (MPU6050_ReadRawData(hmpu, &raw_data) != HAL_OK) {
            return HAL_ERROR;
        }
        
        accel_sum_x += raw_data.accel_x;
        accel_sum_y += raw_data.accel_y;
        accel_sum_z += raw_data.accel_z;
        gyro_sum_x += raw_data.gyro_x;
        gyro_sum_y += raw_data.gyro_y; 
        gyro_sum_z += raw_data.gyro_z;
        
        HAL_Delay(10);
    }
    
    // 计算平均值作为偏移量
    hmpu->calib.accel_offset_x = (int16_t)(accel_sum_x / samples);
    hmpu->calib.accel_offset_y = (int16_t)(accel_sum_y / samples);
    hmpu->calib.accel_offset_z = (int16_t)(accel_sum_z / samples) - ACCEL_SENSITIVITY[hmpu->accel_fs]; // 减去1g
    
    hmpu->calib.gyro_offset_x = (int16_t)(gyro_sum_x / samples);
    hmpu->calib.gyro_offset_y = (int16_t)(gyro_sum_y / samples);
    hmpu->calib.gyro_offset_z = (int16_t)(gyro_sum_z / samples);
    
    return HAL_OK;
}

uint8_t MPU6050_SetAccelRange(MPU6050_Handle_t* hmpu, Accel_FS_t fs) {
    hmpu->accel_fs = fs;
    return MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG, fs << 3);
}

uint8_t MPU6050_SetGyroRange(MPU6050_Handle_t* hmpu, Gyro_FS_t fs) {
    hmpu->gyro_fs = fs;
    return MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG, fs << 3);
}

uint8_t MPU6050_SetDLPF(MPU6050_Handle_t* hmpu, DLPF_BW_t bw) {
    hmpu->dlpf_bw = bw;
    return MPU6050_WriteReg(hmpu, MPU6050_REG_CONFIG, bw);
}

void MPU6050_CalculateAttitude(MPU6050_Data_t* data) {
    // 使用加速度计数据计算俯仰角和横滚角
    // 注意：这种方法只在静态或低速运动时准确
    
    // 计算俯仰角（绕Y轴旋转）
    data->pitch = atan2f(-data->accel_x, 
                        sqrtf(data->accel_y * data->accel_y + data->accel_z * data->accel_z)) 
                 * 180.0f / M_PI;
    
    // 计算横滚角（绕X轴旋转）
    data->roll = atan2f(data->accel_y, data->accel_z) * 180.0f / M_PI;
}

bool MPU6050_SelfTest(MPU6050_Handle_t* hmpu) {
    uint8_t self_test[6];
    uint8_t who_am_i;
    
    // 读取WHO_AM_I寄存器
    if (MPU6050_ReadReg(hmpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1) != HAL_OK) {
        return false;
    }
    
    if (who_am_i != 0x68) {
        return false;
    }
    
    // 读取自检寄存器
    if (MPU6050_ReadReg(hmpu, MPU6050_REG_SELF_TEST_X, self_test, 6) != HAL_OK) {
        return false;
    }
    
    // 简单的自检判断（实际应用中需要更复杂的判断逻辑）
    for (int i = 0; i < 6; i++) {
        if (self_test[i] == 0x00 || self_test[i] == 0xFF) {
            return false; // 异常值
        }
    }
    
    return true;
}

uint8_t MPU6050_Reset(MPU6050_Handle_t* hmpu) {
    return MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1, 0x80);
}

bool MPU6050_IsConnected(MPU6050_Handle_t* hmpu) {
    uint8_t who_am_i;
    
    if (MPU6050_ReadReg(hmpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1) != HAL_OK) {
        return false;
    }
    
    return (who_am_i == 0x68);
}