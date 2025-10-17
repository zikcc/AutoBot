#include "mcp4017.h"

// GPIO控制函数
void I2C_SCL_High(void) { HAL_GPIO_WritePin(SCL_MCP_GPIO_Port, SCL_MCP_Pin, GPIO_PIN_SET); }
void I2C_SCL_Low(void) { HAL_GPIO_WritePin(SCL_MCP_GPIO_Port, SCL_MCP_Pin, GPIO_PIN_RESET); }
void I2C_SDA_High(void) { HAL_GPIO_WritePin(SDA_MCP_GPIO_Port, SDA_MCP_Pin, GPIO_PIN_SET); }
void I2C_SDA_Low(void) { HAL_GPIO_WritePin(SDA_MCP_GPIO_Port, SDA_MCP_Pin, GPIO_PIN_RESET); }
void I2C_SDA_OutputMode(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; GPIO_InitStruct.Pin = SDA_MCP_Pin; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_PULLUP; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(SDA_MCP_GPIO_Port, &GPIO_InitStruct); }
void I2C_SDA_InputMode(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; GPIO_InitStruct.Pin = SDA_MCP_Pin; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(SDA_MCP_GPIO_Port, &GPIO_InitStruct); }
uint8_t I2C_SDA_Read(void) { return HAL_GPIO_ReadPin(SDA_MCP_GPIO_Port, SDA_MCP_Pin); }

// I2C起始条件
void I2C_Start(void) {
	I2C_SDA_OutputMode();
    I2C_SDA_High();
    I2C_SCL_High();
    HAL_Delay(1); // 短暂延时
    I2C_SDA_Low();
    HAL_Delay(1);
    I2C_SCL_Low();
}

// I2C停止条件
void I2C_Stop(void) {
	I2C_SDA_OutputMode();
    I2C_SDA_Low();
    I2C_SCL_High();
    HAL_Delay(1);
    I2C_SDA_High();
    HAL_Delay(1);
	I2C_SCL_Low();
}

void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    I2C_SDA_OutputMode();
    I2C_SCL_Low();//拉低时钟开始数据传输

    for(t = 0; t < 8; t++)
    {
        if (txd & 0x80) {
            I2C_SDA_High();
        } else {
            I2C_SDA_Low();
        }		
        txd <<= 1;
        I2C_SCL_High();
        HAL_Delay(1);
        I2C_SCL_Low();
        HAL_Delay(1);
    }
}

// I2C等待ACK
uint8_t IIC_Wait_Ack(void)
{
    uint16_t ucErrTime = 0;
    I2C_SDA_InputMode();      //SDA设置为输入
    I2C_SDA_High();
    HAL_Delay(1);
    I2C_SCL_High();
    HAL_Delay(1);

    while(I2C_SDA_Read())
    {
        ucErrTime++;
		
        if(ucErrTime > 2000)
        {
            I2C_Stop();
            return 1;
        }
    }

    I2C_SCL_Low();//时钟输出0
    return 0;
}

// 设置 MCP4017 的电阻值
void MCP4017_SetResistance(uint8_t value) {
    I2C_Start();
    IIC_Send_Byte(0x5E); // 写地址
    if (IIC_Wait_Ack()) {
        I2C_Stop();
        return; // 没有收到ACK，通信失败
    }
    IIC_Send_Byte(value); // 发送电阻值
    IIC_Wait_Ack();
    I2C_Stop();
}




