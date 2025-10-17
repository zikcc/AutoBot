# AutoBot - 智能移动机器人平台

## 🖼️ 系统实物图

### AutoBot 智能移动机器人平台

![AutoBot机器人平台实物图](https://zikcc.oss-cn-beijing.aliyuncs.com/img/202510171107038.jpg)  

![AutoBot机器人平台实物图](https://zikcc.oss-cn-beijing.aliyuncs.com/img/202510171107038.jpg)

![AutoBot机器人平台实物图](https://zikcc.oss-cn-beijing.aliyuncs.com/img/202510171107999.jpg)

![语音模块图](https://zikcc.oss-cn-beijing.aliyuncs.com/img/202510171107603.jpg)

## 🚀 项目概述

AutoBot是一个基于**ROS + STM32**架构的智能移动机器人平台，具备**多模态感知、语音交互和自主导航**能力。本项目在实验室已有的机器人移动底盘基础上，进行了深度的软硬件功能扩展与系统集成，实现了从环境感知到运动控制的完整闭环系统。

## ✨ 核心特性

- **🔍 多模态感知**：深度视觉 + 离线语音交互 + IMU姿态感知
- **⚡ 实时控制**：STM32 1kHz高频电机控制，PID算法优化
- **📡 可靠通信**：自定义串口协议，CRC16校验 + 超时重传机制
- **🧩 模块化设计**：ROS + STM32分层架构，支持独立开发与测试
- **🗣️ 语音控制**：支持中英文离线语音指令，响应延迟 <200ms

## 🏗️ 项目结构

AAutoBot/
├── ros/                           # ROS工作空间
│   └── ros_ws/
│       └── src/
│           ├── bringup/           # 硬件接口与通信
│           │   ├── CMakeLists.txt
│           │   ├── launch/
│           │   │   └── autobot.launch
│           │   ├── package.xml
│           │   └── src/
│           │       ├── protocol.cpp
│           │       ├── protocol.hpp
│           │       └── serial_node.cpp
│           ├── vision/            # 视觉感知处理
│           │   ├── CMakeLists.txt
│           │   ├── config/
│           │   │   └── vision_params.yaml
│           │   ├── launch/
│           │   │   └── vision.launch
│           │   ├── package.xml
│           │   ├── scripts/
│           │   │   └── calibrate_camera.py
│           │   └── src/
│           │       ├── depth_filter.cpp
│           │       └── obstacle_detection.py
│           ├── voice/             # 语音交互
│           │   ├── CMakeLists.txt
│           │   ├── config/
│           │   │   ├── voice_commands.json
│           │   │   └── voice_params.yaml
│           │   ├── launch/
│           │   │   └── voice.launch
│           │   ├── package.xml
│           │   ├── scripts/
│           │   │   └── train_commands.py
│           │   └── src/
│           │       └── voice_command.py
│           └── control/           # 运动控制
│               ├── CMakeLists.txt
│               ├── config/
│               │   └── control_params.yaml
│               ├── launch/
│               │   └── control.launch
│               ├── package.xml
│               └── src/
│                   └── motion_controller.cpp
└── stm32/                         # STM32嵌入式固件
    ├── Core/
    │   ├── Inc/
    │   │   ├── main.h
    │   │   ├── stm32l4xx_hal_conf.h
    │   │   └── stm32l4xx_it.h
    │   └── Src/
    │       ├── main.c
    │       ├── stm32l4xx_hal_msp.c
    │       ├── stm32l4xx_it.c
    │       └── system_stm32l4xx.c
    ├── Drivers/
    │   └── CMSIS
    │       └── Device
    │           └── ST
    │               └── STM32L4xx
    │                   └── License.md
    ├── HARDWARE/                  # 硬件驱动层
    │   ├── ads8867.c              # ADC驱动
    │   ├── ads8867.h
    │   ├── encoder.c              # 编码器接口
    │   ├── encoder.h
    │   ├── ly68l6400.c            # PSRAM驱动
    │   ├── ly68l6400.h
    │   ├── mcp4017.c              # 数字电位器
    │   ├── mcp4017.h
    │   ├── motor_pid.c            # PID控制算法
    │   ├── motor_pid.h
    │   ├── mpu6050.c              # 6轴IMU驱动
    │   ├── mpu6050.h
    │   ├── myfun.c                # 通用功能
    │   ├── myfun.h
    │   ├── myusart.c              # 自定义串口协议
    │   ├── myusart.h
    │   ├── pwm_motor.c            # PWM电机驱动
    │   ├── pwm_motor.h
    │   ├── w25q128.c              # Flash存储
    │   └── w25q128.h
    ├── my32.ioc                   # STM32CubeMX工程文件
    └── showdata.ini               # 数据展示配置文件


## 🛠️ 硬件架构

| 组件           | 型号/接口                  | 功能描述           |
| -------------- | -------------------------- | ------------------ |
| **主控制器**   | Jetson Nano 4GB            | 上层决策与感知处理 |
| **协处理器**   | STM32L476RG                | 底层实时控制       |
| **深度相机**   | 奥比中光Astra Pro (USB3.0) | 环境感知与避障     |
| **语音模块**   | CI1302 (UART)              | 离线语音识别       |
| **IMU传感器**  | MPU6050 (I2C)              | 6轴姿态感知        |
| **电机驱动**   | TB6612FNG (PWM)            | 双路电机控制       |
| **编码器**     | 正交编码器 (TIM)           | 电机转速反馈       |
| **ADC模块**    | ADS8867 (SPI)              | 高精度数据采集     |
| **数字电位器** | MCP4017 (I2C)              | 参数调节           |
| **Flash存储**  | W25Q128 (SPI)              | 数据存储           |

## 📊 技术指标

| 性能指标           | 参数值    | 测试条件           |
| ------------------ | --------- | ------------------ |
| **控制频率**       | 1kHz      | STM32定时器中断    |
| **通信速率**       | 115200bps | 串口UART           |
| **端到端延迟**     | <200ms    | 语音指令到电机响应 |
| **障碍物检测距离** | 0.3m - 5m | 深度相机有效范围   |
| **运动控制精度**   | ±2cm/10m  | 直线轨迹跟踪       |
| **语音识别准确率** | >85%      | 安静实验室环境     |
| **最大运动速度**   | 0.8m/s    | 平地面测试         |
| **连续运行时间**   | >2小时    | 满载运行           |

## 🧩 模块详细说明

### 1. 硬件接口层 (bringup)

**功能**：负责Jetson Nano与STM32之间的可靠通信

**核心文件**：
- `protocol.hpp/cpp` - 自定义通信协议实现
- `serial_node.cpp` - 串口通信节点

**通信协议特性**：

```
// 协议帧结构
typedef struct {
uint8_t header;         // 0xAA
uint8_t cmd_type;       // 指令类型
uint16_t data_len;      // 数据长度
uint8_t payload[32];    // 数据负载
uint16_t crc16;         // CRC16校验
uint8_t footer;         // 0x55
} ProtocolFrame;
```

**支持指令类型**：

- `CMD_SET_VELOCITY` - 设置速度指令
- `CMD_GET_ODOM` - 请求里程计数据  
- `CMD_GET_IMU` - 请求IMU数据
- `CMD_ACK` - 应答指令

### 2. 视觉感知层 (vision)

**功能**：处理深度相机数据，实现环境感知与障碍物检测

**核心文件**：
- `obstacle_detection.py` - Python障碍物检测
- `depth_filter.cpp` - C++深度数据滤波
- `calibrate_camera.py` - 相机校准脚本

**算法流程**：

```
深度数据处理流程
深度图像获取 (V4L2接口)
无效值滤波 (NaN/Inf处理)
距离转换 (像素到实际距离)
障碍物检测 (最近距离计算)
安全区域划分 (动态阈值)
```

**配置参数** (`vision_params.yaml`):

```yaml
obstacle_detection:
min_distance: 0.3      # 最小检测距离 (米)
max_distance: 5.0      # 最大检测距离 (米)
safety_threshold: 0.8   # 安全距离阈值 (米)
depth_filter:
bilateral_d: 5
bilateral_sigma_color: 50.0
bilateral_sigma_space: 50.0
median_kernel_size: 3
gaussian_kernel_size: 3
```

### 3. 语音交互层 (voice)

<p align="center">
  <br>
  <em>CI1302离线语音模块连接示意图</em>
</p>

**功能**：实现离线语音识别与控制

**核心文件**：
- `voice_command.py` - 语音命令处理节点
- `train_commands.py` - 命令训练脚本
- `voice_commands.json` - 命令配置文件

**支持指令**：

```json
{
"wakewords": ["小智", "robot", "hey bot"],
"commands": {
    "前进": "forward",
    "后退": "backward",
    "左转": "turn left",
    "右转": "turn right",
    "停止": "stop",
    "加速": "faster",
    "减速": "slower"
	}
}
```

**通信协议**：

- 串口协议：115200bps, 8N1
- 数据格式：`asr:命令文本` 或 `command:预定义命令`

### 4. 运动控制层 (control)

**功能**：实现运动决策与控制算法

**核心文件**：
- `motion_controller.cpp` - 运动控制节点

**控制算法特性**：

```cpp
// PID控制结构
typedef struct {
float Kp, Ki, Kd;              // PID参数
float integral_max;            // 积分限幅
float output_max;              // 输出限幅
float derivative_alpha;        // 微分滤波系数
bool derivative_on_measurement;// 微分先行
float integral;                // 积分项
float last_error;              // 上次误差
float last_measurement;         // 上次测量值
} PID_Controller;
```

**配置参数** (`control_params.yaml`):

```yaml
motion_controller:
max_linear_speed: 0.5    # m/s
max_angular_speed: 1.0   # rad/s
safety_distance: 0.5     # m
control_rate: 10.0       # Hz
pid_linear:
kp: 0.8
ki: 0.01
kd: 0.05
pid_angular:
kp: 1.2
ki: 0.02
kd: 0.1
```

### 5. STM32嵌入式层

**功能**：底层硬件控制与实时响应

**核心驱动**：

- `motor_pid.c/h` - PID电机控制算法
- `encoder.c/h` - 正交编码器接口
- `mpu6050.c/h` - MPU6050 IMU驱动
- `myusart.c/h` - 自定义串口协议
- `pwm_motor.c/h` - PWM电机驱动
- `ads8867.c/h` - ADC数据采集驱动
- `mcp4017.c/h` - 数字电位器控制
- `w25q128.c/h` - Flash存储管理

**实时性能**：

- 1kHz控制循环频率 (定时器中断)
- 硬件PWM生成 (20kHz频率)
- 编码器高速计数 (2000PPR)
- 低延迟中断响应 (<1μs)
- 串口通信速率 (115200bps)
- SPI传输速率 (10MHz)
- I2C通信速率 (400kHz)

**关键特性**：

**PID控制算法** (`motor_pid.c/h`):

```c
// PID控制器结构
typedef struct {
    float Kp, Ki, Kd;              // PID参数
    float integral_max;            // 积分限幅
    float output_max;              // 输出限幅
    float derivative_alpha;        // 微分滤波系数
    bool derivative_on_measurement;// 微分先行
    float integral;                // 积分项
    float last_error;              // 上次误差
    float last_measurement;        // 上次测量值
} PID_Controller;

// PID更新函数
float PID_Update(PID_Controller* pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // 积分项（带限幅）
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    
    // 微分项（微分先行）
    float derivative = (measurement - pid->last_measurement) / dt;
    float d_term = -pid->Kd * derivative;
    
    // 输出计算
    float output = pid->Kp * error + pid->Ki * pid->integral + d_term;
    
    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    else if (output < -pid->output_max) output = -pid->output_max;
    
    // 更新状态
    pid->last_measurement = measurement;
    pid->last_error = error;

    return output;
}
```



**编码器接口** (`encoder.c/h`):

```c
// 编码器初始化
void Encoder_Init(TIM_HandleTypeDef* htim) {
    // 正交编码器模式配置
    htim->Init.Period = 0xFFFF;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    // 启动编码器接口
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

// 获取编码器计数
int32_t Encoder_GetCount(TIM_HandleTypeDef* htim) {
    return (int32_t)__HAL_TIM_GET_COUNTER(htim);
}

// 计算转速（RPM）
float Encoder_GetSpeedRPM(TIM_HandleTypeDef* htim, float dt, uint16_t pulses_per_rev) {
    int32_t count = Encoder_GetCount(htim);
    float rpm = (count * 60.0f) / (pulses_per_rev * dt * 4); // 4倍频
    __HAL_TIM_SET_COUNTER(htim, 0); // 重置计数器
    return rpm;
}
```

**MPU6050驱动** (`mpu6050.c/h`):

```c
// MPU6050初始化
uint8_t MPU6050_Init(I2C_HandleTypeDef* hi2c) {
    // 唤醒设备
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(100);
    
    // 配置陀螺仪量程 ±2000dps
    MPU605极0_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x18);
    
    // 配置加速度计量程 ±8g
    MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x10);
    
    // 配置DLPF带宽 44Hz
    MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03);
    
    return HAL_OK;
}

// 读取6轴数据
void MPU6050_ReadData(MPU6050_Data* data) {
    uint8_t buffer[14];
    MPU6050_ReadReg(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    
    data->accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
    data->temp = (int16_t)(buffer[6] << 8 | buffer[7]);
    data->gyro_x = (int16_t)(buffer[8] << 8 | buffer[9]);
    data->gyro_y = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gyro_z = (int16极_t)(buffer[12] << 8 | buffer[13]);
}
```

**自定义串口协议** (`myusart.c/h`):

```c
// 协议帧解析状态机
typedef enum {
    STATE_WAIT_HEADER,
    STATE_READ_CMD_TYPE,
    STATE_READ_DATA_LEN_L,
    STATE_READ_DATA_LEN_H,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC_L,
    STATE_READ_CRC_H,
    STATE_WAIT_FOOTER
} ProtocolState;

// 协议处理函数
void Protocol_ProcessByte(uint8_t byte) {
    static ProtocolState state = STATE_WAIT_HEADER;
    static ProtocolFrame frame;
    static uint16_t payload_index = 0;
    
    switch (state) {
        case STATE_WAIT_HEADER:
            if (byte == PROTOCOL_HEADER) {
                frame.header = byte;
                state = STATE_READ_CMD_TYPE;
            }
            break;
            
        case STATE_READ_CMD_TYPE:
            frame.cmd_type = byte;
            state = STATE_READ_DATA_LEN_L;
            break;
            
        case STATE_READ_DATA_LEN_L:
            frame.data_len = byte;
            state = STATE_READ_DATA_LEN_H;
            break;
            
        case STATE_READ_DATA_LEN_H:
            frame.data_len |= (byte << 8);
            payload_index = 0;
            state = STATE_READ_PAYLOAD;
            break;
            
        case STATE_READ_PAYLOAD:
            if (payload_index < frame.data_len && payload_index < sizeof(frame.payload)) {
                frame.payload[payload_index++] = byte;
                if (payload_index >= frame.data_len) {
                    state = STATE_READ_CRC_L;
                }
            } else {
                state = STATE_WAIT_HEADER; // 错误恢复
            }
            break;
            
        case STATE_READ_CRC_L:
            frame.crc16 = byte;
            state = STATE_READ_CRC_H;
            break;
            
        case STATE_READ_CRC_H:
            frame.crc16 |= (byte << 8);
            state = STATE_WAIT_FOOTER;
            break;
            
        case STATE_WAIT_FOOTER:
            if (byte == PROTOCOL_FOOTER) {
                frame.footer = byte;
                if (Protocol_VerifyCRC(&frame)) {
                    Protocol_HandleFrame(&frame);
                }
            }
            state = STATE_WAIT_HEADER;
            break;
    }
}
```

**PWM电机控制** (`pwm_motor.c/h`):

```c
// 电机初始化
void Motor_Init(TIM_HandleTypeDef* htim, uint32_t channel) {
    // PWM配置
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim, channel);
}

// 设置电机速度
void Motor_SetSpeed(TIM_HandleTypeDef* htim, uint32_t channel, float speed) {
    // 速度限幅 (-100% to 100%)
    speed = fmaxf(-1.0f, fminf(1.0f, speed));
    
    // 计算PWM占空比
    uint32_t pulse = (uint32_t)(fabsf(speed) * (htim->Init.Period / 2));
    
    if (speed >= 0) {
        // 正转
        __HAL_TIM_SET_COMPARE(htim, channel, pulse);
        Motor_SetDirection(MOTOR_DIR_FORWARD);
    } else {
        // 反转
        __HAL_TIM_SET_COMPARE(htim, channel, pulse);
        Motor_SetDirection(MOTOR_DIR_REVERSE);
    }
}
```

**中断处理** (`stm32l4xx_it.c`):

```c
// 1kHz定时器中断
void TIM1极_UP_TIM16_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
            
            // 1kHz控制循环
            Control_Loop();
        }
    }
}

// 控制循环
void Control_Loop(void) {
    static uint32_t tick = 0;
    
    // 读取传感器数据
    MPU6050_Data imu_data;
    MPU6050_ReadData(&imu_data);
    
    int32_t encoder_count = Encoder_GetCount(&htim2);
    
    // 计算控制输出
    float control_output = PID_Update(&pid_controller, target_speed, current_speed, 0.001f);
    
    // 应用控制输出
    Motor_SetSpeed(&htim3, TIM_CHANNEL_1, control_output);
    
    // 定期发送数据（100Hz）
    if (tick % 10 == 0) {
        Protocol_SendData(&huart1, &imu_data, encoder_count);
    }
    
    tick++;
}
```

**配置参数** (`showdata.ini`):

```ini
[system]
control_frequency=1000
uart_baudrate=115200
i2c_speed=400000
spi_speed=10000000

[pid]
kp_linear=0.8
ki_linear=0.01
kd_linear=0.05
kp_angular=1.2
ki_angular=0.02
kd_angular=0.1
integral_max=1000
output_max=800

[encoder]
pulses_per_revolution=2000
filter_alpha=0.2

[mpu6050]
accel_scale=16384.0
gyro_scale=131.0
dlpf_bandwidth=5

[motor]
pwm_frequency=20000
max_duty_cycle=950
min_duty_cycle=50
dead_time=100
```