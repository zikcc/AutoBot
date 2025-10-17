// autobot_bringup/src/protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// 协议常量定义
#define PROTOCOL_HEADER 0xAA
#define PROTOCOL_FOOTER 0x55
#define MAX_FRAME_LEN 64

// 指令类型枚举
typedef enum {
    CMD_SET_VELOCITY = 0x10,     // 设置速度指令
    CMD_GET_ODOM = 0x11,         // 请求里程计数据
    CMD_GET_IMU = 0x12,          // 请求IMU数据
    CMD_ACK = 0xFF               // 应答指令
} CommandType;

// 协议帧结构体
#pragma pack(push, 1) // 确保1字节对齐，防止结构体填充
typedef struct {
    uint8_t header;
    uint8_t cmd_type;
    uint16_t data_len;
    uint8_t payload[32];
    uint16_t crc16;
    uint8_t footer;
} ProtocolFrame;
#pragma pack(pop)

// 函数声明
#ifdef __cplusplus
extern "C" {
#endif

uint16_t calculate_crc16(const uint8_t *data, uint16_t length);
bool verify_frame(const ProtocolFrame *frame);
void build_frame(ProtocolFrame *frame, uint8_t cmd_type, const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H