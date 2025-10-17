// autobot_bringup/src/protocol.c
#include "protocol.h"

// CRC16-CCITT计算（多项式0x1021）
uint16_t calculate_crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool verify_frame(const ProtocolFrame *frame) {
    // 检查头尾
    if (frame->header != PROTOCOL_HEADER || frame->footer != PROTOCOL_FOOTER) {
        return false;
    }
    
    // 检查数据长度
    if (frame->data_len > sizeof(frame->payload)) {
        return false;
    }
    
    // 计算并检查CRC
    uint16_t calculated_crc = calculate_crc16((const uint8_t*)frame, 
                                             sizeof(ProtocolFrame) - sizeof(uint16_t));
    return calculated_crc == frame->crc16;
}

void build_frame(ProtocolFrame *frame, uint8_t cmd_type, const uint8_t *payload, uint16_t payload_len) {
    frame->header = PROTOCOL_HEADER;
    frame->cmd_type = cmd_type;
    frame->data_len = (payload_len < sizeof(frame->payload)) ? payload_len : sizeof(frame->payload);
    frame->footer = PROTOCOL_FOOTER;
    
    // 复制数据
    if (payload && payload_len > 0) {
        memcpy(frame->payload, payload, frame->data_len);
    }
    
    // 计算CRC（在数据填充后）
    frame->crc16 = calculate_crc16((const uint8_t*)frame, 
                                  sizeof(ProtocolFrame) - sizeof(uint16_t));
}