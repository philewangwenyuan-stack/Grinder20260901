/**
 * @file sl_frame.h
 * @brief SL-LinkA 协议帧头定义
 *
 * 定义 SL-LinkA 通信协议的帧结构，包括：
 * - 帧定界符（STX1/STX2/TAIL）
 * - 协议版本号
 * - 标志位（ACK 请求、加密等）
 * - 帧头结构体 sl_frame_header_t（包含序列号、设备 ID、消息 ID 等字段）
 *
 * 所有多字节字段采用小端序。
 */

#ifndef SL_FRAME_H
#define SL_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 帧定界符
#define SL_STX1 0xFD
#define SL_STX2 0x55
#define SL_TAIL 0xFE

// 协议版本
#define SL_PROTOCOL_VERSION 0x01

// 标志位
#define SL_FLAG_ACK_REQUIRED  0x01
#define SL_FLAG_ENCRYPTED     0x02

// 最大负载长度
#define SL_MAX_PAYLOAD_SIZE 65535

// 帧头结构（小端序）
#pragma pack(push, 1)
typedef struct {
    uint8_t  stx1;        // 0xFD
    uint8_t  stx2;        // 0x55
    uint8_t  version;     // 协议版本
    uint8_t  flags;       // 标志（ACK、加密等）
    uint16_t seq;         // 序列号（发送方递增）
    uint16_t ack_seq;     // 确认序列号
    uint8_t  src_id;      // 源设备 ID
    uint8_t  dst_id;      // 目标设备 ID
    uint8_t  comp_id;     // 组件/模块 ID
    uint16_t msg_id;      // 消息 ID（Protobuf 消息类型）
    uint16_t len;         // 负载长度
} sl_frame_header_t;
#pragma pack(pop)

#define SL_FRAME_HEADER_SIZE (sizeof(sl_frame_header_t))

#ifdef __cplusplus
}
#endif

#endif // SL_FRAME_H
