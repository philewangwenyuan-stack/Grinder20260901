/**
 * @file sl_packet.h
 * @brief SL-LinkA 串口流解析器（逐字节状态机）
 *
 * 从连续的串口字节流中识别并提取完整的 SL-LinkA 协议帧。解析流程：
 *   STX1 -> STX2 -> 帧头 -> 负载 -> CRC16 校验 -> TAIL 定界符
 *
 * 使用方法：
 * 1. 调用 sl_parser_init() 初始化解析器上下文
 * 2. 将每个接收到的字节传入 sl_parse_byte()
 * 3. 当 sl_parse_byte() 返回 true 时，表示收到完整帧，
 *    并通过回调 sl_frame_received() 通知用户（用户须实现此函数）
 */

#ifndef SL_PACKET_H
#define SL_PACKET_H

#include <stdint.h>
#include <stdbool.h>
#include "sl_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

// 解析器状态
typedef enum {
    SL_PARSE_WAIT_STX1,
    SL_PARSE_WAIT_STX2,
    SL_PARSE_HEADER,
    SL_PARSE_PAYLOAD,
    SL_PARSE_CRC,
    SL_PARSE_TAIL
} sl_parse_state_t;

// 解析器上下文
typedef struct {
    sl_parse_state_t state;
    uint8_t header_buf[SL_FRAME_HEADER_SIZE];
    uint8_t* payload_buf;
    uint16_t payload_size;
    uint16_t payload_capacity;
    uint16_t header_index;
    uint16_t payload_index;
    uint16_t crc_received;
    uint8_t crc_index;
    sl_frame_header_t current_header;
} sl_parser_context_t;

/**
 * @brief 初始化解析器上下文
 * @param ctx 解析器上下文
 * @param payload_buffer 预分配的负载缓冲区（可为 NULL 表示由内部管理）
 * @param buffer_size 预分配缓冲区大小
 */
void sl_parser_init(sl_parser_context_t* ctx, uint8_t* payload_buffer, uint16_t buffer_size);

/**
 * @brief 从串口流中解析一个字节
 * @param ctx 解析器上下文
 * @param byte 输入字节
 * @return 若收到完整一帧返回 true，否则 false
 */
bool sl_parse_byte(sl_parser_context_t* ctx, uint8_t byte);

/**
 * @brief 将解析器重置为初始状态
 * @param ctx 解析器上下文
 */
void sl_parser_reset(sl_parser_context_t* ctx);

/**
 * @brief 收到完整一帧时调用的回调
 * @param header 帧头指针
 * @param payload 负载数据指针
 * @param len 负载长度
 *
 * 注意：用户必须实现此函数
 */
void sl_frame_received(const sl_frame_header_t* header, const uint8_t* payload, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // SL_PACKET_H
