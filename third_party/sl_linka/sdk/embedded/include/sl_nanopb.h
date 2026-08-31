/**
 * @file sl_nanopb.h
 * @brief 基于 NanoPb 的消息发送接口
 *
 * 封装 NanoPb 序列化流程，提供高层消息发送 API：
 * - sl_send_message()：序列化 Protobuf 结构体并组帧发送
 * - sl_send_ack()：发送确认帧
 * - sl_set_source_id() / sl_get_source_id()：管理本机设备 ID
 * - 序列号自动递增管理
 *
 * 用户需实现 sl_uart_send() 以对接具体硬件 UART 驱动。
 */

#ifndef SL_NANOPB_H
#define SL_NANOPB_H

#include <stdint.h>
#include <stdbool.h>
#include "pb.h"
#include "sl_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 使用 NanoPb 序列化发送一条消息
 * @param msg_id 消息 ID（来自 MessageId 枚举）
 * @param pb_struct Protobuf 结构体指针
 * @param fields pb_msgdesc_t 描述符指针（见 sl_link.pb.h 中各消息的 xxx_fields）
 * @param dst_id 目标设备 ID
 * @param comp_id 组件 ID
 * @param flags 帧标志（如需要 ACK 等）
 * @return 成功返回 true，否则 false
 */
bool sl_send_message(
    uint16_t msg_id,
    const void* pb_struct,
    const pb_msgdesc_t* fields,
    uint8_t dst_id,
    uint8_t comp_id,
    uint8_t flags,
    uint16_t ack_seq
);

/**
 * @brief 发送确认帧（通常无负载）
 * @param msg_id 被确认的消息 ID
 * @param dst_id 目标设备 ID
 * @param comp_id 组件 ID
 * @param ack_seq 被确认的序列号
 * @return 成功返回 true
 */
bool sl_send_ack(uint16_t msg_id, uint8_t dst_id, uint8_t comp_id, uint16_t ack_seq);


/**
 * @brief UART 发送函数，须由用户实现
 * @param data 待发送数据指针
 * @param len 数据长度
 * @return 成功返回 true，否则 false
 */
extern bool sl_uart_send(const uint8_t* data, uint16_t len);

/**
 * @brief 获取下一个序列号（内部自动递增）
 * @return 下一个序列号
 */
uint16_t sl_get_next_seq(void);

/**
 * @brief 设置本机源设备 ID
 * @param src_id 源设备 ID
 */
void sl_set_source_id(uint8_t src_id);

/**
 * @brief 获取当前源设备 ID
 * @return 当前源设备 ID
 */
uint8_t sl_get_source_id(void);

#ifdef __cplusplus
}
#endif

#endif // SL_NANOPB_H
