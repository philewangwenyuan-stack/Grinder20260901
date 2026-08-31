/**
 * @file sl_crc16.h
 * @brief CRC16-CCITT 校验和计算模块
 *
 * 提供 CRC16-CCITT 校验和算法，用于 SL-LinkA 协议帧的数据完整性校验。
 * 多项式 0x1021，初始值 0xFFFF。
 */

#ifndef SL_CRC16_H
#define SL_CRC16_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算 CRC16-CCITT 校验和
 * @param data 数据缓冲区指针
 * @param len 数据长度
 * @return CRC16 校验值
 *
 * 算法: CRC16-CCITT
 * 多项式: 0x1021
 * 初始值: 0xFFFF
 */
uint16_t sl_crc16_calculate(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // SL_CRC16_H
