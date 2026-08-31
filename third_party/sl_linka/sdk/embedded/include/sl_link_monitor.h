/**
 * @file sl_link_monitor.h
 * @brief SL-LinkA 链路质量监控模块
 *
 * 通过跟踪各设备的序列号跳变来检测丢包，提供以下功能：
 * - 按源设备 ID 自动统计收包数与丢包数
 * - 支持最多 SL_MAX_MONITORED_DEVICES 个设备的并发监控
 * - 日志输出与单设备统计重置
 */

#ifndef SL_LINK_MONITOR_H
#define SL_LINK_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SL_MAX_MONITORED_DEVICES 8

typedef struct {
    uint8_t src_id;             // 被监控的设备 ID
    uint16_t last_seq;          // 上次收到的序列号
    uint32_t total_received;    // 从该设备收到的总包数
    uint32_t total_lost;        // 检测到的总丢包数
    bool is_active;             // 该槽位是否在用
} sl_device_stats_t;

typedef struct {
    sl_device_stats_t devices[SL_MAX_MONITORED_DEVICES];
} sl_link_monitor_t;

/**
 * @brief 初始化链路监控
 * @param monitor 监控结构体指针
 */
void sl_link_monitor_init(sl_link_monitor_t* monitor);

/**
 * @brief 用新收到的一帧信息更新监控统计
 * @param monitor 监控结构体指针
 * @param src_id 该帧的源 ID
 * @param seq 该帧的序列号
 * @return 本次跳变检测到的丢包数（无丢包为 0）
 */
uint16_t sl_link_monitor_update(sl_link_monitor_t* monitor, uint8_t src_id, uint16_t seq);

/**
 * @brief 将当前统计打印到 stdout
 * @param monitor 监控结构体指针
 */
void sl_link_monitor_log(const sl_link_monitor_t* monitor);

/**
 * @brief 重置指定设备的统计
 * @param monitor 监控结构体指针
 * @param src_id 要重置的源设备 ID
 */
void sl_link_monitor_reset_device(sl_link_monitor_t* monitor, uint8_t src_id);

#ifdef __cplusplus
}
#endif

#endif // SL_LINK_MONITOR_H
