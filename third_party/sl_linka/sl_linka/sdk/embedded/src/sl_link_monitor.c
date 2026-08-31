#include "sl_link_monitor.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

void sl_link_monitor_init(sl_link_monitor_t* monitor) {
    if (!monitor) return;
    memset(monitor, 0, sizeof(sl_link_monitor_t));
}

static sl_device_stats_t* find_or_create_slot(sl_link_monitor_t* monitor, uint8_t src_id) {
    // 1. Find existing
    for (int i = 0; i < SL_MAX_MONITORED_DEVICES; i++) {
        if (monitor->devices[i].is_active && monitor->devices[i].src_id == src_id) {
            return &monitor->devices[i];
        }
    }
    // 2. Find empty slot
    for (int i = 0; i < SL_MAX_MONITORED_DEVICES; i++) {
        if (!monitor->devices[i].is_active) {
            monitor->devices[i].src_id = src_id;
            monitor->devices[i].is_active = true;
            monitor->devices[i].total_received = 0;
            monitor->devices[i].total_lost = 0;
            // Note: We don't set last_seq here yet, update will do it
            return &monitor->devices[i];
        }
    }
    return NULL; // No more slots
}

uint16_t sl_link_monitor_update(sl_link_monitor_t* monitor, uint8_t src_id, uint16_t seq) {
    if (!monitor) return 0;
    
    sl_device_stats_t* stats = find_or_create_slot(monitor, src_id);
    if (!stats) return 0;
    
    uint16_t lost_now = 0;
    
    if (stats->total_received > 0) {
        // Check for continuity
        uint16_t expected = (uint16_t)((stats->last_seq + 1) & 0xFFFF);
        if (seq != expected) {
            // Gap detected. Calculate distance handling wrap-around
            lost_now = (uint16_t)((seq - stats->last_seq + 65536) % 65536 - 1);
            stats->total_lost += lost_now;
        }
    }
    
    stats->last_seq = seq;
    stats->total_received++;
    
    return lost_now;
}

void sl_link_monitor_log(const sl_link_monitor_t* monitor) {
    if (!monitor) return;
    printf("\n--- SL-LinkA Quality Stats ---\n");
    printf("%-8s | %-8s | %-8s | %-8s | %-6s\n", "SRC_ID", "RECV", "LOST", "TOTAL", "LOSS%%");
    printf("---------|----------|----------|----------|--------\n");
    
    for (int i = 0; i < SL_MAX_MONITORED_DEVICES; i++) {
        if (monitor->devices[i].is_active) {
            const sl_device_stats_t* s = &monitor->devices[i];
            uint32_t total = s->total_received + s->total_lost;
            float loss_percent = (total > 0) ? ((float)s->total_lost / total * 100.0f) : 0.0f;
            
            printf("0x%02X     | %-8" PRIu32 " | %-8" PRIu32 " | %-8" PRIu32 " | %3.2f%%\n",
                   s->src_id, s->total_received, s->total_lost, total, loss_percent);
        }
    }
}

void sl_link_monitor_reset_device(sl_link_monitor_t* monitor, uint8_t src_id) {
    if (!monitor) return;
    for (int i = 0; i < SL_MAX_MONITORED_DEVICES; i++) {
        if (monitor->devices[i].is_active && monitor->devices[i].src_id == src_id) {
            monitor->devices[i].total_received = 0;
            monitor->devices[i].total_lost = 0;
            break;
        }
    }
}
