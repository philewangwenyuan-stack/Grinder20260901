#include "sl_crc16.h"

uint16_t sl_crc16_calculate(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;  // Initial value
    
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;  // Polynomial: 0x1021
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}
