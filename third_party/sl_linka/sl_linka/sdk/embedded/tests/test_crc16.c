#include <stdio.h>
#include <assert.h>
#include "sl_crc16.h"

void test_crc16_basic(void) {
    const uint8_t data[] = "123456789";
    uint16_t crc = sl_crc16_calculate(data, 9);
    // CRC16-CCITT for "123456789" is 0x29B1 (Initial 0xFFFF)
    // Actually, it depends on the exact implementation. Let's check our implementation.
    // Our implementation uses 0x1021 polynomial and 0xFFFF initial.
    printf("CRC16 for '123456789': 0x%04X\n", crc);
    // For CCITT-FALSE (Initial 0xFFFF): 0x29B1
    assert(crc == 0x29B1);
}

void test_crc16_empty(void) {
    uint16_t crc = sl_crc16_calculate(NULL, 0);
    assert(crc == 0xFFFF); // Initial value
}

int main(void) {
    printf("Running CRC16 tests...\n");
    test_crc16_basic();
    test_crc16_empty();
    printf("CRC16 tests passed!\n");
    return 0;
}
