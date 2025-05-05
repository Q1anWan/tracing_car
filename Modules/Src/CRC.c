#include "CRC.h"
#include <stdint.h>
#include <stddef.h>

// CRC8计算函数
uint8_t CRC8Maxim(uint8_t *data, uint16_t length) {
    uint8_t crc = 0x00;  // 初始值
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C;  // 0x8C 为反转后的多项式
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint32_t CRC32(uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;  // 初始值
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xEDB88320;  // 反转多项式
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;  // 最终异或
}
