#pragma once
#ifndef __CRC_HPP__
#define __CRC_HPP__
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

uint8_t CRC8Maxim(uint8_t *data, uint16_t len);
uint32_t CRC32(uint8_t *data, uint32_t length);

#ifdef __cplusplus
};
#endif
#endif
