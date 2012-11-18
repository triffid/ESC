#ifndef _CRC_H
#define _CRC_H

#include <stdint.h>

uint8_t  crc7 (uint8_t  icrc, void *data, int len);
uint16_t crc16(uint16_t icrc, void *data, int len);

#endif /* _CRC_H */
