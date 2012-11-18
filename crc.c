#include "crc.h"

#include	<util/crc16.h>

uint8_t crc7 (uint8_t icrc, void *data, int len)
{
	int i;
	register uint8_t bit;
	register uint8_t c;
	register uint8_t crc = icrc;

	for (i = 0; i < len; i++, data++)
	{
		c = *((uint8_t *) data);
		for (bit = 0; bit < 8; bit++)
		{
			crc <<= 1;
			if ((c ^ crc) & 0x80)
				crc ^= 0x09;

			c <<= 1;
		}
		crc &= 0x7F;
	}
	return crc;
}

// avr-libc's _crc16_update is equivalent to the following:
//
// 	uint16_t _crc16_update(uint16_t crc, uint8_t a) {
// 		int i;
// 		crc ^= a;
// 		for (i = 0; i < 8; ++i)
// 		{
// 			if (crc & 1)
// 				crc = (crc >> 1) ^ 0xA001;
// 			else
// 				crc = (crc >> 1);
// 		}
// 		return crc;
// 	}

/** block-at-once CRC16 calculator
	\param *data data to find crc16 for
	\param len length of data
	\return uint16 crc16 of passed data

	uses avr-libc's optimised crc16 routine
*/
uint16_t crc16(uint16_t icrc, void *data, int len) {
	register uint16_t crc = icrc;
	for (; len; data++, len--) {
		crc = _crc16_update(crc, *((uint8_t *) data));
	}
	return crc;
}
