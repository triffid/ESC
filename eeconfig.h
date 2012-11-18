#ifndef	_EECONFIG_H
#define	_EECONFIG_H

#include	<stdint.h>

typedef struct {
	uint16_t servo_center;
	uint16_t servo_range;

	uint16_t crc;
} eeconfig_struct;

extern eeconfig_struct config;

uint8_t eeconfig_init(void);
void    eeconfig_save(void);

#endif	/* _EECONFIG_H */
