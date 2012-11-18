#include	"eeconfig.h"

#include	<avr/eeprom.h>

#include	"crc.h"
#include	"config.h"

/// in-memory configuration data structure
eeconfig_struct config;

/// in-eeprom configuration data structure
eeconfig_struct EEMEM EE_config;

uint8_t eeconfig_init() {
	uint16_t mycrc;
	eeprom_read_block(&config, &EE_config, sizeof(eeconfig_struct));
	mycrc = crc16(0xFFFF, &config, sizeof(eeconfig_struct) - sizeof(uint16_t));
	if (mycrc != config.crc)
		return 0;
	return 1;
}

void eeconfig_save() {
	for (;eeprom_is_ready() == 0;);
	config.crc = crc16(0xFFFF, &config, sizeof(eeconfig_struct) - sizeof(uint16_t));
	eeprom_write_block(&config, &EE_config, sizeof(eeconfig_struct));
}
