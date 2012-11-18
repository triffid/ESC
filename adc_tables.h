#ifndef _ADC_TABLES_H
#define _ADC_TABLES_H

#include <stdint.h>

#include <avr/pgmspace.h>

extern const uint16_t adc_vin_lookup[1024];
extern const uint16_t adc_vservo_lookup[1024];
extern const uint16_t adc_isense_lookup[1024];

#define adc_vin(adc)	pgm_read_word(&(adc_vin_lookup[adc]));
#define adc_vservo(adc)	pgm_read_word(&(adc_vservo_lookup[adc]));
#define adc_isense(adc)	pgm_read_word(&(adc_isense_lookup[adc]));

#endif /* _ADC_TABLES_H */
