#ifndef _HAL_H
#define _HAL_H

#include <stdint.h>

typedef enum {
	MODE_COAST,
	MODE_BRAKE,
	MODE_PWM_SINGLE,
	MODE_LOCKED_ANTIPHASE
} MOTOR_MODES;

void hal_init(void);

void hal_mainloop(void);

void hal_setpwm(int16_t);
int16_t hal_getpwm(void);

void hal_setmode(MOTOR_MODES mode);
MOTOR_MODES hal_getmode(void);

void hal_reportcurrent(uint16_t milliamps);

void hal_reportmotorvoltage(uint16_t millivolts);

void hal_128us(void);
// void hal_8192us(void);
// void hal_1s(void);

// placeholders for future functions
#define hal_8192us() do {} while (0)
#define hal_1s()     do {} while (0)

#endif /* _HAL_H */
