/*****************************************************************************
 *                                                                            *
 * ESC - Electronic Speed Controller for R/C cars and robots                  *
 *                                                                            *
 * Hardware Abstraction Library - Any alterations to the schematic or PCB     *
 *                                should be reflected in this file            *
 *                                                                            *
 * by Triffid Hunter                                                          *
 *                                                                            *
 *                                                                            *
 * This firmware is Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the GNU General Public License as published by       *
 * the Free Software Foundation; either version 2 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * GNU General Public License for more details.                               *
 *                                                                            *
 * You should have received a copy of the GNU General Public License          *
 * along with this program; if not, write to the Free Software                *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA *
 *                                                                            *
 *****************************************************************************/

#include "hal.h"

#include <avr/interrupt.h>

#include <stdlib.h>

#include "config.h"

#include "pins.h"
#include "arduino.h"

typedef enum {
	NON_INVERT,
	INVERT
} INVERSION;

#define SET_AHI(a) WRITE(AHI, a);
#define SET_ALO(a) WRITE(ALO, a);
#define SET_BHI(a) WRITE(BHI, a);
#define SET_BLO(a) WRITE(BLO, a);

#define SET_FHI(a) do { if (current_direction == DIR_FORWARD) { WRITE(AHI, a); } else { WRITE(BHI, a); } } while (0)
#define SET_FLO(a) do { if (current_direction == DIR_FORWARD) { WRITE(ALO, a); } else { WRITE(BLO, a); } } while (0)
#define SET_RHI(a) do { if (current_direction == DIR_REVERSE) { WRITE(AHI, a); } else { WRITE(BHI, a); } } while (0)
#define SET_RLO(a) do { if (current_direction == DIR_REVERSE) { WRITE(ALO, a); } else { WRITE(BLO, a); } } while (0)

void SET_APWM(uint16_t pwm, uint8_t invert)
{
	OCR1A = pwm;
	cli();
	uint8_t nTCCR1A = (TCCR1A & ~(MASK(COM1A0))) |
		MASK(COM1A1)                             |
		(invert?0:MASK(COM1A0));
	if (TCCR1A != nTCCR1A) TCCR1A = nTCCR1A;
	sei();
}

void SET_BPWM(uint16_t pwm, uint8_t invert)
{
	OCR1B = pwm;
	cli();
	uint8_t nTCCR1A = (TCCR1A & ~(MASK(COM1B0))) |
		MASK(COM1B1)                             |
		(invert?0:MASK(COM1B0));
	if (TCCR1A != nTCCR1A) TCCR1A = nTCCR1A;
	sei();
}

#define SET_FPWM(pwm, invert) do { if (current_direction == DIR_FORWARD) { SET_APWM(pwm, invert); } else { SET_BPWM(pwm, invert); } } while (0)
#define SET_RPWM(pwm, invert) do { if (current_direction == DIR_REVERSE) { SET_APWM(pwm, invert); } else { SET_BPWM(pwm, invert); } } while (0)

#define CLEAR_APWM() do { TCCR1A &= ~(MASK(COM1A1) | MASK(COM1A0)); } while (0)
#define CLEAR_BPWM() do { TCCR1A &= ~(MASK(COM1B1) | MASK(COM1B0)); } while (0)

#define CLEAR_FPWM() do { if (current_direction == DIR_FORWARD) { CLEAR_APWM(); } else { CLEAR_BPWM(); } } while (0)
#define CLEAR_RPWM() do { if (current_direction == DIR_REVERSE) { CLEAR_APWM(); } else { CLEAR_BPWM(); } } while (0)

typedef enum {
	DIR_FORWARD,
	DIR_REVERSE
} DIRECTION;

uint8_t pwm_invalid;
int16_t validate_pwm;

uint16_t  target_pwm;
uint16_t current_pwm;
uint16_t currentlimit_pwm;

uint8_t  target_direction;
uint8_t current_direction;

MOTOR_MODES  target_mode;
MOTOR_MODES current_mode;

uint16_t brake_time;

uint16_t max_current_ma;

void hal_stop_pwm(void);
void hal_stop_pwm()
{
}

void hal_start_pwm(void);
void hal_start_pwm()
{
}

void hal_init()
{
	hal_stop_pwm();

	SET_OUTPUT(AHI); WRITE(AHI, 0);
	SET_OUTPUT(ALO); WRITE(ALO, 0);
	SET_OUTPUT(BHI); WRITE(BHI, 0);
	SET_OUTPUT(BLO); WRITE(BLO, 0);

	SET_OUTPUT(DISABLE); WRITE(DISABLE, 1);

	current_mode = MODE_COAST;
	target_mode = MODE_COAST;

	brake_time = BRAKE_TIME_US / 128;

	pwm_invalid = 64;

	currentlimit_pwm = 0;
	max_current_ma = MAX_CURRENT_MA;
}

// newpwm valid range: -1023 to +1023
void hal_setpwm(int16_t newpwm)
{
	if ((newpwm > 1023) || (newpwm < -1023))
	{
		pwm_invalid = 64;
		target_mode = MODE_COAST;
		return;
	}

	if (pwm_invalid) {
		if (abs(validate_pwm - newpwm) < 32)
			pwm_invalid--;
		if (pwm_invalid)
			return;
	}

	target_pwm = abs(newpwm);
	uint8_t new_direction = (newpwm >= 0)?DIR_FORWARD:DIR_REVERSE;
	if ((new_direction != target_direction) && (new_direction != current_direction))
		brake_time = BRAKE_TIME_US / 128;
	target_direction = new_direction;
}

int16_t hal_getpwm()
{
	int16_t r;

	r = target_pwm;
	if (target_direction == DIR_REVERSE)
		r *= -1;

	return r;
}

void hal_setmode(MOTOR_MODES newmode)
{
	switch (newmode)
	{
		case MODE_COAST: {
		}
		case MODE_BRAKE: {
		}
		case MODE_PWM_SINGLE: {
		}
		case MODE_LOCKED_ANTIPHASE: {
		}
	}
	target_mode = newmode;
}

MOTOR_MODES hal_getmode(void)
{
	return current_mode;
}

void hal_reportcurrent(uint16_t milliamps)
{
	if (milliamps > max_current_ma)
		currentlimit_pwm = max_current_ma * ((uint32_t) currentlimit_pwm) / ((uint32_t) milliamps);
	else if ((milliamps < max_current_ma) && (currentlimit_pwm < 1023))
		currentlimit_pwm++;
}

void hal_reportmotorvoltage(uint16_t millivolts)
{
	if (millivolts < MIN_VOLTAGE_MV)
	{
		currentlimit_pwm >>= 1;
		if (currentlimit_pwm < 16)
			target_mode = MODE_COAST;
	}
}

void hal_mainloop()
{
	if (pwm_invalid == 0)
	{
		if (target_mode != current_mode)
		{
			switch (target_mode)
			{
				case MODE_COAST:
				{
					// this one's easy, just release all 4 mosfets
					WRITE(DISABLE, 1);
					SET_AHI(0);
					SET_ALO(0);
					SET_BHI(0);
					SET_BLO(0);
					CLEAR_APWM();
					CLEAR_BPWM();
					current_mode = target_mode;
					break;
				}
				case MODE_BRAKE:
				{
					SET_RHI(0);
					SET_RLO(1);
					CLEAR_RPWM();
					SET_FHI(0);
					SET_FPWM(current_pwm, INVERT);
					current_mode = target_mode;
					break;
				}
				case MODE_PWM_SINGLE:
				{
					SET_RHI(0);
					SET_RLO(1);
					CLEAR_RPWM();
					SET_FHI(1);
					SET_FPWM(current_pwm, NON_INVERT);
					current_mode = target_mode;
					break;
				}
				case MODE_LOCKED_ANTIPHASE:
				{
					SET_AHI(0);
					SET_BHI(0);
					SET_ALO(0);
					SET_BLO(0);
					current_mode = target_mode;
					break;
				}
			}
		}
	}

	switch(current_mode)
	{
		case MODE_COAST: break;
		case MODE_BRAKE:
		{
			SET_FPWM(current_pwm, INVERT);
			break;
		}
		case MODE_PWM_SINGLE:
		{
			if (target_direction != current_direction)
			{
				if (brake_time == 0)
					current_direction = target_direction;
				else
				{
					SET_RHI(0);
					SET_RLO(1);
					CLEAR_RPWM();
					SET_FHI(0);
					SET_FPWM(target_pwm, INVERT);
				}
			}
			else {
				brake_time = 0;
				SET_RHI(0);
				SET_RLO(1);
				CLEAR_RPWM();
				SET_FHI(1);
				SET_FPWM(target_pwm, NON_INVERT);
			}
			break;
		}
		case MODE_LOCKED_ANTIPHASE:
		{
			uint16_t locked_pwm = 512;
			if (current_direction == DIR_FORWARD)
				locked_pwm += current_pwm / 2;
			else
				locked_pwm -= current_pwm / 2;
			SET_APWM(locked_pwm, NON_INVERT);
			SET_BPWM(locked_pwm, INVERT);
			SET_AHI(1);
			SET_BHI(1);
			break;
		}
	}

	if (pwm_invalid == 0)
	{
		if (target_pwm > currentlimit_pwm)
			current_pwm = currentlimit_pwm;
		else
			current_pwm = target_pwm;
	}
}

void hal_128us()
{
	if (brake_time)
		brake_time--;
}
