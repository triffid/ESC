/*****************************************************************************
*                                                                            *
* ESC - Electronic Speed Controller for R/C cars and robots                  *
*                                                                            *
* Setup and Main Loop routines                                               *
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

#include	<avr/interrupt.h>
#include	<avr/pgmspace.h>
#include	<avr/wdt.h>

#include	<stdint.h>
#include	<stdlib.h>

#include	"arduino.h"
#include	"watchdog.h"
#include	"serial.h"
#include	"sersendf.h"
#include	"adc_tables.h"
#include	"pins.h"
#include	"eeconfig.h"
#include	"hal.h"

#ifndef	MASK
	#define	MASK(m)	(1 << m)
#endif

#define	DEBUG 0

#define diff(a,b) (((a) >= (b))?((a)-(b)):((b)-(a)))

/*
 * Fault Conditions
 */
#define COND_NO_SERVO_SIGNAL	1
#define COND_NO_SERIAL_SIGNAL	2
#define COND_OVERVOLTAGE		4
#define COND_UNDERVOLTAGE		8
#define COND_OVERCURRENT		16
volatile uint8_t cond;

/*
 * State
 *
 * NO_SIGNAL - Both NO_SERVO_SIGNAL and NO_SERIAL_SIGNAL are asserted in condition register
 * WAIT_OK   - A fault has been recently cleared, we're waiting a little while
 * RUN       -
 */
volatile enum {
	STATE_NO_SIGNAL,
	STATE_WAIT_OK,
	STATE_RUN,
	STATE_FAULT
} state;

int16_t input_pwm;

#define MAX_PWM 1023

volatile uint16_t servo_pulse_width = 0;
uint16_t rise_edge_time = 0, fall_edge_time = 0;
volatile uint8_t servo_pulse_timeout = 0;

/*
 * Timer Flags.
 * Set in the Timer1 Overflow interrupt, these allow basic timekeeping operations in mainloop
 * They must be cleared in mainloop once they've been processed
 * There is no provision for detecting an overflow
 */
#define TFLAG_128US		1
#define TFLAG_8192US	2
#define TFLAG_1S		4
volatile uint8_t timer_flag = 0;

/*
 * Stale flags
 *
 * A variable is marked as stale when it has been updated in an ISR
 * main loop should re-check its value and respond appropriately
 */
#define STALE_VMOTOR	1
#define STALE_VSERVO	2
#define STALE_ISENSE	4
#define STALE_SERVOPWM	8
#define STALE_INPUTPWM	16
volatile uint8_t stale = 0;

#define ADC_VMOTOR	1
#define ADC_VSERVO	2
#define ADC_ISENSE	3

volatile uint16_t adc_vmotor = 0;
volatile uint16_t adc_vservo = 0;
volatile uint16_t adc_isense = 0;

uint16_t vmotor = 0;
uint16_t vservo = 0;
uint16_t isense = 0;

volatile uint8_t no_fault_time = 0;

// these values only used for printing debug info to the serial port
// if you do not wish to receive debug info, you can safely comment these
// and remove references to them throughout this file.
extern uint16_t brake_time;
uint16_t _r, _f, _o;

int main(void) {
	wdt_reset();
	wdt_disable();

	/*
	 * set up pins
	 */
	SET_INPUT(SERVO_IN);

	SET_INPUT(SERIAL_RX);
	SET_OUTPUT(SERIAL_TX);

	SET_OUTPUT(MISO);
	SET_INPUT(MOSI);
    WRITE(MOSI, 1);
	SET_OUTPUT(SCK);

	SET_INPUT(AIN0);
	SET_INPUT(AIN1);

	SET_OUTPUT(AHI); WRITE(AHI, 0);
	SET_OUTPUT(BHI); WRITE(BHI, 0);
	SET_OUTPUT(ALO); WRITE(ALO, 0);
	SET_OUTPUT(BLO); WRITE(BLO, 0);

	/*
	 * set up comparator for gross overcurrent detection
	 */
	DIDR1 = MASK(AIN1D) | MASK(AIN0D);
	ACSR  = MASK(ACIS1);
	ACSR |= MASK(ACIE);

	/*
	 * set up timer1 for input capture and clocking
	 * NOTE: mode is FAST PWM 10-bit so counter only counts to 1024
	 * this gives a PWM frequency of 7812.5 Hz, right in the middle of our audible range unfortunately
	 *
	 * Actual testing indicates that it's fairly quiet despite being within the audible range.
	 */
	TCCR1A = MASK(WGM11) | MASK(WGM10);
	TCCR1B = MASK(ICNC1) | MASK(ICES1) | MASK(WGM12) | MASK(CS10);
	TIMSK1 = MASK(ICIE1) | MASK(TOIE1);

	/*
	 * set up ADC
	 */
	// 1.1v internal reference
	ADMUX = MASK(REFS1) | MASK(REFS0);
	// enable, interrupt enable, prescale 64 (8MHz / 64 = 125KHz)
	ADCSRA = MASK(ADEN) | MASK(ADIE) | MASK(ADPS2) | MASK(ADPS1);
	// digital input disables
	DIDR0 = (MASK(SERVO_VSENSE) | MASK(MOTOR_VSENSE) | MASK(ISENSE)) & 0x3F;
	// start first conversion
	ADCSRA |= MASK(ADSC);

	/*
	 * disable unused peripherals
	 */
	PRR = (~MASK(PRTIM1)) & (~MASK(PRUSART0)) & (~MASK(PRADC));

	/*
	 * set up USART0
	 */
	serial_init();

	/*
	 * set up HAL
	 */
	hal_init();

	/*
	 * initialise some variables
	 */
	cond = COND_NO_SERVO_SIGNAL | COND_NO_SERIAL_SIGNAL;
	state = STATE_NO_SIGNAL;

	/*
	 * start watchdog timer
	 */
	wd_init();

	/*
	 * read configuration from eeprom
	 */
// 	if (eeconfig_init() == 0)
// 	{
		// eeprom configuration not valid, use defaults
		config.servo_center = 1500UL * (F_CPU / 1000000UL);
		config.servo_range  =  375UL * (F_CPU / 1000000UL);
// 	}

	/*
     * Enable interrupts- everything that uses interrupts should be initialised by now
     */

	sei();

    serial_writestr_P(PSTR("Start\n"));

    if (READ(MOSI) == 0)
	{
		// TODO: MOSI is held low- go into setup mode
	}

	uint8_t  servo_valid_pulse_count = 32;
	uint16_t servo_valid_pulse_width = config.servo_center;

	for (;;) {
		wd_reset();

		cli();
		uint8_t timer_flags = timer_flag;
		timer_flag = 0;
		sei();

		cli();
		if (stale & STALE_VMOTOR) {
			stale &= ~STALE_VMOTOR;
			sei();
			vmotor = adc_vin(ADC);

			if (vmotor < 9000) {
				cond |= COND_UNDERVOLTAGE;
// 				no_fault_time = 0;
			}
			else if (vmotor > 16000) {
				cond |= COND_OVERVOLTAGE;
// 				no_fault_time = 0;
			}
			else
			{
				cond &= ~COND_UNDERVOLTAGE;
				cond &= ~COND_OVERVOLTAGE;
			}

			hal_reportmotorvoltage(vmotor);
		}
		else
			sei();

		cli();
		if (stale & STALE_VSERVO) {
			stale &= ~STALE_VSERVO;
			sei();
			vservo = adc_vservo(ADC);
		}
		else
			sei();

		cli();
		if (stale & STALE_ISENSE) {
			stale &= ~STALE_ISENSE;
			sei();
			isense = adc_isense(ADC);

			hal_reportcurrent(isense);
		}
		else
			sei();

		cli();
		if (stale & STALE_SERVOPWM) {
			stale &= ~STALE_SERVOPWM;
			sei();

			uint16_t spw = servo_pulse_width;
			if ((spw > 2500 US) || (spw < 500 US)) {
				// if this pulse is out of range, wait for signal to stabilise again before feeding PWM to hal
				cond |= COND_NO_SERVO_SIGNAL;
				if (servo_valid_pulse_count < 16)
					servo_valid_pulse_count = 16;
				else if (servo_valid_pulse_count < 32)
					servo_valid_pulse_count++;
			}
			else {
				if (servo_valid_pulse_count)
				{
					/*
					 * wait for 32 servo pulses all within 64us of the same length
					 * and all within the standard servo pulse width range, 1-2ms
					 */
					if (diff(spw, servo_valid_pulse_width) < 128)
					{
						servo_valid_pulse_count--;
					}
					// valid_width = 3/4 valid_width + 1/4 spw
					servo_valid_pulse_width = (servo_valid_pulse_width / 2) + (servo_valid_pulse_width / 4) + (spw / 4);
				}
				else {
					// dynamically adjust if incoming pulses are out of the current servo_range window, up to some sensible maximums

					input_pwm = (((((int32_t) spw) - ((int32_t) config.servo_center)) * 1024) / config.servo_range);

					if (input_pwm > MAX_PWM)
						input_pwm = MAX_PWM;
					if (input_pwm < -MAX_PWM)
						input_pwm = -MAX_PWM;

					hal_setpwm(input_pwm);

					cond &= ~COND_NO_SERVO_SIGNAL;
				}
			}
		}
		else
			sei();

		if (timer_flags & TFLAG_128US)
		{
			hal_128us();
		}

		if (timer_flags & TFLAG_8192US)
		{
			if (servo_pulse_timeout)
				servo_pulse_timeout--;
			else
			{
				cond |= COND_NO_SERVO_SIGNAL;
				// reset pulse count, so when signal reappears we wait for it to stabilise again
				servo_valid_pulse_count = 32;
			}

			if (no_fault_time < NO_FAULT_TIME)
				no_fault_time++;

			hal_8192us();
		}

		if (timer_flags & TFLAG_1S)
		{
			hal_1s();
		}

		/***************************************************
		 *                                                 *
		 * MAIN PROGRAM LOGIC                              *
		 *                                                 *
		 **************************************************/

		if (cond & ~(COND_NO_SERVO_SIGNAL | COND_NO_SERIAL_SIGNAL)) {
			state = STATE_FAULT;
			no_fault_time = 0;
		}
		else if ((cond & COND_NO_SERVO_SIGNAL) && (cond & COND_NO_SERIAL_SIGNAL)) {
			state = STATE_NO_SIGNAL;
		}
		else if (no_fault_time < NO_FAULT_TIME) {
			state = STATE_WAIT_OK;
		}
		else {
			state = STATE_RUN;
		}

		if (state == STATE_RUN) {
			hal_setmode(MODE_PWM_SINGLE);
		}
		else {
			hal_setmode(MODE_COAST);
		}

		hal_mainloop();

		if (timer_flags & TFLAG_1S)
		{
			/*
			 * spit out a bunch of internal state to the serial port for debugging
			 */
			// 			serial_writestr_P(PSTR("1s\n"));
			sersendf_P(PSTR("W:%u / %u / %u\n"), servo_pulse_width, config.servo_center, config.servo_range);
			sersendf_P(PSTR("P:%u / %u / %u\n"), _r, _o, _f);
			sersendf_P(PSTR("X:%u / %u / %u\n"), servo_valid_pulse_count, servo_valid_pulse_width, servo_pulse_timeout);
			sersendf_P(PSTR("S:%u / 0x%sx / %u / %u\n"), state, cond, no_fault_time, hal_getmode());
			sersendf_P(PSTR("mV: %umV, mA: %umA, sV: %umV\n"), vmotor, isense, vservo);
			sersendf_P(PSTR("B: %u\n"), brake_time);
		}
	}
}

/*
 * Interrupt Routines
 */

// Analog Comparator interrupt - NOT ADC!
// this ONLY triggers if we hit the _hard_ overcurrent limit set by the resistor divider on the PCB
// as such, it is considered a permanent fault which can only be cleared by a powercycle
ISR(ANALOG_COMP_vect) {
	cond |= COND_OVERCURRENT;

	state = STATE_FAULT;

	no_fault_time = 0;

	hal_setmode(MODE_COAST);

	WRITE(AHI, 0);
	WRITE(ALO, 0);
	WRITE(BHI, 0);
	WRITE(BLO, 0);

	TCCR1A &= 0x0F;

	OCR1A = OCR1B = 0;
}

// Timer1 Input Capture interrupt
// this triggers when we receive a rising or falling edge (specified by TCCR1B:ICES1) on the servo input pin
ISR(TIMER1_CAPT_vect) {
	if (TCCR1B & MASK(ICES1))
	{
		// got a RISING edge - simply record the start time and set up the overflow counter
		rise_edge_time = _r = ICR1;
		_o = 0;
		servo_pulse_width = 0;
		// now look for a falling edge
		TCCR1B &= ~MASK(ICES1);
	}
	else
	{
		// got a FALLING edge
		fall_edge_time = _f = ICR1;
		// work out time since rising edge - servo_pulse_width gets incremented in the overflow interrupt
		servo_pulse_width = servo_pulse_width + fall_edge_time - rise_edge_time;
		// reset pulse timeout
		servo_pulse_timeout = 5;
		// now look for a RISING edge
		TCCR1B |= MASK(ICES1);
		// and tell mainloop that servo_pulse_width has been refreshed
		stale |= STALE_SERVOPWM;
	}
	// changing the target edge inside the interrupt can spuriously re-enable the flag,
	// causing the interrupt to re-fire as soon as we return
	// so we explicitly clear the flag here, and simply hope that there wasn't a genuine edge while we were servicing the ISR!
	// if we did, then we're probably picking up noise anyway
	TIFR1 = MASK(ICF1);
}

// Timer1 Overflow Interrupt
// every 1024 / 8MHz = 128uS
ISR(TIMER1_OVF_vect) {
	static uint16_t second_counter = 0;
	static uint8_t ovf_counter = 0;

	if (TIMSK1 & MASK(OCIE1A)) {
		WRITE(ALO, 0);
		WRITE(BHI, 0);
		WRITE(AHI, 1);
		WRITE(BLO, 1);
	}

	// if we're waiting for a falling pulse
	if ((TCCR1B & MASK(ICES1)) == 0)
	{
		// record this overflow
		servo_pulse_width += 1024 - rise_edge_time;
		rise_edge_time = 0;
		_o++;
	}

	timer_flag |= TFLAG_128US;

	// every 65536 / 8MHz = 8.192mS
	if (++ovf_counter & 64) {
		ovf_counter = 0;

		timer_flag |= TFLAG_8192US;
	}

	// we want to do c += 128; if (c >= 1000000) { c -= 1000000; ... }
	// would be nice if we could use a uint16_t for c.
	// turns out, 1000000 and 128 both have 64 as a common factor
	// 15625 * 64 = 1000000, and 2 * 64 = 128
	// so we can simply do c += 2; if (c >= 15625) { c -= 15625; ... }
	second_counter += 2;
	if (second_counter >= 15625) {
		second_counter -= 15625;

		timer_flag |= TFLAG_1S;
	}
}

// ADC interrupt
// triggers whenever we complete a conversion
ISR(ADC_vect) {
	static uint8_t which = 0;
	switch (which) {
		case ADC_VMOTOR: {
			adc_vmotor = ADC;
			stale |= STALE_VMOTOR;
			break;
		};
		case ADC_VSERVO: {
			adc_vservo = ADC;
			stale |= STALE_VSERVO;
			break;
		};
		case ADC_ISENSE: {
			adc_isense = ADC;
			stale |= STALE_ISENSE;
			break;
		};
	}

	which++;
	if (which >= 3) which -= 3;

	ADMUX &= 0xF0;
	switch (which) {
		case ADC_VMOTOR:
			ADMUX |= MOTOR_VSENSE;
			break;
		case ADC_VSERVO:
			ADMUX |= SERVO_VSENSE;
			break;
		case ADC_ISENSE:
			ADMUX |= ISENSE;
			break;
	}
	ADCSRA |= MASK(ADSC);
}


#include <avr/fuse.h>
#include <avr/io.h>

// TODO: check that atmega328p is selected, these fuses are only valid for '328p!
FUSES = {
	// CKSEL 0xD = Internal R/C osc @ 8MHz
    .low =      FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3,
	// enable SPI programming, keep EEPROM contents when instructed to chip erase
    .high =     FUSE_SPIEN & FUSE_EESAVE,
	// BODLEVEL = 0x5 - BOD at 2.5 to 2.9v (2.7v nominal)
    .extended = FUSE_BODLEVEL1,
};
// nothing locked - no bootloader. program chip using ISP header
LOCKBITS = 0xFF;
