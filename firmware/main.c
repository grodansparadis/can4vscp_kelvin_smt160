/* ******************************************************************************
 * VSCP (Very Simple Control Protocol) 
 * http://www.vscp.org
 *
 *  Kelvin NTC10KA Module
 *
 * Copyright (C) 1995-2014 Ake Hedman, Grodans Paradis AB
 *                          <akhe@grodansparadis.com>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 
 *	This file is part of VSCP - Very Simple Control Protocol 	
 *	http://www.vscp.org
 *
 * ******************************************************************************
 */

#include "vscp_compiler.h"
#include "vscp_projdefs.h"

#include <p18cxxx.h>
#include <timers.h>
#include <adc.h>
#include <math.h>
#include <eeprom.h>
#include <inttypes.h>
#include <ECAN.h>
#include <vscp_firmware.h>
#include <vscp_class.h>
#include <vscp_type.h>
#include "main.h" 
#include "version.h"
#include "ntc.h"

#if defined(RELEASE)

#pragma config WDT = ON, WDTPS = 128
#pragma config OSC = HSPLL
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 3
#pragma config LVP = ON
#pragma config CPB = ON
#pragma config BBSIZ = 2048

#else

#pragma config WDT = OFF
#pragma config OSC = HSPLL
#pragma config PWRT = ON
#pragma config BOREN = BOACTIVE
#pragma config STVREN = ON
#pragma config BORV = 3
#pragma config LVP = OFF
#pragma config CPB = OFF

#endif

// ISR Routines
void isr_low(void);

// The device URL (max 32 characters including null termination)
const uint8_t vscp_deviceURL[] = "www.eurosource.se/ntc10KA_2.xml";

// Calibration index
uint8_t calibration_index = 0;

// Global Variable Declarations
int16_t current_temp[6]; // Current temperature

uint8_t adc[NUMBER_OF_TEMP_SERIES * 12]; // Current ADC values
uint8_t adc_conversion_flags; // Bits to flag new adc values
uint8_t adc_series_counter; // Series counter

uint32_t measurement_clock; // Clock for measurments
uint32_t timeout_clock; // Clock used ofr timeouts
uint8_t seconds; // counter for seconds

uint8_t seconds_temp[6]; // timers for temp event

// Alarm flag bits
uint8_t low_alarm;
uint8_t high_alarm;

// Thermistor coefficients
double sh_coefficients[6*3]; // 6 sensors each with 3 32-bit constants


///////////////////////////////////////////////////////////////////////////////
// Isr() 	- Interrupt Service Routine
//      	- Services Timer0 Overflow
//      	- Services GP3 Pin Change
//////////////////////////////////////////////////////////////////////////////


#ifdef RELOCATE
#pragma code low_vector = 0x208
#else
//#pragma code low_vector = 0x08
#endif

void interrupt low_priority interrupt_at_low_vector(void)
{

	// Check timer
	if (INTCONbits.TMR0IF) { // If A Timer0 Interrupt, Then

		// Reload value for 1 ms reolution
		WriteTimer0(TIMER0_RELOAD_VALUE);

		vscp_timer++;
		measurement_clock++;
		timeout_clock++;

		// Check for init button
		if (!(PORTC & 0x01)) {
			// Active
			vscp_initbtncnt++;
		} else {
			vscp_initbtncnt = 0;
		}

		// Status LED
		vscp_statuscnt++;
		if ((VSCP_LED_BLINK1 == vscp_initledfunc) && (vscp_statuscnt > 100)) {
			if (PORTC & 0x02) {
				PORTC &= ~0x02;
			} else {
				PORTC |= 0x02;
			}
			vscp_statuscnt = 0;
		} else if (VSCP_LED_ON == vscp_initledfunc) {
			PORTC |= 0x02;
			vscp_statuscnt = 0;
		} else if (VSCP_LED_OFF == vscp_initledfunc) {
			PORTC &= ~0x02;
			vscp_statuscnt = 0;
		}


		INTCONbits.TMR0IF = 0; // Clear Timer0 Interrupt Flag

	}

	// Check ADC
	if (PIR1bits.ADIF) {

		switch (0x3C & ADCON0) {

		case SELECT_ADC_TEMP0:
			// Read conversion
			adc[(12 * adc_series_counter) + 0] = ADRESH;
			adc[(12 * adc_series_counter) + 1] = ADRESL;
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP1 + 1;

			// Mark that a new adc value is available if a full series
			// has been completed.
			if ((NUMBER_OF_TEMP_SERIES - 1) == adc_series_counter) {
				adc_conversion_flags |= 1;
			}
			break;

		case SELECT_ADC_TEMP1:
			// Read conversion
			adc[(12 * adc_series_counter) + 2] = ADRESH;
			adc[(12 * adc_series_counter) + 3] = ADRESL;
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP2 + 1;

			// Mark that a new adc value is available if a full series
			// has been completed.
			if ((NUMBER_OF_TEMP_SERIES - 1) == adc_series_counter) {
				adc_conversion_flags |= 1 << 1;
			}
			break;

		case SELECT_ADC_TEMP2:
			// Read conversion
			adc[(12 * adc_series_counter) + 4] = ADRESH;
			adc[(12 * adc_series_counter) + 5] = ADRESL;
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP3 + 1;

			// Mark that a new adc value is available if a full series
			// has been completed.
			if ((NUMBER_OF_TEMP_SERIES - 1) == adc_series_counter) {
				adc_conversion_flags |= 1 << 2;
			}
			break;

		case SELECT_ADC_TEMP3:
			// Read conversion
			adc[(12 * adc_series_counter) + 6] = ADRESH;
			adc[(12 * adc_series_counter) + 7] = ADRESL;
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP4 + 1;

			// Mark that a new adc value is available if a full series
			// has been completed.
			if ((NUMBER_OF_TEMP_SERIES - 1) == adc_series_counter) {
				adc_conversion_flags |= 1 << 3;
			}
			break;

		case SELECT_ADC_TEMP4:
			// Read conversion
			adc[(12 * adc_series_counter) + 8] = ADRESH;
			adc[(12 * adc_series_counter) + 9] = ADRESL;
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP5 + 1;

			// Mark that a new adc value is available if a full series
			// has been completed.
			if ((NUMBER_OF_TEMP_SERIES - 1) == adc_series_counter) {
				adc_conversion_flags |= 1 << 4;
			}
			break;

		case SELECT_ADC_TEMP5:
			// Read conversion
			adc[(12 * adc_series_counter) + 10] = ADRESH;
			adc[(12 * adc_series_counter) + 11] = ADRESL;
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP0 + 1;

			// Mark that a new adc value is available if a full series
			// has been completed.
			if ((NUMBER_OF_TEMP_SERIES - 1) == adc_series_counter) {
				adc_conversion_flags |= 1 << 5;
			}

			// Fill next series
			adc_series_counter++;
			if (adc_series_counter >= NUMBER_OF_TEMP_SERIES) {
				adc_series_counter = 0;
			}
			break;

		default:
			// Start new nonversion
			ADCON0 = SELECT_ADC_TEMP0 + 1;
			adc_series_counter = 0;
			break;
		}

		// Start conversion
		ConvertADC();

		PIR1bits.ADIF = 0; // Reset interrupt flag

	}

	return;
}


//***************************************************************************
// Main() - Main Routine
//***************************************************************************

void main()
{
	unsigned char i;

	init(); // Initialize Microcontroller

	// Check VSCP persistent storage and
	// restore if needed
	if (!vscp_check_pstorage()) {

		// Spoiled or not initialized - reinitialize
		init_app_eeprom();

	}

	vscp_init(); // Initialize the VSCP functionality

	while (TRUE) { // Loop Forever

		ClrWdt(); // Feed the dog

		if ((vscp_initbtncnt > 250) && (VSCP_STATE_INIT != vscp_node_state)) {

			// Init button pressed
			vscp_nickname = VSCP_ADDRESS_FREE;
			writeEEPROM(VSCP_EEPROM_NICKNAME, VSCP_ADDRESS_FREE);
			vscp_init();

		}


		// Check for a valid  event
		vscp_imsg.flags = 0;
		vscp_getEvent();


		switch (vscp_node_state) {

		case VSCP_STATE_STARTUP: // Cold/warm reset

			// Get nickname from EEPROM
			if (VSCP_ADDRESS_FREE == vscp_nickname) {
				// new on segment need a nickname
				vscp_node_state = VSCP_STATE_INIT;
			} else {
				// been here before - go on
				vscp_node_state = VSCP_STATE_ACTIVE;
				vscp_goActiveState();
			}
			break;

		case VSCP_STATE_INIT: // Assigning nickname
			vscp_handleProbeState();
			break;

		case VSCP_STATE_PREACTIVE: // Waiting for host initialisation
			vscp_goActiveState();
			break;

		case VSCP_STATE_ACTIVE: // The normal state

			// Check for incoming message?
			if (vscp_imsg.flags & VSCP_VALID_MSG) {

				if (VSCP_CLASS1_PROTOCOL == vscp_imsg.class) {

					// Handle protocol event
					vscp_handleProtocolEvent();

				} else if ((VSCP_CLASS1_CONTROL == vscp_imsg.class) &&
					(VSCP_TYPE_CONTROL_SYNC == vscp_imsg.type)) {
					handle_sync();
				}

			}
			break;

		case VSCP_STATE_ERROR: // Everything is *very* *very* bad.
			vscp_error();
			break;

		default: // Should not be here...
			vscp_node_state = VSCP_STATE_STARTUP;
			break;

		}


		// do a meaurement if needed
		if (measurement_clock > 1000) {

			measurement_clock = 0;
			doOneSecondWork();
			seconds++;

			// Temperature report timers are only updated if in active
			// state
			if (VSCP_STATE_ACTIVE == vscp_node_state) {
				for (i = 0; i < 6; i++) {
					seconds_temp[i]++;
				}
			}

			if (seconds > 60) {
				seconds = 0;
			}

			// Do VSCP one second jobs
			vscp_doOneSecondWork();

			// Also do some work
			doWork();

		}


	} // while
}


///////////////////////////////////////////////////////////////////////////////
// doWork
//
// The actual work is done here.
//

void doWork(void)
{
	uint8_t i, j;
	uint16_t B;
	uint64_t mean_adc;
	double resistance;
	double Rinf;
	double temp;
	double v;
	double calVoltage;
	int setpoint;

	calVoltage = ((uint16_t) readEEPROM(EEPROM_CALIBRATED_VOLTAGE_MSB)*256 +
		readEEPROM(EEPROM_CALIBRATED_VOLTAGE_LSB));

	// Check if there are new adc values to
	// convert to temperatures

	for (i = 0; i < 6; i++) {

		if (adc_conversion_flags & 1 << i) {

			// Calulate mean value for this adc
			mean_adc = 0;
			for (j = 0; j < NUMBER_OF_TEMP_SERIES; j++) {
				mean_adc += ((uint16_t) adc[12 * j + 2 * i])*256 + adc[12 * j + 2 * i + 1];
			}
			mean_adc = mean_adc / NUMBER_OF_TEMP_SERIES;

			if (1) {

				// Use B-constant
				// ==============
				// http://en.wikipedia.org/wiki/Thermistor
				// R1 = (R2V - R2V2) / V2  R2= 10K, V = 5V,  V2 = adc * voltage/1024
				// T = B / ln(r/Rinf)
				// Rinf = R0 e (-B/T0), R0=10K, T0 = 273.15 + 25 = 298.15
				B = (uint16_t) readEEPROM(2 * i + EEPROM_B_CONSTANT0_MSB)*256 +
					readEEPROM(2 * i + EEPROM_B_CONSTANT0_LSB);
				Rinf = 10000.0 * exp(B / -298.15);
				//itemp = Rinf * 10000;
				v = 5.0 * (double) mean_adc / 1025;
				//itemp = v * 100;
				resistance = (calVoltage - 10000.0 * v) / v;
				//itemp = r;
				temp = ((double) B) / log(resistance / Rinf);
				//itemp = log(r/Rinf);
				temp = temp - 273.15; // Convert Kelvin to Celcius
				current_temp[ i ] = (current_temp[ i ] + ((long) (temp * 100) + getCalibrationValue(i))) / 2;

			} else {

				// Use S-H equation
				// ================

				// Assuming a 10k Thermistor.  Calculation is actually: Resistance = (1024/ADC)
				resistance = ((10240000 / adc[2 * i + 1]) - 10000);

				/********************************************************************/
				/* Utilizes the Steinhart-Hart Thermistor Equation:					*/
				/*    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}		*/
				/********************************************************************/
				temp = log(resistance);
				temp = 1 / (sh_coefficients[i * 3] + (sh_coefficients[i * 3 + 1] * temp) +
					(sh_coefficients[i * 3 + 2] * temp * temp * temp));
				temp = temp - 273.15; // Convert Kelvin to Celsius
				current_temp[ i ] = (current_temp[ i ] + ((long) (temp * 100) + getCalibrationValue(i))) / 2;

			}
			// Check if this is the lowest temperature ever
			if (current_temp[ i ] <
				(int16_t) (readEEPROM(2 * i + EEPROM_ABSOLUT_LOW0_MSB)*256 +
				readEEPROM(2 * i + EEPROM_ABSOLUT_LOW0_LSB))) {
				// Store new lowest value
				writeEEPROM(2 * i + EEPROM_ABSOLUT_LOW0_MSB, current_temp[ i ] >> 8);
				writeEEPROM(2 * i + EEPROM_ABSOLUT_LOW0_LSB, 0xff & current_temp[ i ]);
			}

			// Check if this is the highest temperature ever
			if (current_temp[ i ] >
				(int16_t) (readEEPROM(2 * i + EEPROM_ABSOLUT_HIGH0_MSB)*256 +
				readEEPROM(2 * i + EEPROM_ABSOLUT_HIGH0_LSB))) {
				// Store new lowest value
				writeEEPROM(2 * i + EEPROM_ABSOLUT_HIGH0_MSB, current_temp[ i ] >> 8);
				writeEEPROM(2 * i + EEPROM_ABSOLUT_HIGH0_LSB, 0xff & current_temp[ i ]);
			}

			// Reset flag
			adc_conversion_flags &= ~(1 << i);

		}
	}


	// Check if alarm events should be sent
	if (VSCP_STATE_ACTIVE == vscp_node_state) {

		// Check alarm conditions
		for (i = 0; i < 6; i++) {

			// Check low alarm condition already
			if (low_alarm & 1 << i) {

				// We have an alarm condition already
				setpoint =
					(int16_t) (readEEPROM(2 * i + EEPROM_LOW_ALARM0_MSB)*256 +
					readEEPROM(2 * i + EEPROM_LOW_ALARM0_LSB) +
					readEEPROM(i + EEPROM_HYSTERESIS_SENSOR0)*100);

				// Check if it is no longer valid
				// that is under hysteresis so we can rest
				// alarm condition
				if (current_temp[ i ] > setpoint) {

					// Reset alarm confition
					low_alarm &= ~(1 << i);

				}

			} else {

				// We do not have an alarm condition
				// check if we should have
				setpoint =
					(int16_t) (readEEPROM(2 * i + EEPROM_LOW_ALARM0_MSB)*256 +
					readEEPROM(2 * i + EEPROM_LOW_ALARM0_LSB));

				if (current_temp[ i ] < setpoint) {

					// We have a low alarm condition
					low_alarm |= (1 << i);

					// Set module alarm flag
					// Note that this bit is set even if we are uanble
					// to send an alarm event.
					vscp_alarmstatus |= MODULE_LOW_ALARM;

					// Should ALARM (TURNON/TURNOFF) events be sent
					if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_LOW_ALARM) {

						vscp_omsg.class = VSCP_CLASS1_ALARM;
						vscp_omsg.type = VSCP_TYPE_ALARM_ALARM;
						vscp_omsg.priority = VSCP_PRIORITY_HIGH;
						vscp_omsg.flags = VSCP_VALID_MSG + 3;

						// Should TurnOn/TurnOff evenst be sent
						if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNX) {

							if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNON_INVERT) {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNON;
							} else {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNOFF;
							}

						}

						vscp_omsg.data[ 0 ] = i; // Index
						vscp_omsg.data[ 1 ] =
							readEEPROM(2 * i + EEPROM_SENSOR0_ZONE); // Zone
						vscp_omsg.data[ 2 ] =
							readEEPROM(2 * i + EEPROM_SENSOR0_SUBZONE); // Subzone

						// Send event
						if (!vscp_sendEvent()) {
							// Could not send alarm event
							// Reset alarm - we try again next round
							low_alarm &= ~(1 << i);
						}
					}
				}
			}

			// Check high alarm
			if (high_alarm & (1 << i)) {

				// We have an alarm condition already

				setpoint =
					(int16_t) (readEEPROM(2 * i + EEPROM_HIGH_ALARM0_MSB)*256 +
					readEEPROM(2 * i + EEPROM_HIGH_ALARM0_LSB) -
					readEEPROM(i + EEPROM_HYSTERESIS_SENSOR0)*100);

				// Under hysteresis so we can reset condition
				if (current_temp[ i ] < setpoint) {

					// Reset alarm
					high_alarm &= ~(1 << i);

				}

			} else {

				// We do not have an alarm condition
				// check for one

				setpoint = (int16_t) (readEEPROM(2 * i + EEPROM_HIGH_ALARM0_MSB)*256 +
					readEEPROM(2 * i + EEPROM_HIGH_ALARM0_LSB));

				if (current_temp[ i ] > setpoint) {

					// We have a low alarm condition
					high_alarm |= (1 << i);

					// Set module alarm flag
					// Note that this bit is set even if we are uanble
					// to send an alarm event.

					vscp_alarmstatus |= MODULE_HIGH_ALARM;

					// Should ALARM or TURNON/TURNOFF events be sent
					if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_HIGH_ALARM) {

						vscp_omsg.class = VSCP_CLASS1_ALARM;
						vscp_omsg.type = VSCP_TYPE_ALARM_ALARM;
						vscp_omsg.priority = VSCP_PRIORITY_HIGH;
						vscp_omsg.flags = VSCP_VALID_MSG + 3;

						if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNX) {
							if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNON_INVERT) {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNOFF;
							} else {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNON;
							}
						}

						vscp_omsg.data[ 0 ] = i; // Index
						vscp_omsg.data[ 1 ] =
							readEEPROM(2 * i +
							EEPROM_SENSOR0_ZONE); // Zone
						vscp_omsg.data[ 2 ] =
							readEEPROM(2 * i +
							EEPROM_SENSOR0_SUBZONE); // Subzone

						// Send event
						if (!vscp_sendEvent()) {
							// Could not send alarm event
							// Reset alarm - we try again next round
							high_alarm &= ~(1 << i);
						}
					}
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// doOneSecondWork
//

void doOneSecondWork(void)
{
	uint8_t tmp;
	uint8_t i;

	// Check if events should be sent
	if (VSCP_STATE_ACTIVE == vscp_node_state) {

		for (i = 0; i < 6; i++) {

			// Time for temperature report
			tmp = readEEPROM(EEPROM_REPORT_INTERVAL0 + i);
			if (tmp && (seconds_temp[i] > tmp)) {

				// Send event
				if (sendTempEvent(i)) {
					seconds_temp[i] = 0;
				}

			}

			// Check for continuous alarm
			if (MASK_CONTROL_CONTINUOUS & readEEPROM(EEPROM_CONTROLREG0 + i)) {

				// Check low alarm
				if (low_alarm & (1 << i)) {

					// Should ALARM or TURNON/TURNOFF events be sent
					if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_LOW_ALARM) {


						vscp_omsg.priority = VSCP_PRIORITY_HIGH;
						vscp_omsg.flags = VSCP_VALID_MSG + 3;

						if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNX) {

							if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNON_INVERT) {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNON;
							} else {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNOFF;
							}

						} else {
							// Alarm event should be sent
							vscp_omsg.class = VSCP_CLASS1_ALARM;
							vscp_omsg.type = VSCP_TYPE_ALARM_ALARM;
						}

						vscp_omsg.data[ 0 ] = i; // Index
						vscp_omsg.data[ 1 ] =
							readEEPROM(2 * i + EEPROM_SENSOR0_ZONE); // Zone
						vscp_omsg.data[ 2 ] =
							readEEPROM(2 * i + EEPROM_SENSOR0_SUBZONE); // Subzone

						// Send event
						// We allow for missing to send this event
						// as it will be sent next second instead.
						vscp_sendEvent();

					}
				}

				// Check high alarm
				if (high_alarm & (1 << i)) {

					// Should ALARM or TURNON/TURNOFF events be sent
					if ((readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_HIGH_ALARM)) {

						vscp_omsg.priority = VSCP_PRIORITY_HIGH;
						vscp_omsg.flags = VSCP_VALID_MSG + 3;

						if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNX) {
							if (readEEPROM(i + EEPROM_CONTROLREG0) & CONFIG_ENABLE_TURNON_INVERT) {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNOFF;
							} else {
								vscp_omsg.class = VSCP_CLASS1_CONTROL;
								vscp_omsg.type = VSCP_TYPE_CONTROL_TURNON;
							}
						} else {
							// Alarm event should be sent
							vscp_omsg.class = VSCP_CLASS1_ALARM;
							vscp_omsg.type = VSCP_TYPE_ALARM_ALARM;
						}

						vscp_omsg.data[ 0 ] = i; // Index
						vscp_omsg.data[ 1 ] = readEEPROM(2 * i +
							EEPROM_SENSOR0_ZONE); // Zone
						vscp_omsg.data[ 2 ] = readEEPROM(2 * i +
							EEPROM_SENSOR0_SUBZONE); // Subzone

						// Send event
						// We allow for missing to send this event
						// as it will be sent next second instead.
						vscp_sendEvent();
					}
				}
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// sendTempEvent
//

int8_t sendTempEvent(uint8_t i)
{
	vscp_omsg.priority = VSCP_PRIORITY_MEDIUM;
	vscp_omsg.flags = VSCP_VALID_MSG + 4;
	vscp_omsg.class = VSCP_CLASS1_MEASUREMENT;
	vscp_omsg.type = VSCP_TYPE_MEASUREMENT_TEMPERATURE;

	// Data format
	vscp_omsg.data[ 0 ] = 0x80 | // Normalized integer
		((0x03 & readEEPROM(i + EEPROM_CONTROLREG0)) << 3) | // Unit
		i; // Sensor
	// Exponent
	vscp_omsg.data[ 1 ] = 0x02;

	setEventData(current_temp[i], (0x03 & readEEPROM(i + EEPROM_CONTROLREG0)));

	// Send event
	if (!vscp_sendEvent()) {
		return FALSE;
	}

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// setEventData
//

void setEventData(int v, unsigned char unit)
{
	double newval;
	int ival;

	if (TEMP_UNIT_KELVIN == unit) {
		// Convert to Kelvin
		newval = Celsius2Kelvin(v);
	} else if (TEMP_UNIT_FAHRENHEIT == unit) {
		// Convert to Fahrenheit
		newval = Celsius2Fahrenheit(v);
	} else {
		// Defaults to Celsius
		newval = v;
	}

	ival = (int) newval;
	if (newval < 0) {
		vscp_omsg.data[ 1 ] |= 0x80; // if negative set sign bit
		ival = -1 * ival;
	}

	vscp_omsg.data[ 2 ] = ((ival & 0xff00) >> 8);
	vscp_omsg.data[ 3 ] = (ival & 0xff);
}

///////////////////////////////////////////////////////////////////////////////
// getCalibrationValue
//
// Get the calibration value for a specific sensor.
//

int16_t getCalibrationValue(uint8_t i)
{
	int16_t cal;

	cal = readEEPROM(2 * i + EEPROM_CALIBRATION_SENSOR0_MSB) * 256 +
		readEEPROM(2 * i + EEPROM_CALIBRATION_SENSOR0_LSB);

	return cal;
}

///////////////////////////////////////////////////////////////////////////////
// Init - Initialization Routine
//  

void init()
{

	//uint8_t msgdata[ 8 ];

	// Initialize data
	init_app_ram();

	// Initialize the uP

	// PORTB
	// RA0/AN0 - input
	// RA1/AN1 - input
	// RA2/AN2 - input
	TRISA = 0x07;

	// PortB
	// RB7 - Not used.
	// RB& - Not used.
	// RB5 - Not used.
	// RB4/AN9 - input
	// RB3 CAN RX - input
	// RB2 CAN TX - output
	// RB1/AN8 - input
	// RB0/AN10 -input
	TRISB = 0x1B;

	// RC7 - Output - Not used.
	// RC6 - Output - Not used.
	// RC5 - Output - Not used.
	// RC3 - Output - Not used.
	// RC2 - Output - Not used.
	// RC1 - Output - Status LED - Default off
	// RC0 - Input  - Init button

	TRISC = 0x01;
	PORTC = 0x00;

	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_8);
	WriteTimer0(TIMER0_RELOAD_VALUE);

	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_20_TAD,
		ADC_CH0 & ADC_INT_ON & ADC_11ANA &
		ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
		15);

	// Initialize CAN
	ECANInitialize();

	// Must be in Config mode to change many of settings.
	//ECANSetOperationMode(ECAN_OP_MODE_CONFIG);

	// Return to Normal mode to communicate.
	//ECANSetOperationMode(ECAN_OP_MODE_NORMAL);

	/*
		msgdata[ 0 ] = 1;
		msgdata[ 1 ] = 2;
		msgdata[ 2 ] = 3;
		msgdata[ 3 ] = 4;

		if ( vscp18f_sendMsg( 0x123,  msgdata, 4, CAN_TX_PRIORITY_0 & CAN_TX_XTD_FRAME & CAN_TX_NO_RTR_FRAME ) ) {
			;
		}

	 */

	// Enable global interrupt
	INTCONbits.GIE = 1;

	ConvertADC();

	return;
}

///////////////////////////////////////////////////////////////////////////////
// writeCoeffs2Ram
//

void writeCoeffs2Ram(void)
{
	int i, j;
	uint8_t c[3];

	for (i = 0; i < 6; i++) {
		for (j = 2; j > 0; j--) {
			// Store Microchip order (Little endian)
			c[2 - j] = readEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_0 + i * 3 + j);
		}
		sh_coefficients[i] = *((double*) c);
	}
}

///////////////////////////////////////////////////////////////////////////////
// init_app_ram
//

void init_app_ram(void)
{

	char i;

	measurement_clock = 0; // start a new meaurement cycle
	seconds = 0;

	// no temp. reads yet
	for (i = 0; i < 6; i++) {
		seconds_temp[i] = 0;
		current_temp[i] = 0;
	}

	// No adc conversions done
	adc_conversion_flags = 0;

	// Start with series 0
	adc_series_counter = 0;

	// No low temperature alarms
	low_alarm = 0;

	// No high temperature alarms
	high_alarm = 0;

	// Write coefficients to ram
	writeCoeffs2Ram();
}



///////////////////////////////////////////////////////////////////////////////
// init_app_eeprom
//

void init_app_eeprom(void)
{

	writeEEPROM(EEPROM_CONTROLREG0, DEFAULT_CONTROL_REG);
	writeEEPROM(EEPROM_CONTROLREG1, DEFAULT_CONTROL_REG);
	writeEEPROM(EEPROM_CONTROLREG2, DEFAULT_CONTROL_REG);
	writeEEPROM(EEPROM_CONTROLREG3, DEFAULT_CONTROL_REG);
	writeEEPROM(EEPROM_CONTROLREG4, DEFAULT_CONTROL_REG);
	writeEEPROM(EEPROM_CONTROLREG5, DEFAULT_CONTROL_REG);

	writeEEPROM(EEPROM_REPORT_INTERVAL0, DEFAULT_REPORT_INTERVAL_SENSOR0);
	writeEEPROM(EEPROM_REPORT_INTERVAL1, DEFAULT_REPORT_INTERVAL);
	writeEEPROM(EEPROM_REPORT_INTERVAL2, DEFAULT_REPORT_INTERVAL);
	writeEEPROM(EEPROM_REPORT_INTERVAL3, DEFAULT_REPORT_INTERVAL);
	writeEEPROM(EEPROM_REPORT_INTERVAL4, DEFAULT_REPORT_INTERVAL);
	writeEEPROM(EEPROM_REPORT_INTERVAL5, DEFAULT_REPORT_INTERVAL);

	// B Constants

	writeEEPROM(EEPROM_B_CONSTANT0_MSB, DEFAULT_B_CONSTANT_SENSOR0_MSB);
	writeEEPROM(EEPROM_B_CONSTANT0_LSB, DEFAULT_B_CONSTANT_SENSOR0_LSB);

	writeEEPROM(EEPROM_B_CONSTANT1_MSB, DEFAULT_B_CONSTANT_MSB);
	writeEEPROM(EEPROM_B_CONSTANT1_LSB, DEFAULT_B_CONSTANT_LSB);

	writeEEPROM(EEPROM_B_CONSTANT2_MSB, DEFAULT_B_CONSTANT_MSB);
	writeEEPROM(EEPROM_B_CONSTANT2_LSB, DEFAULT_B_CONSTANT_LSB);

	writeEEPROM(EEPROM_B_CONSTANT3_MSB, DEFAULT_B_CONSTANT_MSB);
	writeEEPROM(EEPROM_B_CONSTANT3_LSB, DEFAULT_B_CONSTANT_LSB);

	writeEEPROM(EEPROM_B_CONSTANT4_MSB, DEFAULT_B_CONSTANT_MSB);
	writeEEPROM(EEPROM_B_CONSTANT4_LSB, DEFAULT_B_CONSTANT_LSB);

	writeEEPROM(EEPROM_B_CONSTANT5_MSB, DEFAULT_B_CONSTANT_MSB);
	writeEEPROM(EEPROM_B_CONSTANT5_LSB, DEFAULT_B_CONSTANT_LSB);

	// Low alarms

	writeEEPROM(EEPROM_LOW_ALARM0_MSB, DEFAULT_LOW_ALARM_MSB);
	writeEEPROM(EEPROM_LOW_ALARM0_LSB, DEFAULT_LOW_ALARM_LSB);

	writeEEPROM(EEPROM_LOW_ALARM1_MSB, DEFAULT_LOW_ALARM_MSB);
	writeEEPROM(EEPROM_LOW_ALARM1_LSB, DEFAULT_LOW_ALARM_LSB);

	writeEEPROM(EEPROM_LOW_ALARM2_MSB, DEFAULT_LOW_ALARM_MSB);
	writeEEPROM(EEPROM_LOW_ALARM2_LSB, DEFAULT_LOW_ALARM_LSB);

	writeEEPROM(EEPROM_LOW_ALARM3_MSB, DEFAULT_LOW_ALARM_MSB);
	writeEEPROM(EEPROM_LOW_ALARM3_LSB, DEFAULT_LOW_ALARM_LSB);

	writeEEPROM(EEPROM_LOW_ALARM4_MSB, DEFAULT_LOW_ALARM_MSB);
	writeEEPROM(EEPROM_LOW_ALARM4_LSB, DEFAULT_LOW_ALARM_LSB);

	writeEEPROM(EEPROM_LOW_ALARM5_MSB, DEFAULT_LOW_ALARM_MSB);
	writeEEPROM(EEPROM_LOW_ALARM5_LSB, DEFAULT_LOW_ALARM_LSB);

	// High alarms

	writeEEPROM(EEPROM_HIGH_ALARM0_MSB, DEFAULT_HIGH_ALARM_MSB);
	writeEEPROM(EEPROM_HIGH_ALARM0_LSB, DEFAULT_HIGH_ALARM_LSB);

	writeEEPROM(EEPROM_HIGH_ALARM1_MSB, DEFAULT_HIGH_ALARM_MSB);
	writeEEPROM(EEPROM_HIGH_ALARM1_LSB, DEFAULT_HIGH_ALARM_LSB);

	writeEEPROM(EEPROM_HIGH_ALARM2_MSB, DEFAULT_HIGH_ALARM_MSB);
	writeEEPROM(EEPROM_HIGH_ALARM2_LSB, DEFAULT_HIGH_ALARM_LSB);

	writeEEPROM(EEPROM_HIGH_ALARM3_MSB, DEFAULT_HIGH_ALARM_MSB);
	writeEEPROM(EEPROM_HIGH_ALARM3_LSB, DEFAULT_HIGH_ALARM_LSB);

	writeEEPROM(EEPROM_HIGH_ALARM4_MSB, DEFAULT_HIGH_ALARM_MSB);
	writeEEPROM(EEPROM_HIGH_ALARM4_LSB, DEFAULT_HIGH_ALARM_LSB);

	writeEEPROM(EEPROM_HIGH_ALARM5_MSB, DEFAULT_HIGH_ALARM_MSB);
	writeEEPROM(EEPROM_HIGH_ALARM5_LSB, DEFAULT_HIGH_ALARM_LSB);

	// Sensor zone/subzone information

	writeEEPROM(EEPROM_SENSOR0_ZONE, DEFAULT_SENSOR_ZONE);
	writeEEPROM(EEPROM_SENSOR0_SUBZONE, DEFAULT_SENSOR_SUBZONE);

	writeEEPROM(EEPROM_SENSOR1_ZONE, DEFAULT_SENSOR_ZONE);
	writeEEPROM(EEPROM_SENSOR1_SUBZONE, DEFAULT_SENSOR_SUBZONE);

	writeEEPROM(EEPROM_SENSOR2_ZONE, DEFAULT_SENSOR_ZONE);
	writeEEPROM(EEPROM_SENSOR2_SUBZONE, DEFAULT_SENSOR_SUBZONE);

	writeEEPROM(EEPROM_SENSOR3_ZONE, DEFAULT_SENSOR_ZONE);
	writeEEPROM(EEPROM_SENSOR3_SUBZONE, DEFAULT_SENSOR_SUBZONE);

	writeEEPROM(EEPROM_SENSOR4_ZONE, DEFAULT_SENSOR_ZONE);
	writeEEPROM(EEPROM_SENSOR4_SUBZONE, DEFAULT_SENSOR_SUBZONE);

	writeEEPROM(EEPROM_SENSOR5_ZONE, DEFAULT_SENSOR_ZONE);
	writeEEPROM(EEPROM_SENSOR5_SUBZONE, DEFAULT_SENSOR_SUBZONE);

	// Absolut low temperatures

	writeEEPROM(EEPROM_ABSOLUT_LOW0_MSB, DEFAULT_LOW_MSB);
	writeEEPROM(EEPROM_ABSOLUT_LOW0_LSB, DEFAULT_LOW_LSB);

	writeEEPROM(EEPROM_ABSOLUT_LOW1_MSB, DEFAULT_LOW_MSB);
	writeEEPROM(EEPROM_ABSOLUT_LOW1_LSB, DEFAULT_LOW_LSB);

	writeEEPROM(EEPROM_ABSOLUT_LOW2_MSB, DEFAULT_LOW_MSB);
	writeEEPROM(EEPROM_ABSOLUT_LOW2_LSB, DEFAULT_LOW_LSB);

	writeEEPROM(EEPROM_ABSOLUT_LOW3_MSB, DEFAULT_LOW_MSB);
	writeEEPROM(EEPROM_ABSOLUT_LOW3_LSB, DEFAULT_LOW_LSB);

	writeEEPROM(EEPROM_ABSOLUT_LOW4_MSB, DEFAULT_LOW_MSB);
	writeEEPROM(EEPROM_ABSOLUT_LOW4_LSB, DEFAULT_LOW_LSB);

	writeEEPROM(EEPROM_ABSOLUT_LOW5_MSB, DEFAULT_LOW_MSB);
	writeEEPROM(EEPROM_ABSOLUT_LOW5_LSB, DEFAULT_LOW_LSB);

	// Absolut high temperatures

	writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_MSB);
	writeEEPROM(EEPROM_ABSOLUT_HIGH0_LSB, DEFAULT_HIGH_LSB);

	writeEEPROM(EEPROM_ABSOLUT_HIGH1_MSB, DEFAULT_HIGH_MSB);
	writeEEPROM(EEPROM_ABSOLUT_HIGH1_LSB, DEFAULT_HIGH_LSB);

	writeEEPROM(EEPROM_ABSOLUT_HIGH2_MSB, DEFAULT_HIGH_MSB);
	writeEEPROM(EEPROM_ABSOLUT_HIGH2_LSB, DEFAULT_HIGH_LSB);

	writeEEPROM(EEPROM_ABSOLUT_HIGH3_MSB, DEFAULT_HIGH_MSB);
	writeEEPROM(EEPROM_ABSOLUT_HIGH3_LSB, DEFAULT_HIGH_LSB);

	writeEEPROM(EEPROM_ABSOLUT_HIGH4_MSB, DEFAULT_HIGH_MSB);
	writeEEPROM(EEPROM_ABSOLUT_HIGH4_LSB, DEFAULT_HIGH_LSB);

	writeEEPROM(EEPROM_ABSOLUT_HIGH5_MSB, DEFAULT_HIGH_MSB);
	writeEEPROM(EEPROM_ABSOLUT_HIGH5_LSB, DEFAULT_HIGH_LSB);

	// Sensor hysteresis

	writeEEPROM(EEPROM_HYSTERESIS_SENSOR0, DEFAULT_HYSTERESIS);
	writeEEPROM(EEPROM_HYSTERESIS_SENSOR1, DEFAULT_HYSTERESIS);
	writeEEPROM(EEPROM_HYSTERESIS_SENSOR2, DEFAULT_HYSTERESIS);
	writeEEPROM(EEPROM_HYSTERESIS_SENSOR3, DEFAULT_HYSTERESIS);
	writeEEPROM(EEPROM_HYSTERESIS_SENSOR4, DEFAULT_HYSTERESIS);
	writeEEPROM(EEPROM_HYSTERESIS_SENSOR5, DEFAULT_HYSTERESIS);

	// Calibration

	writeEEPROM(EEPROM_CALIBRATION_SENSOR0_MSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR0_LSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR1_MSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR1_LSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR2_MSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR2_LSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR3_MSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR3_LSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR4_MSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR4_LSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR5_MSB, 0);
	writeEEPROM(EEPROM_CALIBRATION_SENSOR5_LSB, 0);

	// S-H Coefficients sensor 0
	writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_0, 0);
	writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_1, 0);
	writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_2, 0);
    writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_3, 0);
	writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR0_0, 0);
	writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR0_1, 0);
	writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR0_2, 0);
    writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR0_3, 0);
	writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR0_0, 0);
	writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR0_1, 0);
	writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR0_2, 0);
    writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR0_3, 0);

	// S-H Coefficients sensor 1-5
	for (uint8_t i = 0; i<6; i++) {
		writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR1_0 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR1_1 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR1_2 + i * 12, 0);
        writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR1_3 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR1_0 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR1_1 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR1_2 + i * 12, 0);
        writeEEPROM(EEPROM_COEFFICIENT_B_SENSOR1_3 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR1_0 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR1_1 + i * 12, 0);
		writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR1_2 + i * 12, 0);
        writeEEPROM(EEPROM_COEFFICIENT_C_SENSOR1_3 + i * 12, 0);
	}

	// Calibrated voltage
	writeEEPROM(EEPROM_CALIBRATED_VOLTAGE_MSB, 0xc3);
	writeEEPROM(EEPROM_CALIBRATED_VOLTAGE_LSB, 0x50);
}


///////////////////////////////////////////////////////////////////////////////
// convertTemperature
//

int8_t convertTemperature(int temp, unsigned char unit)
{
	int newval;

	if (TEMP_UNIT_KELVIN == unit) {
		// Convert to Kelvin
		newval = (int) Celsius2Kelvin(temp);
	} else if (TEMP_UNIT_FAHRENHEIT == unit) {
		// Convert to Fahrenheit
		newval = (int) Celsius2Fahrenheit(temp);
	} else {
		// Defaults to Celsius
		newval = (int) temp;
	}

	return newval;
}

///////////////////////////////////////////////////////////////////////////////
// handle_sync
//
// Report all temperature values
//

void handle_sync(void)
{
	uint8_t i;
	//uint32_t start;

	for (i = 0; i < 6; i++) {

		if ((0xff != vscp_imsg.data[ 1 ] ||
			(readEEPROM(2 * i + EEPROM_SENSOR0_ZONE) != vscp_imsg.data[ 1 ])) &&
			(0xff != vscp_imsg.data[ 2 ] ||
			(readEEPROM(2 * i + EEPROM_SENSOR0_SUBZONE) != vscp_imsg.data[ 2 ]))) {

			// We have a one second timeout
			timeout_clock = 0;
			while (!sendTempEvent(i)) {
				if (timeout_clock > 1000) break;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// vscp_readAppReg
//

uint8_t vscp_readAppReg(unsigned char reg)
{
	uint8_t rv;
	//int tmpval;
	//uint8_t val, checksum;
	//uint8_t unit = (0x03 & readEEPROM(EEPROM_CONTROLREG0));

	if (0 == vscp_page_select) {

		switch (reg) {

			// Zone
		case 0x00:
			rv = readEEPROM(EEPROM_ZONE);
			break;

			// Subzone
		case 0x01:
			rv = readEEPROM(EEPROM_SUBZONE);
			break;

			// Control register for sensor 0
		case 0x02:
			rv = readEEPROM(EEPROM_CONTROLREG0);
			break;

			// Control register for sensor 1
		case 0x03:
			rv = readEEPROM(EEPROM_CONTROLREG1);
			break;

			// Control register for sensor 2
		case 0x04:
			rv = readEEPROM(EEPROM_CONTROLREG2);
			break;

			// Control register for sensor 3
		case 0x05:
			rv = readEEPROM(EEPROM_CONTROLREG3);
			break;

			// Control register for sensor 4
		case 0x06:
			rv = readEEPROM(EEPROM_CONTROLREG4);
			break;

			// Control register for sensor 5
		case 0x07:
			rv = readEEPROM(EEPROM_CONTROLREG5);
			break;

			// MSB of current temperature for sensor 0
		case 0x08:
			rv = ((current_temp[0] & 0xff00) >> 8);
			break;

			// LSB of current temperature for sensor 0
		case 0x09:
			rv = (current_temp[0] & 0x00ff);
			break;

			// MSB of current temperature for sensor 1
		case 0x0A:
			rv = ((current_temp[1] & 0xff00) >> 8);
			break;

			// LSB of current temperature for sensor 1
		case 0x0B:
			rv = (current_temp[1] & 0x00ff);
			break;

			// MSB of current temperature for sensor 2
		case 0x0C:
			rv = ((current_temp[2] & 0xff00) >> 8);
			break;

			// LSB of current temperature for sensor 2
		case 0x0D:
			rv = (current_temp[2] & 0x00ff);
			break;

			// MSB of current temperature for sensor 3
		case 0x0E:
			rv = ((current_temp[3] & 0xff00) >> 8);
			break;

			// LSB of current temperature for sensor 3
		case 0x0F:
			rv = (current_temp[3] & 0x00ff);
			break;

			// MSB of current temperature for sensor 4
		case 0x10:
			rv = ((current_temp[4] & 0xff00) >> 8);
			break;

			// LSB of current temperature for sensor 4
		case 0x11:
			rv = (current_temp[4] & 0x00ff);
			break;

			// MSB of current temperature for sensor 4
		case 0x12:
			rv = ((current_temp[4] & 0xff00) >> 8);
			break;

			// LSB of current temperature for sensor 4
		case 0x13:
			rv = (current_temp[4] & 0x00ff);
			break;

			// Report interval register for sensor 0
		case 0x14:
			rv = readEEPROM(EEPROM_REPORT_INTERVAL0);
			break;

			// Report interval register for sensor 1
		case 0x15:
			rv = readEEPROM(EEPROM_REPORT_INTERVAL1);
			break;

			// Report interval register for sensor 2
		case 0x16:
			rv = readEEPROM(EEPROM_REPORT_INTERVAL2);
			break;

			// Report interval register for sensor 3
		case 0x17:
			rv = readEEPROM(EEPROM_REPORT_INTERVAL3);
			break;

			// Report interval register for sensor 4
		case 0x18:
			rv = readEEPROM(EEPROM_REPORT_INTERVAL4);
			break;

			// Report interval register for sensor 5
		case 0x19:
			rv = readEEPROM(EEPROM_REPORT_INTERVAL5);
			break;

			// B Constants

			// B constant MSB for sensor 0
		case 0x26:
			rv = readEEPROM(EEPROM_B_CONSTANT0_MSB);
			break;

			// B constant LSB for sensor 0
		case 0x27:
			rv = readEEPROM(EEPROM_B_CONSTANT0_LSB);
			break;

			// B constant MSB for sensor 1
		case 0x28:
			rv = readEEPROM(EEPROM_B_CONSTANT1_MSB);
			break;

			// B constant LSB for sensor 1
		case 0x29:
			rv = readEEPROM(EEPROM_B_CONSTANT1_LSB);
			break;

			// B constant MSB for sensor 2
		case 0x2A:
			rv = readEEPROM(EEPROM_B_CONSTANT2_MSB);
			break;

			// B constant LSB for sensor 2
		case 0x2B:
			rv = readEEPROM(EEPROM_B_CONSTANT2_LSB);
			break;

			// B constant MSB for sensor 3
		case 0x2C:
			rv = readEEPROM(EEPROM_B_CONSTANT3_MSB);
			break;

			// B constant LSB for sensor 3
		case 0x2D:
			rv = readEEPROM(EEPROM_B_CONSTANT3_LSB);
			break;

			// B constant MSB for sensor 4
		case 0x2E:
			rv = readEEPROM(EEPROM_B_CONSTANT4_MSB);
			break;

			// B constant LSB for sensor 4
		case 0x2F:
			rv = readEEPROM(EEPROM_B_CONSTANT4_LSB);
			break;

			// B constant MSB for sensor 5
		case 0x30:
			rv = readEEPROM(EEPROM_B_CONSTANT5_MSB);
			break;

			// B constant LSB for sensor 5
		case 0x31:
			rv = readEEPROM(EEPROM_B_CONSTANT5_LSB);
			break;


			// Low alarm registers


			// Low alarm registers MSB for sensor 0
		case 0x32:
			rv = readEEPROM(EEPROM_LOW_ALARM0_MSB);
			break;

			// Low alarm registers LSB for sensor 0
		case 0x33:
			rv = readEEPROM(EEPROM_LOW_ALARM0_LSB);
			break;

			// Low alarm registers MSB for sensor 1
		case 0x34:
			rv = readEEPROM(EEPROM_LOW_ALARM1_MSB);
			break;

			// Low alarm registers LSB for sensor 1
		case 0x35:
			rv = readEEPROM(EEPROM_LOW_ALARM1_LSB);
			break;

			// Low alarm registers MSB for sensor 2
		case 0x36:
			rv = readEEPROM(EEPROM_LOW_ALARM2_MSB);
			break;

			// Low alarm registers LSB for sensor 2
		case 0x37:
			rv = readEEPROM(EEPROM_LOW_ALARM2_LSB);
			break;

			// Low alarm registers MSB for sensor 3
		case 0x38:
			rv = readEEPROM(EEPROM_LOW_ALARM3_MSB);
			break;

			// Low alarm registers LSB for sensor 3
		case 0x39:
			rv = readEEPROM(EEPROM_LOW_ALARM3_LSB);
			break;

			// Low alarm registers MSB for sensor 4
		case 0x3A:
			rv = readEEPROM(EEPROM_LOW_ALARM4_MSB);
			break;

			// Low alarm registers LSB for sensor 4
		case 0x3B:
			rv = readEEPROM(EEPROM_LOW_ALARM4_LSB);
			break;

			// Low alarm registers MSB for sensor 5
		case 0x3C:
			rv = readEEPROM(EEPROM_LOW_ALARM5_MSB);
			break;

			// Low alarm registers LSB for sensor 5
		case 0x3D:
			rv = readEEPROM(EEPROM_LOW_ALARM5_LSB);
			break;



			// High alarm registers


			// High alarm registers MSB for sensor 0
		case 0x3E:
			rv = readEEPROM(EEPROM_HIGH_ALARM0_MSB);
			break;

			// High alarm registers LSB for sensor 0
		case 0x3F:
			rv = readEEPROM(EEPROM_HIGH_ALARM0_LSB);
			break;

			// High alarm registers MSB for sensor 1
		case 0x40:
			rv = readEEPROM(EEPROM_HIGH_ALARM1_MSB);
			break;

			// High alarm registers LSB for sensor 1
		case 0x41:
			rv = readEEPROM(EEPROM_HIGH_ALARM1_LSB);
			break;

			// High alarm registers MSB for sensor 2
		case 0x42:
			rv = readEEPROM(EEPROM_HIGH_ALARM2_MSB);
			break;

			// High alarm registers LSB for sensor 2
		case 0x43:
			rv = readEEPROM(EEPROM_HIGH_ALARM2_LSB);
			break;

			// High alarm registers MSB for sensor 3
		case 0x44:
			rv = readEEPROM(EEPROM_HIGH_ALARM3_MSB);
			break;

			// High alarm registers LSB for sensor 3
		case 0x45:
			rv = readEEPROM(EEPROM_HIGH_ALARM3_LSB);
			break;

			// High alarm registers MSB for sensor 4
		case 0x46:
			rv = readEEPROM(EEPROM_HIGH_ALARM4_MSB);
			break;

			// High alarm registers LSB for sensor 4
		case 0x47:
			rv = readEEPROM(EEPROM_HIGH_ALARM4_LSB);
			break;

			// High alarm registers MSB for sensor 5
		case 0x48:
			rv = readEEPROM(EEPROM_HIGH_ALARM5_MSB);
			break;

			// High alarm registers LSB for sensor 5
		case 0x49:
			rv = readEEPROM(EEPROM_HIGH_ALARM5_LSB);
			break;


			// Sensor zone information


			// Zone for sensor 0
		case 0x4A:
			rv = readEEPROM(EEPROM_SENSOR0_ZONE);
			break;

			// Subzone for senor 0
		case 0x4B:
			rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
			break;

			// Zone for sensor 1
		case 0x4C:
			rv = readEEPROM(EEPROM_SENSOR0_ZONE);
			break;

			// Subzone for senor 1
		case 0x4D:
			rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
			break;

			// Zone for sensor 2
		case 0x4E:
			rv = readEEPROM(EEPROM_SENSOR0_ZONE);
			break;

			// Subzone for senor 2
		case 0x4F:
			rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
			break;

			// Zone for sensor 3
		case 0x50:
			rv = readEEPROM(EEPROM_SENSOR0_ZONE);
			break;

			// Subzone for senor 3
		case 0x51:
			rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
			break;

			// Zone for sensor 4
		case 0x52:
			rv = readEEPROM(EEPROM_SENSOR0_ZONE);
			break;

			// Subzone for senor 4
		case 0x53:
			rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
			break;

			// Zone for sensor 5
		case 0x54:
			rv = readEEPROM(EEPROM_SENSOR0_ZONE);
			break;

			// Subzone for senor 5
		case 0x55:
			rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
			break;


			// Sensor absolute low


			// Absolute low registers MSB for sensor 0
		case 0x56:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW0_MSB);
			break;

			// Absolute low  registers LSB for sensor 0
		case 0x57:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW0_LSB);
			break;

			// Absolute low  registers MSB for sensor 1
		case 0x58:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW1_MSB);
			break;

			// Absolute low  registers LSB for sensor 1
		case 0x59:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW1_LSB);
			break;

			// Absolute low  registers MSB for sensor 2
		case 0x5A:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW2_MSB);
			break;

			// Absolute low  registers LSB for sensor 2
		case 0x5B:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW2_LSB);
			break;

			// Absolute low  registers MSB for sensor 3
		case 0x5C:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW3_MSB);
			break;

			// Absolute low  registers LSB for sensor 3
		case 0x5D:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW3_LSB);
			break;

			// Absolute low  registers MSB for sensor 4
		case 0x5E:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW4_MSB);
			break;

			// Absolute low  registers LSB for sensor 4
		case 0x5F:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW4_LSB);
			break;

			// Absolute low  registers MSB for sensor 5
		case 0x60:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW5_MSB);
			break;

			// Absolute low  registers LSB for sensor 5
		case 0x61:
			rv = readEEPROM(EEPROM_ABSOLUT_LOW5_LSB);
			break;


			// Sensor absolute high


			// Absolute high registers MSB for sensor 0
		case 0x62:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH0_MSB);
			break;

			// Absolute high  registers LSB for sensor 0
		case 0x63:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH0_LSB);
			break;

			// Absolute high  registers MSB for sensor 1
		case 0x64:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH1_MSB);
			break;

			// Absolute high  registers LSB for sensor 1
		case 0x65:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH1_LSB);
			break;

			// Absolute high  registers MSB for sensor 2
		case 0x66:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH2_MSB);
			break;

			// Absolute high  registers LSB for sensor 2
		case 0x67:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH2_LSB);
			break;

			// Absolute high  registers MSB for sensor 3
		case 0x68:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH3_MSB);
			break;

			// Absolute high  registers LSB for sensor 3
		case 0x69:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH3_LSB);
			break;

			// Absolute high  registers MSB for sensor 4
		case 0x6A:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH4_MSB);
			break;

			// Absolute high  registers LSB for sensor 4
		case 0x6B:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH4_LSB);
			break;

			// Absolute high  registers MSB for sensor 5
		case 0x6C:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH5_MSB);
			break;

			// Absolute high  registers LSB for sensor 5
		case 0x6D:
			rv = readEEPROM(EEPROM_ABSOLUT_HIGH5_LSB);
			break;


			// Hsyteresis



			// hysteresis for sensor 0
		case 0x6E:
			rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR0);
			break;

			// hysteresis for sensor 1
		case 0x6F:
			rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR1);
			break;

			// hysteresis for sensor 2
		case 0x70:
			rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR2);
			break;

			// hysteresis for sensor 3
		case 0x71:
			rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR3);
			break;

			// hysteresis for sensor 4
		case 0x72:
			rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR4);
			break;

			// hysteresis for sensor 5
		case 0x73:
			rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR5);
			break;

			// Calibration

			// Index for calibration
		case 0x74:
			rv = calibration_index;
			break;

			// Calibration values
		case 0x75:
			rv = readEEPROM(EEPROM_CALIBRATION_SENSOR0_MSB +
				calibration_index);
			break;

		case 0x76:
			rv = readEEPROM(EEPROM_CALIBRATED_VOLTAGE_MSB);
			break;

		case 0x77:
			rv = readEEPROM(EEPROM_CALIBRATED_VOLTAGE_LSB);
			break;

		default:
			rv = 0;
			break;
		}
	} else if (1 == vscp_page_select) {

		// SH Coeffecients
		if (reg <= 0x48) {
			rv = readEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_0 +
				reg - 0x48);
		}// Raw A/D values
        else if (reg <= 0x54) {
            // The byte order is differenet in registers
            uint8_t pos = reg - 0x54;
            if (0 == pos) pos = 11;
            rv = adc[reg - 0x54];
        }
    }

    return rv;

}

///////////////////////////////////////////////////////////////////////////////
// vscp_writeAppReg
//

uint8_t vscp_writeAppReg(unsigned char reg, unsigned char val)
{
    uint8_t rv;
    //int tmpval;
    //uint8_t checksum;
    //uint8_t unit = (0x03 & readEEPROM(EEPROM_CONTROLREG0));
    //uint16_t page = readEEPROM(EEPROM_CONTROLREG0) +
    //                    readEEPROM(EEPROM_CONTROLREG0)* 256;

    rv = ~val; // error return

    if (0 == vscp_page_select) {

        switch (reg) {

            // Zone
        case 0x00:
            writeEEPROM(EEPROM_ZONE, val);
            rv = readEEPROM(EEPROM_ZONE);
            break;

            // SubZone
        case 0x01:
            writeEEPROM(EEPROM_SUBZONE, val);
            rv = readEEPROM(EEPROM_SUBZONE);
            break;


            // Control registers


            // Control register sensor 0
        case 0x02:
            writeEEPROM(EEPROM_CONTROLREG0, val);
            rv = readEEPROM(EEPROM_CONTROLREG0);
            break;

            // Control register sensor 1
        case 0x03:
            writeEEPROM(EEPROM_CONTROLREG1, val);
            rv = readEEPROM(EEPROM_CONTROLREG1);
            break;

            // Control register sensor 2
        case 0x04:
            writeEEPROM(EEPROM_CONTROLREG2, val);
            rv = readEEPROM(EEPROM_CONTROLREG2);
            break;

            // Control register sensor 3
        case 0x05:
            writeEEPROM(EEPROM_CONTROLREG3, val);
            rv = readEEPROM(EEPROM_CONTROLREG3);
            break;

            // Control register sensor 4
        case 0x06:
            writeEEPROM(EEPROM_CONTROLREG4, val);
            rv = readEEPROM(EEPROM_CONTROLREG4);
            break;

            // Control register sensor 5
        case 0x07:
            writeEEPROM(EEPROM_CONTROLREG5, val);
            rv = readEEPROM(EEPROM_CONTROLREG5);
            break;


            // Report interval


            // Report interval register sensor 0
        case 0x14:
            writeEEPROM(EEPROM_REPORT_INTERVAL0, val);
            rv = readEEPROM(EEPROM_REPORT_INTERVAL0);
            break;

            // Report interval register sensor 1
        case 0x15:
            writeEEPROM(EEPROM_REPORT_INTERVAL1, val);
            rv = readEEPROM(EEPROM_REPORT_INTERVAL1);
            break;

            // Report interval register sensor 2
        case 0x16:
            writeEEPROM(EEPROM_REPORT_INTERVAL2, val);
            rv = readEEPROM(EEPROM_REPORT_INTERVAL2);
            break;

            // Report interval register sensor 3
        case 0x17:
            writeEEPROM(EEPROM_REPORT_INTERVAL3, val);
            rv = readEEPROM(EEPROM_REPORT_INTERVAL3);
            break;

            // Report interval register sensor 4
        case 0x18:
            writeEEPROM(EEPROM_REPORT_INTERVAL4, val);
            rv = readEEPROM(EEPROM_REPORT_INTERVAL4);
            break;

            // Report interval register sensor 5
        case 0x19:
            writeEEPROM(EEPROM_REPORT_INTERVAL5, val);
            rv = readEEPROM(EEPROM_REPORT_INTERVAL5);
            break;


            // B constant registers


            // B constant register MSB for sensor 0
        case 0x26:
            writeEEPROM(EEPROM_B_CONSTANT0_MSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT0_MSB);
            break;

            // B constant register LSB for sensor 0
        case 0x27:
            writeEEPROM(EEPROM_B_CONSTANT0_LSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT0_LSB);
            break;

            // B constant register MSB for sensor 1
        case 0x28:
            writeEEPROM(EEPROM_B_CONSTANT1_MSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT1_MSB);
            break;

            // B constant register LSB for sensor 1
        case 0x29:
            writeEEPROM(EEPROM_B_CONSTANT1_LSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT1_LSB);
            break;

            // B constant register MSB for sensor 2
        case 0x2A:
            writeEEPROM(EEPROM_B_CONSTANT2_MSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT2_MSB);
            break;

            // B constant register LSB for sensor 2
        case 0x2B:
            writeEEPROM(EEPROM_B_CONSTANT2_LSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT2_LSB);
            break;

            // B constant register MSB for sensor 3
        case 0x2C:
            writeEEPROM(EEPROM_B_CONSTANT3_MSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT3_MSB);
            break;

            // B constant register LSB for sensor 3
        case 0x2D:
            writeEEPROM(EEPROM_B_CONSTANT3_LSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT3_LSB);
            break;

            // B constant register MSB for sensor 4
        case 0x2E:
            writeEEPROM(EEPROM_B_CONSTANT4_MSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT4_MSB);
            break;

            // B constant register LSB for sensor 4
        case 0x2F:
            writeEEPROM(EEPROM_B_CONSTANT4_LSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT4_LSB);
            break;

            // B constant register MSB for sensor 5
        case 0x30:
            writeEEPROM(EEPROM_B_CONSTANT5_MSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT5_MSB);
            break;

            // B constant register LSB for sensor 5
        case 0x31:
            writeEEPROM(EEPROM_B_CONSTANT5_LSB, val);
            rv = readEEPROM(EEPROM_B_CONSTANT5_LSB);
            break;


            // Low alarm registers


            // Low alarm point register MSB for sensor 0
        case 0x32:
            writeEEPROM(EEPROM_LOW_ALARM0_MSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM0_MSB);
            break;

            // Low alarm point register LSB for sensor 0
        case 0x33:
            writeEEPROM(EEPROM_LOW_ALARM0_LSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM0_LSB);
            break;

            // Low alarm point register MSB for sensor 1
        case 0x34:
            writeEEPROM(EEPROM_LOW_ALARM1_MSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM1_MSB);
            break;

            // Low alarm point register LSB for sensor 1
        case 0x35:
            writeEEPROM(EEPROM_LOW_ALARM1_LSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM1_LSB);
            break;

            // Low alarm point register MSB for sensor 2
        case 0x36:
            writeEEPROM(EEPROM_LOW_ALARM2_MSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM2_MSB);
            break;

            // Low alarm point register LSB for sensor 2
        case 0x37:
            writeEEPROM(EEPROM_LOW_ALARM2_LSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM2_LSB);
            break;

            // Low alarm point register MSB for sensor 3
        case 0x38:
            writeEEPROM(EEPROM_LOW_ALARM3_MSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM3_MSB);
            break;

            // Low alarm point register LSB for sensor 3
        case 0x39:
            writeEEPROM(EEPROM_LOW_ALARM3_LSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM3_LSB);
            break;

            // Low alarm point register MSB for sensor 4
        case 0x3A:
            writeEEPROM(EEPROM_LOW_ALARM4_MSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM4_MSB);
            break;

            // Low alarm point register LSB for sensor 4
        case 0x3B:
            writeEEPROM(EEPROM_LOW_ALARM4_LSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM4_LSB);
            break;

            // Low alarm point register MSB for sensor 5
        case 0x3C:
            writeEEPROM(EEPROM_LOW_ALARM5_MSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM5_MSB);
            break;

            // Low alarm point register LSB for sensor 5
        case 0x3D:
            writeEEPROM(EEPROM_LOW_ALARM5_LSB, val);
            rv = readEEPROM(EEPROM_LOW_ALARM5_LSB);
            break;


            // High alarm registers


            // High alarm point register MSB for sensor 0
        case 0x3E:
            writeEEPROM(EEPROM_HIGH_ALARM0_MSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM0_MSB);
            break;

            // High alarm point register LSB for sensor 0
        case 0x3F:
            writeEEPROM(EEPROM_HIGH_ALARM0_LSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM0_LSB);
            break;

            // High alarm point register MSB for sensor 1
        case 0x40:
            writeEEPROM(EEPROM_HIGH_ALARM1_MSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM1_MSB);
            break;

            // High alarm point register LSB for sensor 1
        case 0x41:
            writeEEPROM(EEPROM_HIGH_ALARM1_LSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM1_LSB);
            break;

            // High alarm point register MSB for sensor 2
        case 0x42:
            writeEEPROM(EEPROM_HIGH_ALARM2_MSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM2_MSB);
            break;

            // High alarm point register LSB for sensor 2
        case 0x43:
            writeEEPROM(EEPROM_HIGH_ALARM2_LSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM2_LSB);
            break;

            // High alarm point register MSB for sensor 3
        case 0x44:
            writeEEPROM(EEPROM_HIGH_ALARM3_MSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM3_MSB);
            break;

            // High alarm point register LSB for sensor 3
        case 0x45:
            writeEEPROM(EEPROM_HIGH_ALARM3_LSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM3_LSB);
            break;

            // High alarm point register MSB for sensor 4
        case 0x46:
            writeEEPROM(EEPROM_HIGH_ALARM4_MSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM4_MSB);
            break;

            // High alarm point register LSB for sensor 4
        case 0x47:
            writeEEPROM(EEPROM_HIGH_ALARM4_LSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM4_LSB);
            break;

            // High alarm point register MSB for sensor 5
        case 0x48:
            writeEEPROM(EEPROM_HIGH_ALARM5_MSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM5_MSB);
            break;

            // High alarm point register LSB for sensor 5
        case 0x49:
            writeEEPROM(EEPROM_HIGH_ALARM5_LSB, val);
            rv = readEEPROM(EEPROM_HIGH_ALARM5_LSB);
            break;


            // Sensor zone information registers


            // Zone for sensor 0
        case 0x4A:
            writeEEPROM(EEPROM_SENSOR0_ZONE, val);
            rv = readEEPROM(EEPROM_SENSOR0_ZONE);
            break;

            // Subzone for sensor 0
        case 0x4B:
            writeEEPROM(EEPROM_SENSOR0_SUBZONE, val);
            rv = readEEPROM(EEPROM_SENSOR0_SUBZONE);
            break;

            // Zone for sensor 1
        case 0x4C:
            writeEEPROM(EEPROM_SENSOR1_ZONE, val);
            rv = readEEPROM(EEPROM_SENSOR1_ZONE);
            break;

            // Subzone for sensor 1
        case 0x4D:
            writeEEPROM(EEPROM_SENSOR1_SUBZONE, val);
            rv = readEEPROM(EEPROM_SENSOR1_SUBZONE);
            break;

            // Zone for sensor 2
        case 0x4E:
            writeEEPROM(EEPROM_SENSOR2_ZONE, val);
            rv = readEEPROM(EEPROM_SENSOR2_ZONE);
            break;

            // Subzone for sensor 2
        case 0x4F:
            writeEEPROM(EEPROM_SENSOR2_SUBZONE, val);
            rv = readEEPROM(EEPROM_SENSOR2_SUBZONE);
            break;

            // Zone for sensor 3
        case 0x50:
            writeEEPROM(EEPROM_SENSOR3_ZONE, val);
            rv = readEEPROM(EEPROM_SENSOR3_ZONE);
            break;

            // Subzone for sensor 3
        case 0x51:
            writeEEPROM(EEPROM_SENSOR3_SUBZONE, val);
            rv = readEEPROM(EEPROM_SENSOR3_SUBZONE);
            break;

            // Zone for sensor 4
        case 0x52:
            writeEEPROM(EEPROM_SENSOR4_ZONE, val);
            rv = readEEPROM(EEPROM_SENSOR4_ZONE);
            break;

            // Subzone for sensor 4
        case 0x53:
            writeEEPROM(EEPROM_SENSOR4_SUBZONE, val);
            rv = readEEPROM(EEPROM_SENSOR4_SUBZONE);
            break;

            // Zone for sensor 5
        case 0x54:
            writeEEPROM(EEPROM_SENSOR5_ZONE, val);
            rv = readEEPROM(EEPROM_SENSOR5_ZONE);
            break;

            // Subzone for sensor 5
        case 0x55:
            writeEEPROM(EEPROM_SENSOR5_SUBZONE, val);
            rv = readEEPROM(EEPROM_SENSOR5_SUBZONE);
            break;


            // Sensor absolute low temperature registers


            // Sensor absolute low temperature register MSB for sensor 0
        case 0x56:
            writeEEPROM(EEPROM_ABSOLUT_LOW0_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW0_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW0_MSB);
            break;

            // Sensor absolute low temperature register LSB for sensor 0
        case 0x57:
            writeEEPROM(EEPROM_ABSOLUT_LOW0_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW0_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW0_LSB);
            break;

            // Sensor absolute low temperature register MSB for sensor 1
        case 0x58:
            writeEEPROM(EEPROM_ABSOLUT_LOW1_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW1_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW1_MSB);
            break;

            // Sensor absolute low temperature register LSB for sensor 1
        case 0x59:
            writeEEPROM(EEPROM_ABSOLUT_LOW1_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW1_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW1_LSB);
            break;

            // Sensor absolute low temperature register MSB for sensor 2
        case 0x5A:
            writeEEPROM(EEPROM_ABSOLUT_LOW2_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW2_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW2_MSB);
            break;

            // Sensor absolute low temperature register LSB for sensor 2
        case 0x5B:
            writeEEPROM(EEPROM_ABSOLUT_LOW2_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW2_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW2_LSB);
            break;

            // Sensor absolute low temperature register MSB for sensor 3
        case 0x5C:
            writeEEPROM(EEPROM_ABSOLUT_LOW3_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW3_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW3_MSB);
            break;

            // Sensor absolute low temperature register LSB for sensor 3
        case 0x5D:
            writeEEPROM(EEPROM_ABSOLUT_LOW3_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW3_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW3_LSB);
            break;

            // Sensor absolute low temperature register MSB for sensor 4
        case 0x5E:
            writeEEPROM(EEPROM_ABSOLUT_LOW4_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW4_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW4_MSB);
            break;

            // Sensor absolute low temperature register LSB for sensor 4
        case 0x5F:
            writeEEPROM(EEPROM_ABSOLUT_LOW4_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW4_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW4_LSB);
            break;

            // Sensor absolute low temperature register MSB for sensor 5
        case 0x60:
            writeEEPROM(EEPROM_ABSOLUT_LOW5_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW5_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW5_MSB);
            break;

            // Sensor absolute low temperature register LSB for sensor 5
        case 0x61:
            writeEEPROM(EEPROM_ABSOLUT_LOW5_MSB, DEFAULT_LOW_MSB);
            writeEEPROM(EEPROM_ABSOLUT_LOW5_LSB, DEFAULT_LOW_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_LOW5_LSB);
            break;


            // Sensor absolute high temperature registers


            // Sensor absolute high temperature register MSB for sensor 0
        case 0x62:
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH0_MSB);
            break;

            // Sensor absolute high temperature register LSB for sensor 0
        case 0x63:
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH0_LSB);
            break;

            // Sensor absolute high temperature register MSB for sensor 1
        case 0x64:
            writeEEPROM(EEPROM_ABSOLUT_HIGH1_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH1_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH1_MSB);
            break;

            // Sensor absolute high temperature register LSB for sensor 1
        case 0x65:
            writeEEPROM(EEPROM_ABSOLUT_HIGH1_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH1_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH1_LSB);
            break;

            // Sensor absolute high temperature register MSB for sensor 2
        case 0x66:
            writeEEPROM(EEPROM_ABSOLUT_HIGH2_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH2_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH2_MSB);
            break;

            // Sensor absolute high temperature register LSB for sensor 2
        case 0x67:
            writeEEPROM(EEPROM_ABSOLUT_HIGH2_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH2_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH2_LSB);
            break;

            // Sensor absolute high temperature register MSB for sensor 3
        case 0x68:
            writeEEPROM(EEPROM_ABSOLUT_HIGH3_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH3_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH3_MSB);
            break;

            // Sensor absolute high temperature register LSB for sensor 3
        case 0x69:
            writeEEPROM(EEPROM_ABSOLUT_HIGH3_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH3_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH3_LSB);
            break;

            // Sensor absolute high temperature register MSB for sensor 4
        case 0x6A:
            writeEEPROM(EEPROM_ABSOLUT_HIGH4_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH4_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH4_MSB);
            break;

            // Sensor absolute high temperature register LSB for sensor 4
        case 0x6B:
            writeEEPROM(EEPROM_ABSOLUT_HIGH4_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH4_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH4_LSB);
            break;

            // Sensor absolute high temperature register MSB for sensor 5
        case 0x6C:
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH5_MSB);
            break;

            // Sensor absolute high temperature register LSB for sensor 5
        case 0x6D:
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_MSB);
            writeEEPROM(EEPROM_ABSOLUT_HIGH0_MSB, DEFAULT_HIGH_LSB);
            rv = readEEPROM(EEPROM_ABSOLUT_HIGH5_LSB);
            break;


            // Sensor hysteresis


            // Hysteresis for sensor 0
        case 0x6E:
            writeEEPROM(EEPROM_HYSTERESIS_SENSOR0, val);
            rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR0);
            break;

            // Hysteresis for sensor 1
        case 0x6F:
            writeEEPROM(EEPROM_HYSTERESIS_SENSOR1, val);
            rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR1);
            break;

            // Hysteresis for sensor 2
        case 0x70:
            writeEEPROM(EEPROM_HYSTERESIS_SENSOR2, val);
            rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR2);

            // Hysteresis for sensor 3
        case 0x71:
            writeEEPROM(EEPROM_HYSTERESIS_SENSOR3, val);
            rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR3);
            break;

            // Hysteresis for sensor 4
        case 0x72:
            writeEEPROM(EEPROM_HYSTERESIS_SENSOR4, val);
            rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR4);
            break;

            // Hysteresis for sensor 5
        case 0x73:
            writeEEPROM(EEPROM_HYSTERESIS_SENSOR5, val);
            rv = readEEPROM(EEPROM_HYSTERESIS_SENSOR5);
            break;

            // Calibration

            // Index for calibration
        case 0x74:
            if (val < 12) {
                rv = calibration_index = val;
            }
            break;

            // Calibration values
        case 0x75:
            writeEEPROM(EEPROM_CALIBRATION_SENSOR0_MSB +
                    calibration_index, val);
            rv = readEEPROM(EEPROM_CALIBRATION_SENSOR0_MSB +
                    calibration_index);
            break;

        case 0x76:
            writeEEPROM(EEPROM_CALIBRATED_VOLTAGE_MSB, val);
            rv = readEEPROM(EEPROM_CALIBRATED_VOLTAGE_MSB);
            break;

        case 0x77:
            writeEEPROM(EEPROM_CALIBRATED_VOLTAGE_LSB, val);
            rv = readEEPROM(EEPROM_CALIBRATED_VOLTAGE_LSB);
            break;

        default:
            rv = ~val; // error return
            break;
        }
    } else if (1 == vscp_page_select) {
        // Coeffecients
        if (reg <= 0x48) {
            writeEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_0 + reg, val);
            rv = readEEPROM(EEPROM_COEFFICIENT_A_SENSOR0_0 + reg);
            writeCoeffs2Ram();
        }
    }

    return rv;
}


///////////////////////////////////////////////////////////////////////////////
//                        VSCP Required Methods
//////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// Get Major version number for this hardware module
//

unsigned char vscp_getMajorVersion()
{
    return FIRMWARE_MAJOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Minor version number for this hardware module
//

unsigned char vscp_getMinorVersion()
{
    return FIRMWARE_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// Get Subminor version number for this hardware module
//

unsigned char vscp_getSubMinorVersion()
{
    return FIRMWARE_SUB_MINOR_VERSION;
}

///////////////////////////////////////////////////////////////////////////////
// getVSCP_GUID
//
// Get GUID from EEPROM
//

uint8_t vscp_getGUID(uint8_t idx)
{
    return readEEPROM(VSCP_EEPROM_REG_GUID + idx);
}


///////////////////////////////////////////////////////////////////////////////
// getDeviceURL
//
// Get device URL from EEPROM
//

uint8_t vscp_getMDF_URL(uint8_t idx)
{
    return vscp_deviceURL[ idx ];
}

///////////////////////////////////////////////////////////////////////////////
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getUserID(uint8_t idx)
{
    return readEEPROM(VSCP_EEPROM_REG_USERID + idx);
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPUserID
//

void vscp_setUserID(uint8_t idx, uint8_t data)
{
    writeEEPROM(idx + VSCP_EEPROM_REG_USERID, data);
}

///////////////////////////////////////////////////////////////////////////////
// getVSCPManufacturerId
// 
// Get Manufacturer id and subid from EEPROM
//

uint8_t vscp_getManufacturerId(uint8_t idx)
{
    return readEEPROM(VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx);
}

///////////////////////////////////////////////////////////////////////////////
// getVSCPManufacturerId
// 
// Get Manufacturer id and subid from EEPROM
//

void vscp_setManufacturerId(uint8_t idx, uint8_t data)
{
    writeEEPROM(VSCP_EEPROM_REG_MANUFACTUR_ID0 + idx, data);
}

///////////////////////////////////////////////////////////////////////////////
// Get the bootloader algorithm code
//

uint8_t vscp_getBootLoaderAlgorithm(void)
{
    return VSCP_BOOTLOADER_PIC1;
}

///////////////////////////////////////////////////////////////////////////////
// Get the buffer size
//

uint8_t vscp_getBufferSize(void)
{
    return 8; // Standard CAN frame
}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getMatrixInfo
//

void vscp_getMatrixInfo(char *pData)
{
    uint8_t i;

    // We don't implement a DM for this module
    // So we just set the data to zero
    for (i = 0; i < 8; i++) {
        pData[ i ] = 0;
    }

}

///////////////////////////////////////////////////////////////////////////////
//  vscp_getEmbeddedMdfInfo
//

void vscp_getEmbeddedMdfInfo(void)
{
    // No embedded DM so we respond with info about that

    vscp_omsg.priority = VSCP_PRIORITY_NORMAL;
    vscp_omsg.flags = VSCP_VALID_MSG + 3;
    vscp_omsg.class = VSCP_CLASS1_PROTOCOL;
    vscp_omsg.type = VSCP_TYPE_PROTOCOL_RW_RESPONSE;

    vscp_omsg.data[ 0 ] = 0;
    vscp_omsg.data[ 1 ] = 0;
    vscp_omsg.data[ 2 ] = 0;

    // send the event
    vscp_sendEvent();
}


///////////////////////////////////////////////////////////////////////////////
// vscp_getRegisterPagesUsed
//

uint8_t vscp_getRegisterPagesUsed(void)
{
    return 1;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getZone
//

uint8_t vscp_getZone(void)
{
    return readEEPROM(EEPROM_ZONE);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getSubzone
//

uint8_t vscp_getSubzone(void)
{
    return readEEPROM(EEPROM_SUBZONE);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_goBootloaderMode
//

void vscp_goBootloaderMode(void)
{
    // OK, We should enter boot loader mode
    // 	First, activate bootloader mode
    writeEEPROM(VSCP_EEPROM_BOOTLOADER_FLAG, VSCP_BOOT_FLAG);

    //_asm goto _startup reset _endasm
    //_asm reset _endasm
    asm("reset");
}

///////////////////////////////////////////////////////////////////////////////
//  getNickname
//

uint8_t vscp_readNicknamePermanent(void)
{
    return readEEPROM(VSCP_EEPROM_NICKNAME);
}

///////////////////////////////////////////////////////////////////////////////
//  setNickname
//

void vscp_writeNicknamePermanent(uint8_t nickname)
{
    writeEEPROM(VSCP_EEPROM_NICKNAME, nickname);
}

///////////////////////////////////////////////////////////////////////////////
//  getSegmentCRC
//

uint8_t vscp_getSegmentCRC(void)
{
    return readEEPROM(VSCP_EEPROM_SEGMENT_CRC);
}

///////////////////////////////////////////////////////////////////////////////
//  setSegmentCRC
//

void vscp_setSegmentCRC(uint8_t crc)
{
    writeEEPROM(VSCP_EEPROM_SEGMENT_CRC, crc);
}

///////////////////////////////////////////////////////////////////////////////
//  setVSCPControlByte
//

void vscp_setControlByte(uint8_t ctrl)
{
    writeEEPROM(VSCP_EEPROM_CONTROL, ctrl);
}


///////////////////////////////////////////////////////////////////////////////
//  getVSCPControlByte
//

uint8_t vscp_getControlByte(void)
{
    return readEEPROM(VSCP_EEPROM_CONTROL);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_getFamilyCode
//

uint32_t vscp_getFamilyCode()
{
    return 0;
}


///////////////////////////////////////////////////////////////////////////////
// vscp_getFamilyType
//

uint32_t vscp_getFamilyType()
{
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_restoreDefaults
//

void vscp_restoreDefaults()
{
    init_app_eeprom();
    init_app_ram();
}

///////////////////////////////////////////////////////////////////////////////
// sendVSCPFrame
//

int8_t sendVSCPFrame(uint16_t vscpclass,
        uint8_t vscptype,
        uint8_t nodeid,
        uint8_t priority,
        uint8_t size,
        uint8_t *pData)
{
    uint32_t id = ((uint32_t) priority << 26) |
            ((uint32_t) vscpclass << 16) |
            ((uint32_t) vscptype << 8) |
            nodeid; // nodeaddress (our address)

    if (!sendCANFrame(id, size, pData)) {
        // Failed to send message
        vscp_errorcnt++;
        return FALSE;
    }

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// getVSCPFrame
//

int8_t getVSCPFrame(uint16_t *pvscpclass,
        uint8_t *pvscptype,
        uint8_t *pNodeId,
        uint8_t *pPriority,
        uint8_t *pSize,
        uint8_t *pData)
{
    uint32_t id;

    if (!getCANFrame(&id, pSize, pData)) {
        return FALSE;
    }

    *pNodeId = id & 0x0ff;
    *pvscptype = (id >> 8) & 0xff;
    *pvscpclass = (id >> 16) & 0x1ff;
    *pPriority = (uint16_t) (0x07 & (id >> 26));

    return TRUE;
}


///////////////////////////////////////////////////////////////////////////////
// sendCANFrame
//

int8_t sendCANFrame(uint32_t id, uint8_t dlc, uint8_t *pdata)
{
    if (!ECANSendMessage(id, pdata, dlc, ECAN_TX_XTD_FRAME)) {
        // Failed to send event
        return FALSE;
    }

    vscp_omsg.flags = 0;
    return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// getCANFrame
//

int8_t getCANFrame(uint32_t *pid, uint8_t *pdlc, uint8_t *pdata)
{
    ECAN_RX_MSG_FLAGS flags;

    // Dont read in new message if there already is a message
    // in the input buffer
    if (vscp_imsg.flags & VSCP_VALID_MSG) return FALSE;

    if (ECANReceiveMessage((unsigned long *) pid, (BYTE*) pdata, (BYTE*) pdlc, &flags)) {

        // RTR not interesting
        if (flags & ECAN_RX_RTR_FRAME) return FALSE;

        // Must be extended frame
        if (!(flags & ECAN_RX_XTD_FRAME)) return FALSE;

        return TRUE;
    }

    return FALSE;
}
