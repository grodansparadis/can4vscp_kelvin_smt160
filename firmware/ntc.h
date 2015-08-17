/*
 * Author: AKHE <akhe@grodansparadis.com>
 * Copyright (C) Grodans Paradis AB
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

#ifndef NTC_H
#define NTC_H

#include <inttypes.h>
#include "main.h"

// Defines the number of samples that is taken for each sensor
// An avarage temperature is calulated of this number of samples.
#define NUMBER_OF_TEMP_SERIES   80

//
// Defaults
//

// Default value for control register
// bit 0,1 - 01 = Celsius
// Bit 2 - 0 Reserved bit
// Bit 3 = 0 Low alarm diabled
// Bit 4 = 0 High alarm disabled
// Bit 5 = 0 Alarm is sent
// Bit 6 = 0 Normal TurnOn/Off
// Bit 7 = 0 Non continues alarm
// 
#define DEFAULT_CONTROL_REG                 0b00000001

#define MASK_CONTROL_UNIT                   0x03
#define MASK_CONTROL_LOW_ALARM              0x08
#define MASK_CONTROL_HIGH_ALARM             0x10
#define MASK_CONTROL_TURNX                  0x20
#define MASK_CONTROL_TURNX_INVERT           0x40
#define MASK_CONTROL_CONTINUOUS             0x80

// Default report interval fro sensor 0
#define DEFAULT_REPORT_INTERVAL_SENSOR0     10

// Default report interval for sensor 1-5
#define DEFAULT_REPORT_INTERVAL             0

// Default B constant for sensor 0
// Digikey 90-2456-1-ND
#define DEFAULT_B_CONSTANT_SENSOR0_MSB      0x0F
#define DEFAULT_B_CONSTANT_SENSOR0_LSB      0x68

// Default B constant for sensor 1-5
// Digikey 490-4653-ND  - End of life and not available anymore
#define DEFAULT_B_CONSTANT_MSB              0x0D
#define DEFAULT_B_CONSTANT_LSB              0x7F

// Elfa 60-279-32  - RH16-3H103FB

// Default coefficients for sensor 0
#define DEFAULT_INTERNAL_COEFFICIENT_A1     4.524024725919526e-004
#define DEFAULT_INTERNAL_COEFFICIENT_A2     3.934722516618191e-004
#define DEFAULT_INTERNAL_COEFFICIENT_A3     -7.642331765196044e-006
#define DEFAULT_INTERNAL_COEFFICIENT_A4     4.048572707661904e-007

// Default coefficients for sensor 1-5
#define DEFAULT_EXTERNAL_COEFFICIENT_A1     4.524024725919526e-004
#define DEFAULT_EXTERNAL_COEFFICIENT_A2     3.934722516618191e-004
#define DEFAULT_EXTERNAL_COEFFICIENT_A3     -7.642331765196044e-006
#define DEFAULT_EXTERNAL_COEFFICIENT_A4     4.048572707661904e-007

// Default value for low alarm point -327.68 degrees celsius
// A point that will not be reached
#define DEFAULT_LOW_ALARM_MSB               0x80
#define DEFAULT_LOW_ALARM_LSB               0xff

// Default value for high alarm point +327.67 degrees celsius
// A point that will not be reached
#define DEFAULT_HIGH_ALARM_MSB              0x7f
#define DEFAULT_HIGH_ALARM_LSB              0xff

// Default sensor zon/subzone information
#define DEFAULT_SENSOR_ZONE                 0
#define DEFAULT_SENSOR_SUBZONE              0

// Default absolut low value
// Set to +327.67 degrees celsius
#define DEFAULT_LOW_MSB                     0x7f
#define DEFAULT_LOW_LSB                     0xff

// Default absolut high value
// Set to -327.68 degrees celsius
#define DEFAULT_HIGH_MSB                    0x80
#define DEFAULT_HIGH_LSB                    0x00

// Default hysteresis
#define DEFAULT_HYSTERESIS                  2

//
// Error
//

#define ERROR_GENERAL               0x00
#define ERROR_SENSOR                0x01
#define ERROR_MODULE                0x02

//
// Temp. unit
//

#define TEMP_UNIT_KELVIN            0
#define TEMP_UNIT_CELSIUS           1
#define TEMP_UNIT_FAHRENHEIT        2


//
// Module alarm bits
//
#define MODULE_LOW_ALARM            1
#define MODULE_HIGH_ALARM           2

//
// Configuration bits
//
// Default value for control register
// bit 0,1 - Temperature
// Bit 2 - Reserved bit
// Bit 3 = Low alarm eanble
// Bit 4 = High alarm enable
// Bit 5 = Enable TurnOn/Off
// Bit 6 = Enable invert TurnOn/TurnOff
// Bit 7 = Continues alarm
#define CONFIG_ENABLE_LOW_ALARM         (1<<3)
#define CONFIG_ENABLE_HIGH_ALARM        (1<<4)
#define CONFIG_ENABLE_TURNX             (1<<5)
#define CONFIG_ENABLE_TURNON_INVERT     (1<<6)
#define CONFIG_ENABLE_CONTINOUS_EVENTS  (1<<7)

// EEPROM registers for module persistent data

#define EEPROM_ZONE                 0x41	// Zone node belongs to
#define EEPROM_SUBZONE              0x42	// Subzone node belongs to

#define EEPROM_CONTROLREG0          0x43
#define EEPROM_CONTROLREG1          0x44
#define EEPROM_CONTROLREG2          0x45
#define EEPROM_CONTROLREG3          0x46
#define EEPROM_CONTROLREG4          0x47
#define EEPROM_CONTROLREG5          0x48

#define EEPROM_REPORT_INTERVAL0     0x49
#define EEPROM_REPORT_INTERVAL1     0x4A
#define EEPROM_REPORT_INTERVAL2     0x4B
#define EEPROM_REPORT_INTERVAL3     0x4C
#define EEPROM_REPORT_INTERVAL4     0x4D
#define EEPROM_REPORT_INTERVAL5     0x4E

// B Constants

#define EEPROM_B_CONSTANT0_MSB      0x4F
#define EEPROM_B_CONSTANT0_LSB      0x50

#define EEPROM_B_CONSTANT1_MSB      0x51
#define EEPROM_B_CONSTANT1_LSB      0x52

#define EEPROM_B_CONSTANT2_MSB      0x53
#define EEPROM_B_CONSTANT2_LSB      0x54

#define EEPROM_B_CONSTANT3_MSB      0x55
#define EEPROM_B_CONSTANT3_LSB      0x56

#define EEPROM_B_CONSTANT4_MSB      0x57
#define EEPROM_B_CONSTANT4_LSB      0x58

#define EEPROM_B_CONSTANT5_MSB      0x59
#define EEPROM_B_CONSTANT5_LSB      0x5A

// Low alarms

#define EEPROM_LOW_ALARM0_MSB       0x5B
#define EEPROM_LOW_ALARM0_LSB       0x5C

#define EEPROM_LOW_ALARM1_MSB       0x5D
#define EEPROM_LOW_ALARM1_LSB       0x5E

#define EEPROM_LOW_ALARM2_MSB       0x5F
#define EEPROM_LOW_ALARM2_LSB       0x60

#define EEPROM_LOW_ALARM3_MSB       0x61
#define EEPROM_LOW_ALARM3_LSB       0x62

#define EEPROM_LOW_ALARM4_MSB       0x63
#define EEPROM_LOW_ALARM4_LSB       0x64

#define EEPROM_LOW_ALARM5_MSB       0x65
#define EEPROM_LOW_ALARM5_LSB       0x66

// High alarms

#define EEPROM_HIGH_ALARM0_MSB      0x67
#define EEPROM_HIGH_ALARM0_LSB      0x68

#define EEPROM_HIGH_ALARM1_MSB      0x69
#define EEPROM_HIGH_ALARM1_LSB      0x6A

#define EEPROM_HIGH_ALARM2_MSB      0x6B
#define EEPROM_HIGH_ALARM2_LSB      0x6C

#define EEPROM_HIGH_ALARM3_MSB      0x6D
#define EEPROM_HIGH_ALARM3_LSB      0x6E

#define EEPROM_HIGH_ALARM4_MSB      0x6F
#define EEPROM_HIGH_ALARM4_LSB      0x70

#define EEPROM_HIGH_ALARM5_MSB      0x71
#define EEPROM_HIGH_ALARM5_LSB      0x72

// Sensor zone/subzone information

#define EEPROM_SENSOR0_ZONE         0x73
#define EEPROM_SENSOR0_SUBZONE      0x74

#define EEPROM_SENSOR1_ZONE         0x75
#define EEPROM_SENSOR1_SUBZONE      0x76

#define EEPROM_SENSOR2_ZONE         0x77
#define EEPROM_SENSOR2_SUBZONE      0x78

#define EEPROM_SENSOR3_ZONE         0x79
#define EEPROM_SENSOR3_SUBZONE      0x7A

#define EEPROM_SENSOR4_ZONE         0x7B
#define EEPROM_SENSOR4_SUBZONE      0x7C

#define EEPROM_SENSOR5_ZONE         0x7D
#define EEPROM_SENSOR5_SUBZONE      0x7E

// Absolut low temperatures

#define EEPROM_ABSOLUT_LOW0_MSB     0x7F
#define EEPROM_ABSOLUT_LOW0_LSB     0x80

#define EEPROM_ABSOLUT_LOW1_MSB     0x81
#define EEPROM_ABSOLUT_LOW1_LSB     0x82

#define EEPROM_ABSOLUT_LOW2_MSB     0x83
#define EEPROM_ABSOLUT_LOW2_LSB     0x84

#define EEPROM_ABSOLUT_LOW3_MSB     0x85
#define EEPROM_ABSOLUT_LOW3_LSB     0x86

#define EEPROM_ABSOLUT_LOW4_MSB     0x87
#define EEPROM_ABSOLUT_LOW4_LSB     0x88

#define EEPROM_ABSOLUT_LOW5_MSB     0x89
#define EEPROM_ABSOLUT_LOW5_LSB     0x8A

// Absolut high temperatures

#define EEPROM_ABSOLUT_HIGH0_MSB    0x8B
#define EEPROM_ABSOLUT_HIGH0_LSB    0x8C

#define EEPROM_ABSOLUT_HIGH1_MSB    0x8D
#define EEPROM_ABSOLUT_HIGH1_LSB    0x8E

#define EEPROM_ABSOLUT_HIGH2_MSB    0x8F
#define EEPROM_ABSOLUT_HIGH2_LSB    0x90

#define EEPROM_ABSOLUT_HIGH3_MSB    0x91
#define EEPROM_ABSOLUT_HIGH3_LSB    0x92

#define EEPROM_ABSOLUT_HIGH4_MSB    0x93
#define EEPROM_ABSOLUT_HIGH4_LSB    0x94

#define EEPROM_ABSOLUT_HIGH5_MSB    0x95
#define EEPROM_ABSOLUT_HIGH5_LSB    0x96

// Sensor hysteresis

#define EEPROM_HYSTERESIS_SENSOR0   0x97
#define EEPROM_HYSTERESIS_SENSOR1   0x98
#define EEPROM_HYSTERESIS_SENSOR2   0x99
#define EEPROM_HYSTERESIS_SENSOR3   0x9A
#define EEPROM_HYSTERESIS_SENSOR4   0x9B
#define EEPROM_HYSTERESIS_SENSOR5   0x9C

#define EEPROM_CALIBRATION_SENSOR0_MSB      0x9E
#define EEPROM_CALIBRATION_SENSOR0_LSB      0x9F
#define EEPROM_CALIBRATION_SENSOR1_MSB      0xA0
#define EEPROM_CALIBRATION_SENSOR1_LSB      0xA1
#define EEPROM_CALIBRATION_SENSOR2_MSB      0xA2
#define EEPROM_CALIBRATION_SENSOR2_LSB      0xA3
#define EEPROM_CALIBRATION_SENSOR3_MSB      0xA4
#define EEPROM_CALIBRATION_SENSOR3_LSB      0xA5
#define EEPROM_CALIBRATION_SENSOR4_MSB      0xA6
#define EEPROM_CALIBRATION_SENSOR4_LSB      0xA7
#define EEPROM_CALIBRATION_SENSOR5_MSB      0xA8
#define EEPROM_CALIBRATION_SENSOR5_LSB      0xA9

#define EEPROM_COEFFICIENT_A_SENSOR0_0      0xAA
#define EEPROM_COEFFICIENT_A_SENSOR0_1      0xAB
#define EEPROM_COEFFICIENT_A_SENSOR0_2      0xAC
#define EEPROM_COEFFICIENT_A_SENSOR0_3      0xAD

#define EEPROM_COEFFICIENT_B_SENSOR0_0      0xAE
#define EEPROM_COEFFICIENT_B_SENSOR0_1      0xAF
#define EEPROM_COEFFICIENT_B_SENSOR0_2      0xB0
#define EEPROM_COEFFICIENT_B_SENSOR0_3      0xB1

#define EEPROM_COEFFICIENT_C_SENSOR0_0      0xB2
#define EEPROM_COEFFICIENT_C_SENSOR0_1      0xB3
#define EEPROM_COEFFICIENT_C_SENSOR0_2      0xB4
#define EEPROM_COEFFICIENT_C_SENSOR0_3      0xB5

#define EEPROM_COEFFICIENT_A_SENSOR1_0      0xB6
#define EEPROM_COEFFICIENT_A_SENSOR1_1      0xB7
#define EEPROM_COEFFICIENT_A_SENSOR1_2      0xB8
#define EEPROM_COEFFICIENT_A_SENSOR1_3      0xB9

#define EEPROM_COEFFICIENT_B_SENSOR1_0      0xBA
#define EEPROM_COEFFICIENT_B_SENSOR1_1      0xBB
#define EEPROM_COEFFICIENT_B_SENSOR1_2      0xBC
#define EEPROM_COEFFICIENT_B_SENSOR1_3      0xBD

#define EEPROM_COEFFICIENT_C_SENSOR1_0      0xBE
#define EEPROM_COEFFICIENT_C_SENSOR1_1      0xBF
#define EEPROM_COEFFICIENT_C_SENSOR1_2      0xC0
#define EEPROM_COEFFICIENT_C_SENSOR1_3      0xC1

#define EEPROM_COEFFICIENT_A_SENSOR2_0      0xC2
#define EEPROM_COEFFICIENT_A_SENSOR2_1      0xC3
#define EEPROM_COEFFICIENT_A_SENSOR2_2      0xC4
#define EEPROM_COEFFICIENT_A_SENSOR2_3      0xC5

#define EEPROM_COEFFICIENT_B_SENSOR2_0      0xC6
#define EEPROM_COEFFICIENT_B_SENSOR2_1      0xC7
#define EEPROM_COEFFICIENT_B_SENSOR2_2      0xC8
#define EEPROM_COEFFICIENT_B_SENSOR2_3      0xC9

#define EEPROM_COEFFICIENT_C_SENSOR2_0      0xCA
#define EEPROM_COEFFICIENT_C_SENSOR2_1      0xCB
#define EEPROM_COEFFICIENT_C_SENSOR2_2      0xCC
#define EEPROM_COEFFICIENT_C_SENSOR2_3      0xCD

#define EEPROM_COEFFICIENT_A_SENSOR3_0      0xCE
#define EEPROM_COEFFICIENT_A_SENSOR3_1      0xCF
#define EEPROM_COEFFICIENT_A_SENSOR3_2      0xD0
#define EEPROM_COEFFICIENT_A_SENSOR3_3      0xD1

#define EEPROM_COEFFICIENT_B_SENSOR3_0      0xD2
#define EEPROM_COEFFICIENT_B_SENSOR3_1      0xD3
#define EEPROM_COEFFICIENT_B_SENSOR3_2      0xD4
#define EEPROM_COEFFICIENT_B_SENSOR3_3      0xD5

#define EEPROM_COEFFICIENT_C_SENSOR3_0      0xD6
#define EEPROM_COEFFICIENT_C_SENSOR3_1      0xD7
#define EEPROM_COEFFICIENT_C_SENSOR3_2      0xD8
#define EEPROM_COEFFICIENT_C_SENSOR3_3      0xD9

#define EEPROM_COEFFICIENT_A_SENSOR4_0      0xDA
#define EEPROM_COEFFICIENT_A_SENSOR4_1      0xDB
#define EEPROM_COEFFICIENT_A_SENSOR4_2      0xDC
#define EEPROM_COEFFICIENT_A_SENSOR4_3      0xDD

#define EEPROM_COEFFICIENT_B_SENSOR4_0      0xDE
#define EEPROM_COEFFICIENT_B_SENSOR4_1      0xDF
#define EEPROM_COEFFICIENT_B_SENSOR4_2      0xE0
#define EEPROM_COEFFICIENT_B_SENSOR4_3      0xE1

#define EEPROM_COEFFICIENT_C_SENSOR4_0      0xE2
#define EEPROM_COEFFICIENT_C_SENSOR4_1      0xE3
#define EEPROM_COEFFICIENT_C_SENSOR4_2      0xE4
#define EEPROM_COEFFICIENT_C_SENSOR4_3      0xE5

#define EEPROM_COEFFICIENT_A_SENSOR5_0      0xE6
#define EEPROM_COEFFICIENT_A_SENSOR5_1      0xE7
#define EEPROM_COEFFICIENT_A_SENSOR5_2      0xE8
#define EEPROM_COEFFICIENT_A_SENSOR5_3      0xE9

#define EEPROM_COEFFICIENT_B_SENSOR5_0      0xEA
#define EEPROM_COEFFICIENT_B_SENSOR5_1      0xEB
#define EEPROM_COEFFICIENT_B_SENSOR5_2      0xEC
#define EEPROM_COEFFICIENT_B_SENSOR5_3      0xED

#define EEPROM_COEFFICIENT_C_SENSOR5_0      0xEE
#define EEPROM_COEFFICIENT_C_SENSOR5_1      0xEF
#define EEPROM_COEFFICIENT_C_SENSOR5_2      0xF0
#define EEPROM_COEFFICIENT_C_SENSOR5_2      0xF1

#define EEPROM_CALIBRATED_VOLTAGE_MSB       0xF2
#define EEPROM_CALIBRATED_VOLTAGE_LSB       0xF3

// ADCCON0 ADC select bits
#define SELECT_ADC_TEMP0    (10<<2)  // ADC10
#define SELECT_ADC_TEMP1    (2<<2)   // ADC2
#define SELECT_ADC_TEMP2    (1<<2)   // ADC1
#define SELECT_ADC_TEMP3    (0<<2)   // ADC0
#define SELECT_ADC_TEMP4    (9<<2)   // ADC9
#define SELECT_ADC_TEMP5    (8<<2)   // ADC8

// Temeparture conversions
double Celsius2Fahrenheit(double tc);
double Fahrenheit2Celsius(double tf);
double Celsius2Kelvin(double tc);
double Kelvin2Celsius(double tf);

/*!
	Send Extended ID CAN frame
	@param id CAN extended ID for frame.
	@param size Number of databytes 0-8
	@param pData Pointer to databytes of frame.
	@return TRUE (!=0) on success, FALSE (==0) on failure.
*/
int8_t sendCANFrame( uint32_t id, uint8_t size, uint8_t *pData );

/*!
	Get extended ID CAN frame
	@param pid Pointer to CAN extended ID for frame.
	@param psize Pointer to number of databytes 0-8
	@param pData Pointer to databytes of frame.
	@return TRUE (!=0) on success, FALSE (==0) on failure.
*/
int8_t getCANFrame( uint32_t *pid, uint8_t *psize, uint8_t *pData );


#endif