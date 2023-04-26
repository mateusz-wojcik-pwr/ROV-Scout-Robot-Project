/*
 * BMPXX0.h
 *
 *  The MIT License.
 *  Based on Adafuit libraries.
 *  Created on: 10.08.2018
 *      Author: Mateusz Salamon
 *      www.msalamon.pl
 *
 */

#ifndef BMPXX80_H_
#define BMPXX80_H_

//
// I2C address
//

#define BMP280_I2CADDR	0xEC

//
//	Mode
//

// Temperature resolution
#define BMP280_TEMPERATURE_16BIT 1
#define BMP280_TEMPERATURE_20BIT 5

// Mode
#define BMP280_SLEEPMODE		0
#define BMP280_FORCEDMODE		1
#define BMP280_NORMALMODE		3

//
//	Coeffs registers
//

#define	BMP280_DIG_T1		0x88
#define	BMP280_DIG_T2		0x8A
#define	BMP280_DIG_T3		0x8C

//
//	Registers
//

#define	BMP280_CHIPID			0xD0
#define	BMP280_VERSION			0xD1
#define	BMP280_SOFTRESET		0xE0
#define	BMP280_CAL26			0xE1  // R calibration stored in 0xE1-0xF0
#define	BMP280_STATUS			0xF3
#define	BMP280_CONTROL			0xF4
#define	BMP280_CONFIG			0xF5
#define	BMP280_TEMPDATA			0xFA

//
// User functions
//

void BMP280_Init(I2C_HandleTypeDef *i2c_handler, uint8_t temperature_resolution, uint8_t mode);
float BMP280_ReadTemperature(void);
uint8_t BMP280_Read_Temperature(float *temperature);


#endif /* BMPXX80_H_ */
