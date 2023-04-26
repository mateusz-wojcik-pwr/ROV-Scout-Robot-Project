/*
 * temt6000.h
 *
 *  Created on: Apr 17, 2023
 *      Author: zuzku
 */

#ifndef INC_TEMT6000_H_
#define INC_TEMT6000_H_


#define TEMT6000_ADC_MAX_VALUE	4096
#define TEMT6000_POWER_SUPPLY	3.3
#define TEMT6000_RESISTOR_OHMS	1000
#define TEMT6000_ADC_SAMPLES 8

typedef enum {
	TEMT6000_OK		= 0,
	TEMT6000_ERROR	= 1
} TEMT6000_STATUS;

TEMT6000_STATUS TEMT6000_Init(ADC_HandleTypeDef *hadc);

TEMT6000_STATUS TEMT6000_ReadLight(float *Result);
#endif /* TEMT6000_H_ */
