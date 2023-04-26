#ifndef ANALOG_H_
#define ANALOG_H_


#include "stm32l4xx_hal.h"
#include "math.h"

#define JS_A_X_ADDR 0
#define JS_A_Y_ADDR 1
#define JS_B_X_ADDR 2
#define JS_B_Y_ADDR 3

struct Joystick{
	uint16_t zeroValue[2];
	uint8_t dataAddress;
	uint16_t rawValue[2];
	uint16_t smoothedValue[2];
	uint16_t smoothingFactor;
	int16_t position[2];
	uint16_t deadZone;
	uint16_t minValue[2];
	uint16_t maxValue[2];
	uint16_t maxTravel[2];
	uint16_t minTravel[2];
};

void analogInit();
void JS_Aquisite(struct Joystick *JS, uint16_t *dmaArray);
void JS_Init(struct Joystick *JS, uint8_t addr, uint16_t s, uint16_t dZ);
void JS_Calibrate(struct Joystick *JS, uint8_t axisFlag,  uint16_t minValue, uint16_t maxValue, uint16_t middleValue);

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);


#endif
