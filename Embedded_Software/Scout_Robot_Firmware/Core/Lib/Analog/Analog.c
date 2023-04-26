#include "Analog.h"


ADC_HandleTypeDef *adc;

void analogInit(ADC_HandleTypeDef *hadc, uint32_t* dmaArray){
	adc = hadc;
	HAL_ADC_Start_DMA(adc, dmaArray, 6);
}

void JS_Init(struct Joystick *JS, uint8_t addr, uint16_t s, uint16_t dZ){
	JS->dataAddress = addr;
	JS->smoothingFactor = s;
	JS->position[0] = 0;
	JS->position[1] = 0;
	JS->smoothedValue[0] = 0;
	JS->smoothedValue[1] = 0;
	JS->deadZone = dZ;
}

void JS_Aquisite(struct Joystick *JS, uint16_t *dmaArray){
	uint8_t x_addr = JS->dataAddress;
	uint8_t y_addr = JS->dataAddress+1;
	uint16_t rawX;
	uint16_t rawY;


	JS->rawValue[0] = dmaArray[x_addr];
	JS->rawValue[1] = dmaArray[y_addr];

	rawX = JS->rawValue[0];
	rawY = JS->rawValue[1];

	if(rawX > JS->maxValue[0])
		JS->maxValue[0] = rawX;

	if(rawY > JS->maxValue[1])
		JS->maxValue[1] = rawY;

	if(rawX < JS->minValue[0])
		JS->minValue[0] = rawX;

	if(rawY < JS->minValue[1])
		JS->minValue[1] = rawY;


	JS->smoothedValue[0] = (0.9 * JS->smoothedValue[0]) + ( 0.1 * rawX);
	JS->smoothedValue[1] = (0.9 * JS->smoothedValue[1]) + (0.1 * rawY);

	if(JS->smoothedValue[0] > JS->zeroValue[0]){
		JS->position[0] = map(JS->smoothedValue[0],JS->zeroValue[0], JS->maxValue[0], 0, 2047);
	}

	if(JS->smoothedValue[1] > JS->zeroValue[1]){
		JS->position[1] = map(JS->smoothedValue[1],JS->zeroValue[1], JS->maxValue[1], 0, 2047);
	}

	if(JS->smoothedValue[0] < JS->zeroValue[0]){
		JS->position[0] = map(JS->smoothedValue[0],JS->minValue[0], JS->zeroValue[0], -2048, 0);
	}
	if(JS->smoothedValue[1] < JS->zeroValue[0]){
		JS->position[1] = map(JS->smoothedValue[1],JS->minValue[1], JS->zeroValue[1], -2048, 0);
	}


	if(abs(JS->position[0]) < JS->deadZone)
		JS->position[0] = 0;

	if(abs(JS->position[1]) < JS->deadZone)
			JS->position[1] = 0;


	if(JS->position[0] > 2020)
		JS->position[0] = 2047;

	if(JS->position[1] > 2020)
		JS->position[1] = 2047;

	if(JS->position[0] < -2020)
		JS->position[0] = -2048;

	if(JS->position[1] < -2020)
		JS->position[1] = -2048;
}


void JS_Calibrate(struct Joystick *JS, uint8_t axisFlag,  uint16_t minValue, uint16_t maxValue, uint16_t middleValue){
if(!axisFlag){ //for X axis
	JS->minValue[0] = minValue;
	JS->maxValue[0] = maxValue;
	JS->zeroValue[0] = middleValue;

}
else{ //for Y axis
	JS->minValue[1] = minValue;
	JS->maxValue[1] = maxValue;
	JS->zeroValue[1] = middleValue;

 }
}


int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





