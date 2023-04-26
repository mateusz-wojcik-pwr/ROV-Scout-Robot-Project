#ifndef MOTORS_H
#define MOTORS_H
#include "stm32l4xx_hal.h"
#include "math.h"



struct Motor{
	uint8_t tim_channel;
	uint16_t duty;
	uint16_t p_duty;
	uint8_t dir;
	uint8_t p_dir;
	uint8_t en;
	uint8_t p_en;
	uint16_t en_pin;
	uint16_t dir_pin;
};

void motorsInit(TIM_HandleTypeDef *htim, struct Motor* mot, uint8_t ch);
void motorSetValue(struct Motor* mot, int16_t val);
void motorUpdate(struct Motor* mot, GPIO_TypeDef* en_gpio, uint32_t en_pin, GPIO_TypeDef* dir_gpio, uint32_t dir_pin);



#endif
