#include "motors.h"


TIM_HandleTypeDef *tim;

/*initialize motors - set channel and timer handler */
void motorsInit(TIM_HandleTypeDef *htim, struct Motor* mot, uint8_t ch){
	tim = htim;
	mot->dir = 0;
	mot->duty = 0;
	mot->en = 0;
	mot->tim_channel = ch;

	__HAL_TIM_SET_COMPARE(tim, mot->tim_channel, mot->duty);
	HAL_TIM_PWM_Start(tim, mot->tim_channel);
}

/* set motor PWM to the passed motor structure */
void motorSetValue(struct Motor* mot, int16_t val){
	uint16_t pwmDuty;
	if(val < 0)
		mot->dir = 0;
	else
		mot->dir = 1;

	pwmDuty = abs(val);
	if(pwmDuty > 2000)
		pwmDuty = 2000;

	mot->duty = pwmDuty;
}


/* function to update motor parameters*/
void motorUpdate(struct Motor* mot, GPIO_TypeDef* en_gpio, uint32_t en_pin, GPIO_TypeDef* dir_gpio, uint32_t dir_pin){
	if((mot->p_en != mot->en) || (mot->p_dir != mot->dir) || (mot->p_duty != mot->duty)){

		if(mot->dir)
			HAL_GPIO_WritePin(dir_gpio, dir_pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(dir_gpio, dir_pin, GPIO_PIN_RESET);

		if(mot->en)
			HAL_GPIO_WritePin(en_gpio, en_pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(en_gpio, en_pin, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(tim, mot->tim_channel, mot->duty);
	}
	mot->p_en = mot->en;
	mot->p_dir = mot->dir;
	mot->p_duty = mot->duty;
}





