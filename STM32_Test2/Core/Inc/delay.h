/*
 * delay.h
 *
 *  Created on: Oct 26, 2025
 *      Author: ADMIN
 */
#include "main.h"
#include "stm32f1xx_hal.h"

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

extern TIM_HandleTypeDef htim1;
void delay_us(uint32_t us);

#endif /* INC_DELAY_H_ */
