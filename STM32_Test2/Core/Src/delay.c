#include "delay.h"
#include "tim.h"
extern TIM_HandleTypeDef htim1; // Giả định TIM1 được cấu hình

void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us)
    {
        if (__HAL_TIM_GET_COUNTER(&htim1) > us) break; // Phòng trường hợp counter tràn
    }
}
