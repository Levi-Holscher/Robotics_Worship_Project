/*
 * quad_encoder.cpp
 *
 *  Created on: Jul 6, 2023
 *      Author: Levi.Holscher
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#include "main.h"
#include "quad_encoder.h"
extern TIM_HandleTypeDef htim1;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
{
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	uint32_t velocity_counter = encoder_value -> velocity;
	static uint8_t first_time = 0;
	if(!first_time)
	{
		encoder_value -> accel = 0;
		encoder_value -> velocity = 0;
		//(*encoder_value).velocity = 0;  AKA of above line
		first_time = 1;
	}
	else
	{
		if(temp_counter == encoder_value -> last_counter_value)
		{
			encoder_value -> velocity = 0;
		}
		else if(temp_counter > encoder_value -> last_counter_value)
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value -> velocity = -encoder_value -> last_counter_value - (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter);
			}
			else
			{
				encoder_value -> velocity = temp_counter - encoder_value -> last_counter_value;
			}
		}
		else
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value -> velocity = temp_counter - encoder_value -> last_counter_value;
			}
			else
			{
				encoder_value -> velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder_value -> last_counter_value);
			}
		}
	}
	encoder_value -> position += encoder_value->velocity;
	encoder_value -> accel = encoder_value->velocity - velocity_counter;
	encoder_value -> last_counter_value = temp_counter;
}

void reset_encoder( encoder_instance *encoder_value)
{
	encoder_value -> position = 0;
	encoder_value -> velocity = 0;
	encoder_value -> accel = 0;
	encoder_value -> last_counter_value = 0;
}




//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	timer_counter = __HAL_TIM_GET_COUNTER(&htim1);
//	update_encoder(&encoder1, &htim1);
//	encoder_position = encoder1.position;
//	encoder_velocity = encoder1.velocity;
//	encoder_accel = encoder1.accel;
//}


