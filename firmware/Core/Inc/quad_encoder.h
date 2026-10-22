/*
 * quad_encoder.h
 *
 *  Created on: Jul 26, 2023
 *      Author: Levi.Holscher
 */

#ifndef INC_QUAD_ENCODER_H_
#define INC_QUAD_ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h" // Ensure this is included to provide the definition of TIM_HandleTypeDef

typedef struct {
    int16_t accel;
    int16_t velocity;
    int64_t position;
    uint32_t last_counter_value;
} encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *encoder_value);

#ifdef __cplusplus
}
#endif

#endif /* INC_QUAD_ENCODER_H_ */
