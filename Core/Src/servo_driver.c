/*
 * servo_driver.c
 *
 *  Created on: 12 maj 2021
 *      Author: Marek Jaromin
 */

#ifndef SRC_SERVO_DRIVER_C_
#define SRC_SERVO_DRIVER_C_

#include "servo_driver.h"

/**
  * @brief FUNCTION TO GET PERIOD VALUE OF GIVEN TIMER
  * @param TIM_HandleTypeDef *htim
  * @retval period_value - PERIOD VALUE OF GIVEN TIMER
  */
uint32_t GET_PERIOD_VALUE(TIM_HandleTypeDef *htim){

	uint32_t period_value;
	period_value = htim->Init.Period;
	return period_value;
}

/**
  * @brief SETTING POSITION OF SERVO WITH A GIVEN VALUE(0-180)
  * CALCULATIONS ARE BASED ON PROPORTION OF PULSE WITHDRAW AND POSITION
  * @param uint8_t position - DESIRED POSITION OF SERVO(0-180)
  * @param servo_handle_t *servo - POINTER TO SERVO HANDLE STRUCTURE
  * @retval
  */
void SERVO_SET_POSITION(uint8_t position, servo_handle_t *servo){

	float tim_pulse_time_withdraw = SERVO_MIN_PULSE_WITHDRAW + ((float)position*(float)SERVO_MIN_PULSE_WITHDRAW)/SERVO_MAX_POSITION;
	uint32_t tim_pulse_value = (uint32_t)((tim_pulse_time_withdraw*__HAL_TIM_GET_AUTORELOAD(servo->htim))/SERVO_PULSE_PERIOD);

	__HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, tim_pulse_value);
}

/**
  * @brief SETTING POSITION OF SERVO WITH A GIVEN VALUE(0-180)
  * CALCULATIONS ARE BASED ON PROPORTION OF PULSE WITHDRAW AND POSITION
  * @param uint8_t position - DESIRED POSITION OF SERVO(0-180)
  * @param servo_handle_t *servo - POINTER TO SERVO HANDLE STRUCTURE
  * @retval None
  */
void SERVO_SET_POSITION_UP(servo_handle_t *servo){

	float tim_pulse_time_withdraw = SERVO_MIN_PULSE_WITHDRAW + ((float)((servo->position) + SERVO_POSITION_UP_OFFSET)*(float)SERVO_MIN_PULSE_WITHDRAW)/SERVO_MAX_POSITION;
	uint32_t tim_pulse_value = (uint32_t)((tim_pulse_time_withdraw*__HAL_TIM_GET_AUTORELOAD(servo->htim))/SERVO_PULSE_PERIOD);
	if((servo->position) < 180){
	servo->position += SERVO_POSITION_UP_OFFSET;
	}
	__HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, tim_pulse_value);
}

/**
  * @brief SETTING DOWN POSITION OF SERVO BY VALUE DEFINED IN SERVO HEADER FILE
  * CALCULATIONS ARE BASED ON PROPORTION OF PULSE WITHDRAW AND POSITION
  * @param servo_handle_t *servo - POINTER TO SERVO HANDLE STRUCTURE
  * @retval None
  */
void SERVO_SET_POSITION_DOWN(servo_handle_t *servo){

	float tim_pulse_time_withdraw = SERVO_MIN_PULSE_WITHDRAW + ((float)((servo->position) + SERVO_POSITION_DOWN_OFFSET)*(float)SERVO_MIN_PULSE_WITHDRAW)/SERVO_MAX_POSITION;
	uint32_t tim_pulse_value = (uint32_t)((tim_pulse_time_withdraw*__HAL_TIM_GET_AUTORELOAD(servo->htim))/SERVO_PULSE_PERIOD);

	if((servo->position) > -1){
		servo->position += SERVO_POSITION_DOWN_OFFSET;
	}

	__HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, tim_pulse_value);
}

/**
  * @brief SETTING NEUTRAL POSITION OF SERVO TO THE VALUE DEFINED IN SERVO HEADER FILE
  * IN THIS PROJECT 90 IS CONSIDERED AS NEUTRAL POSITION
  * CALCULATIONS ARE BASED ON PROPORTION OF PULSE WITHDRAW AND POSITION
  * @param servo_handle_t *servo - POINTER TO SERVO HANDLE STRUCTURE
  * @retval None
  */
void SERVO_SET_NEUTRAL_POSITION(servo_handle_t *servo){

	float tim_pulse_time_withdraw = SERVO_MIN_PULSE_WITHDRAW + ((float)SERVO_NEUTRAL_POSITION*(float)SERVO_MIN_PULSE_WITHDRAW)/SERVO_MAX_POSITION;
	uint32_t tim_pulse_value =(uint32_t) ((tim_pulse_time_withdraw*__HAL_TIM_GET_AUTORELOAD(servo->htim))/SERVO_PULSE_PERIOD);

	servo->position = SERVO_NEUTRAL_POSITION;

	__HAL_TIM_SET_COMPARE(servo->htim, servo->Channel, tim_pulse_value);
}


#endif /* SRC_SERVO_DRIVER_C_ */
