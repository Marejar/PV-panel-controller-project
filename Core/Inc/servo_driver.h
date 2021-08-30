/*
 * servo_driver.h
 *
 *  Created on: 12 maj 2021
 *      Author: Marek Jaromin
 */

#ifndef INC_SERVO_DRIVER_H_
#define INC_SERVO_DRIVER_H_

#include <main.h>
/*
 * To properly control servo motor it is needed to provide signal with period of 50Hz
 * with duty cycle between 5-10%. Servo can achive positions from 0-180, so minimal
 * pulse withdraw is 1ms(0 position, 5% duty cycle) and the maximum pulse withdraw
 * is 2ms(180 position, 10% duty cycle)
 */

#define SERVO_MIN_PULSE_WITHDRAW			1  	//STORES VALE OF MIN PULSE WITHDRAW IN ms TO PUT SERVO IN 0 POSITION
#define SERVO_MAX_POSITION					180 //STORES VALE OF MAX PULSE WITHDRAW IN ms TO PUT SERVO IN 180 POSITION
#define SERVO_PULSE_PERIOD					20 	//STORES VALUE OF CONTROL SIGNAL PERIOD IN ms(FOR CONTROLLING SERVO 50Hz IS NEEDED)
#define SERVO_NEUTRAL_POSITION				90 	//STORES VALUE OF NEUTRAL POSITION OF SERVO
#define SERVO_POSITION_UP_OFFSET			2
#define SERVO_POSITION_DOWN_OFFSET			(-2)

typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	uint8_t position;
}servo_handle_t;


uint32_t GET_PERIOD_VALUE(TIM_HandleTypeDef *htim);
void SERVO_SET_POSITION(uint8_t position, servo_handle_t *servo);
void SERVO_SET_POSITION_UP(servo_handle_t *servo);
void SERVO_SET_POSITION_DOWN(servo_handle_t *servo);
void SERVO_SET_NEUTRAL_POSITION(servo_handle_t *servo);

#endif /* INC_SERVO_DRIVER_H_ */
