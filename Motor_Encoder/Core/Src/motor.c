/*
 *  motor.c
 *
 *  V1: Main implementation of DC Motor & Actuator control functions
 *  V2: Main implementation of Servo Motor control function
 * 
 *  Created on: Jan 11, 2023
 *      Author: Jackie Huynh & Michelle Tran
 * 
 */

#include "motor.h"

static char freqError = 0;

#define getFreq(System_CLK, Timer_Prescaler, Timer_Period)		(System_CLK / (Timer_Prescaler+1)) / (Timer_Period+1)

static uint32_t Get_Freq(uint32_t systemClk, uint32_t timerPrescaler, uint32_t timerPeriod){
    return (systemClk / (timerPrescaler+1)) / (timerPeriod+1);
}

static inline uint32_t map(int8_t x, int8_t x_min, int8_t x_max, uint32_t out_min, uint32_t out_max){

	if(x <= x_min)
		return x_min;

	if(x >= x_max)
		return x_max;

	return (x - x_min) * (out_max - out_min) / (x_max - x_min) + out_min;

}

//Servo Functions
Servo_Error Servo_Init(Servo_Instance_t* servo){

    /* Makes sure Servo Config Struct is initialized */
    if(servo->config == NULL)   
        return SERVO_INSTANCE_ERROR;

    uint16_t timFreq;

	/* Check if this timer is setup correctly for SERVO Control */
	timFreq = Get_Freq(HAL_RCC_GetSysClockFreq(), servo->htim->Init.Prescaler, __HAL_TIM_GET_AUTORELOAD(servo->htim));

	if(timFreq != DESIRED_SERVO_FREQ){
		freqError = 1;
		return SERVO_FREQ_ERROR;
	}

	/* Configure Min and Max count based on duty cycle given */
	servo->minCnt = servo->config->minDuty * __HAL_TIM_GET_AUTORELOAD(servo->htim);
	servo->maxCnt = servo->config->maxDuty * __HAL_TIM_GET_AUTORELOAD(servo->htim);

	/* Zero Out SERVO */
	Drive_Servo(servo, 0);

    return SERVO_OK;
}

Servo_Error Drive_Servo(const Servo_Instance_t* servo, const int8_t angle){

    /* Servo Protection */
	if(freqError)
		return SERVO_FREQ_ERROR;

	/* Asserting Params */
	if(servo == NULL)
		return SERVO_INSTANCE_ERROR;
	else if(angle < servo->config->minAngle)
		return SERVO_RANGE_ERROR_MIN;
	else if(angle > servo->config->maxAngle)
		return SERVO_RANGE_ERROR_MAX;

	/* Set New Compare Value */
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, map(angle, servo->config->minAngle, servo->config->maxAngle, servo->minCnt, servo->maxCnt));

    return SERVO_OK;
}

//DC Motor Functions
DCMotor_Error DCMotor_Init(DCMotor_Instance_t* dcMotor){


    return DC_MOTOR_OK;
}

DCMotor_Error Drive_DCMotor(const DCMotor_Instance_t* dcMotor){


    return DC_MOTOR_OK;
}

DCMotor_Error Drive_DCMotor_Angle(const DCMotor_Instance_t* dcMotor){

    
    return DC_MOTOR_OK;
}

//Actuator Functions
Actuator_Error Actuator_Init(Actuator_Instance_t* act){


    return ACTUATOR_OK;
}

Actuator_Error Drive_Actuator(const Actuator_Instance_t* act){


    return ACTUATOR_OK;
}

