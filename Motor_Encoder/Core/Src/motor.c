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
#include <math.h>

static char servoFreqError = 0;
static char actFreqError = 0;

static uint32_t Get_Freq(uint32_t systemClk, uint32_t timerPrescaler, uint32_t timerPeriod){
    return (systemClk / (timerPrescaler+1)) / (timerPeriod+1);
}

static inline int32_t map(int32_t x, int32_t x_min, int32_t x_max, int32_t out_min, int32_t out_max){

	if(x <= x_min)
		return x_min;

	if(x >= x_max)
		return x_max;

	return (x - x_min) * (out_max - out_min) / (x_max - x_min) + out_min;

}

//Servo Functions
Servo_Error Servo_Init(Servo_Instance_t* servo){

    /* Makes sure Servo Struct and Servo Config Struct are initialized */
    if(servo == NULL)   
        return SERVO_INSTANCE_ERROR;
    else if(servo->config == NULL)
        return SERVO_INSTANCE_ERROR;

    uint16_t timFreq;

	/* Check if this timer is setup correctly for SERVO Control */
	timFreq = Get_Freq(HAL_RCC_GetSysClockFreq(), servo->htim->Init.Prescaler, __HAL_TIM_GET_AUTORELOAD(servo->htim));

	if(timFreq != DESIRED_SERVO_FREQ){
		servoFreqError = 1;
		return SERVO_FREQ_ERROR;
	}

	/* Configure Min and Max count based on duty cycle given */
	servo->minCnt = servo->config->minDuty * __HAL_TIM_GET_AUTORELOAD(servo->htim);
	servo->maxCnt = servo->config->maxDuty * __HAL_TIM_GET_AUTORELOAD(servo->htim);

    /* Start PWM Signal */
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
    HAL_TIM_GenerateEvent(servo->htim, TIM_EventSource_Update);     //Sync Counter

	/* Zero Out SERVO */
	Drive_Servo(servo, 0);

    return SERVO_OK;
}

Servo_Error Drive_Servo(const Servo_Instance_t* servo, const int8_t angle){

    /* Servo Protection */
	if(servoFreqError)
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
    HAL_TIM_GenerateEvent(servo->htim, TIM_EventSource_Update);

    return SERVO_OK;
}

//DC Motor Functions
DCMotor_Error DCMotor_Init(DCMotor_Instance_t* dcMotor){

    /* Assert Param */
    if(dcMotor == NULL)
        return DC_MOTOR_INSTANCE_ERR;
    else if(dcMotor->config == NULL)
        return DC_MOTOR_INSTANCE_ERR;

    /* Calculate Min and Max counter value based on user config */
    dcMotor->Min_Cnt = __HAL_TIM_GET_AUTORELOAD(dcMotor->DC_Timer) * (float)(dcMotor->config->Min_Speed / 100.0);
    dcMotor->Max_Cnt = __HAL_TIM_GET_AUTORELOAD(dcMotor->DC_Timer) * (float)(dcMotor->config->Max_Speed / 100.0);

    /* Start PWM Signal */
    HAL_TIM_PWM_Start(dcMotor->DC_Timer, dcMotor->IN1_Channel);
    HAL_TIM_PWM_Start(dcMotor->DC_Timer, dcMotor->IN2_Channel);
    HAL_TIM_GenerateEvent(dcMotor->DC_Timer, TIM_EventSource_Update);       //Sync Counter

    /* Any Encoder Initialization (WIP) */

    return DC_MOTOR_OK;
}

DCMotor_Error Drive_DCMotor(const DCMotor_Instance_t* dcMotor, const uint8_t speed, DCMotor_Direction dir){

    /* Assert Param */
    if(dcMotor == NULL)
        return DC_MOTOR_INSTANCE_ERR;
    else if(dcMotor->config == NULL)
        return DC_MOTOR_INSTANCE_ERR;
    else if(speed < dcMotor->config->Min_Speed)
        return DC_MOTOR_UNDER_RANGE;
    else if(speed > dcMotor->config->Max_Speed)
        return DC_MOTOR_ABOVE_RANGE;

    /* Map speed value to counter value */
    uint16_t mappedValue = map(speed, dcMotor->config->Min_Speed, dcMotor->config->Max_Speed, dcMotor->Min_Cnt, dcMotor->Max_Cnt);

    if(dir == CLOCKWISE){
        __HAL_TIM_SET_COMPARE(dcMotor->DC_Timer, dcMotor->IN1_Channel, mappedValue);
        __HAL_TIM_SET_COMPARE(dcMotor->DC_Timer, dcMotor->IN2_Channel, 0);
    }
    else if(dir == COUNTER_CLOCKWISE){
        __HAL_TIM_SET_COMPARE(dcMotor->DC_Timer, dcMotor->IN1_Channel, 0);
        __HAL_TIM_SET_COMPARE(dcMotor->DC_Timer, dcMotor->IN2_Channel, mappedValue);
    }

    HAL_TIM_GenerateEvent(dcMotor->DC_Timer, TIM_EventSource_Update);       //Sync Counter

    return DC_MOTOR_OK;
}

//WIP
DCMotor_Error Drive_DCMotor_Angle(const DCMotor_Instance_t* dcMotor){

    
    return DC_MOTOR_OK;
}

//Actuator Functions
Actuator_Error Actuator_Init(Actuator_Instance_t* act){

    /* Makes sure Actuator Instance Struct and Actuator Config struct are initialzied */
    if(act == NULL)
        return ACTUATOR_INSTANCE_ERROR;
    else if(act->config == NULL)
        return ACTUATOR_INSTANCE_ERROR;

    /* Calculate Timer Frequency */
    uint16_t timFreq = 0;
    timFreq = Get_Freq(HAL_RCC_GetSysClockFreq(), act->Act_Timer->Init.Prescaler, __HAL_TIM_GET_AUTORELOAD(act->Act_Timer));

    /* Check if timer gives desired frequency */
    if(timFreq != DESIRED_ACT_FREQ){
        actFreqError = 1;
        return ACTUATOR_FREQ_ERROR;
    }

    /* Configure Min and Max counter based on user config */
    act->Min_Cnt = act->config->Min_Pulse / (1 / (DESIRED_ACT_FREQ * pow(10, -6)) / (float)__HAL_TIM_GET_AUTORELOAD(act->Act_Timer));
    act->Max_Cnt = act->config->Max_Pulse / (1 / (DESIRED_ACT_FREQ * pow(10, -6)) / (float)__HAL_TIM_GET_AUTORELOAD(act->Act_Timer));

    /* Start Actuator PWM Signal */
    __HAL_TIM_SET_COMPARE(act->Act_Timer, act->Channel, act->Min_Cnt);
    HAL_TIM_PWM_Start(act->Act_Timer, act->Channel);
    HAL_TIM_GenerateEvent(act->Act_Timer, TIM_EventSource_Update);      //Sync Coutner

    return ACTUATOR_OK;
}

Actuator_Error Drive_Actuator(const Actuator_Instance_t* act, const uint8_t length){

    /* Asserting Parameters */
    if(act == NULL)
        return ACTUATOR_INSTANCE_ERROR;
    else if(act->config == NULL)
        return ACTUATOR_INSTANCE_ERROR;
    else if(length < act->config->Min_Length)
        return ACTUATOR_UNDER_RANGE;
    else if(length > act->config->Max_Length)
        return ACTUATOR_ABOVE_RANGE;

    /* Map length to counter value range */
    uint16_t mappedLength = map(length, act->config->Min_Length, act->config->Max_Length, act->Min_Cnt, act->Max_Cnt);
    
    /* Set New PWM Compare Value */
    __HAL_TIM_SET_COMPARE(act->Act_Timer, act->Channel, mappedLength);

    HAL_TIM_GenerateEvent(act->Act_Timer, TIM_EventSource_Update);      //Sync Counter

    return ACTUATOR_OK;
}

