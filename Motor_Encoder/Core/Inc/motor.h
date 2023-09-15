/*
 *  motor.h
 *
 *  V1: Provide DC Motor & Actuator control functions
 *  V2: Provide Servo Motor control function
 * 
 *  Created on: Jan 11, 2023
 *      Author: Jackie Huynh & Michelle Tran
 * 
 */

#ifndef MOTOR_H_
#define MOTOR_H

#include "main.h"

//Servo Motor Error Enum
typedef enum{

	SERVO_OK 		 		= 0x00U,
	SERVO_FREQ_ERROR 		= 0x01U,
	SERVO_RANGE_ERROR_MIN	= 0x02U,
	SERVO_RANGE_ERROR_MAX	= 0x03U,
	SERVO_INSTANCE_ERROR	= 0x04U

} Servo_Error;

//DC Motor Error Enum

typedef enum{
    DC_MOTOR_OK             = 0x00U,
	DC_MOTOR_UNDER_RANGE	= 0x01U,
	DC_MOTOR_ABOVE_RANGE	= 0x02U
} DCMotor_Error;

//Actuator Enum
typedef enum{
    ACTUATOR_OK             = 0x00U,
    ACTUATOR_UNDER_RANGE    = 0x01U,
    ACTUATOR_ABOVE_RANGE    = 0x02U
} Actuator_Error;

//Servo Motor Struct
typedef struct{

	float minDuty;
	float maxDuty;
	int8_t minAngle;
	int8_t maxAngle;

} SERVO_Config_t;

typedef struct{

	TIM_HandleTypeDef*	htim;
	uint8_t				channel;
	SERVO_Config_t*		config;

	/* These will be set in the INIT function */
	uint32_t 			minCnt;
	uint32_t 			maxCnt;

} Servo_Instance_t;

//DC Motor Struct
typedef struct DCMotor_Struct{
	TIM_HandleTypeDef DC_Timer;
	uint8_t IN1_Channel;
	uint8_t IN2_Channel;
	uint8_t Min_Speed;                      //Percentage Based
	uint8_t Max_Speed;                      //Percentage Based
} DCMotor_Instance_t;

//Actuator Struct
typedef struct{

    uint16_t Min_Pulse;
    uint16_t Max_Pulse;
    uint8_t Min_Length;
    uint8_t Max_Length;

} Actuator_Config_t;

typedef struct 
{
	TIM_HandleTypeDef Act_Timer;
	uint8_t Channel;
	uint8_t Min_Length;                     //mm Based
	uint8_t Max_Length;                     //mm Based
} Actuator_Instance_t;

/* List of Macros */
#define DESIRED_SERVO_FREQ  (50U)

//Servo Functions
Servo_Error Servo_Init(Servo_Instance_t* servo);
Servo_Error Drive_Servo(const Servo_Instance_t* servo, const int8_t angle);

//DC Motor Functions
DCMotor_Error DCMotor_Init(DCMotor_Instance_t* dcMotor);
DCMotor_Error Drive_DCMotor(const DCMotor_Instance_t* dcMotor);
DCMotor_Error Drive_DCMotor_Angle(const DCMotor_Instance_t* dcMotor);

//Actuator Functions
Actuator_Error Actuator_Init(Actuator_Instance_t* act);
Actuator_Error Drive_Actuator(const Actuator_Instance_t* act);

#endif