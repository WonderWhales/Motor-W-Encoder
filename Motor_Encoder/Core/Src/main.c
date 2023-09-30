/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Servo Motor Struct */
Servo_Config_t servoConfig;
Servo_Instance_t servoInstance;

/* Actuator Struct */
Actuator_Config_t actConfig;
Actuator_Instance_t actInstance;

/* DCMotor Struct */
DCMotor_Config_t dcMotorConfig;
DCMotor_Instance_t dcMotor;

/* Encoder Struct */
DCMotor_Encoder_Config_t encConfig;
DCMotor_Encoder_Instance_t encInstance;

/* General Motor Var */
Servo_Error servoError;
Actuator_Error actError;
DCMotor_Error dcError;

uint32_t encoderCnt = 0;
char motorDone = 0;
uint8_t uartBuf[4];
#define defaultValue (35525U)
uint8_t defaultPrint[] = "Please Enter Desired Angle: \n";
volatile char inUART = 0;
const char HANDSHAKE = 0xAA;
int16_t userBalls = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  servoConfig.minDuty   = 0.025;
  servoConfig.maxDuty   = 0.125;
  servoConfig.minAngle  = -90;
  servoConfig.minAngle  = 90;

  servoInstance.htim    = &htim3;
  servoInstance.channel = TIM_CHANNEL_2;
  servoInstance.config  = &servoConfig;

  servoError = Servo_Init(&servoInstance);
  if(servoError != SERVO_OK){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while(1);
  }

  actConfig.Min_Pulse          = 900;
  actConfig.Max_Pulse          = 2100;
  actConfig.Min_Length         = 0;
  actConfig.Max_Length         = 27;
  actConfig.Desired_Max_Length = 0;
  actConfig.Desired_Max_Length = 16;

  actInstance.Act_Timer = &htim3;
  actInstance.Channel   = TIM_CHANNEL_1;
  actInstance.config    = &actConfig;

  actError = Actuator_Init(&actInstance);
  if(actError != ACTUATOR_OK){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while(1);
  }

  Actuator_Init(&actInstance);

  Drive_Actuator(&actInstance, 8);

  Drive_Actuator(&actInstance, 16);

  Drive_Actuator(&actInstance, 19);

  dcMotorConfig.Min_Speed = 0;
  dcMotorConfig.Max_Speed = 100;

  dcMotor.DC_Timer      = &htim12;
  dcMotor.IN1_Channel   = TIM_CHANNEL_1;
  dcMotor.IN2_Channel   = TIM_CHANNEL_2;
  dcMotor.config        = &dcMotorConfig;
  
  DCMotor_Init(&dcMotor);

  encConfig.Current_Angle     = 0;
  encConfig.Default_Counter   = defaultValue;
  encConfig.Degree_Per_Pulse  = (float)360 / defaultValue;
  encConfig.Min_Angle         = -360;
  encConfig.Max_Angle         = 360;

  encInstance.Encoder_Timer   = &htim5;
  encInstance.encConfig       = &encConfig;
  encInstance.motorInstance   = &dcMotor;

  DCMotor_Encoder_Init(&encInstance);

  //Drive_DCMotor_Angle(&encInstance, -60);

  // __HAL_TIM_SET_COUNTER(&htim5, rotationValue);
  // HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // encoderCnt = __HAL_TIM_GET_COUNTER(&htim5);
  // dcError = Drive_DCMotor(&dcMotor, 45, CLOCKWISE);
  // if(dcError != DC_MOTOR_OK){
  //   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  //   while(1);
  // }
  // HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // encoderCnt = __HAL_TIM_GET_COUNTER(&htim5);
    // if(encoderCnt >= 44406){
    //   Stop_DCMotor(&dcMotor);
    //   HAL_Delay(1000);
    //   dcError = Drive_DCMotor(&dcMotor, 42, COUNTER_CLOCKWISE);
    //   while(!motorDone){
    //     encoderCnt = __HAL_TIM_GET_COUNTER(&htim5);
    //     if(encoderCnt <= 35525){
    //       Stop_DCMotor(&dcMotor);
    //       HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    //       motorDone = 1;
    //     }
    //   }
    // }

  //HAL_UART_Transmit(&huart2, defaultPrint, sizeof(defaultPrint), 100);
  HAL_UART_Receive_IT(&huart2, uartBuf, sizeof(uartBuf));
  while(!inUART);
  inUART = 0;
  userBalls = atoi(uartBuf);
  Drive_DCMotor_Angle(&encInstance, userBalls);
  HAL_UART_Transmit(&huart2, &HANDSHAKE, 1, 100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
