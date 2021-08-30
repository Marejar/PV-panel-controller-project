/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd16x2_i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REGULATION_SENSITIVITY 200				//STORES VALUE OF SENSORS VOLTAGE DIFFERENCE WHICH WOULD NOT TRIGGER CHANGING POSITION OF SERVOS
#define BUFFER_SIZE				 1 				//STORES VALUE OF UART RECEIVED MESSAGE BUFFER SIZE, ONE BYTE
#define REMOTE_CONNECTION_ON	 1				//STORES INFORMATION ABOUT CONTROLING MODE: BY PHONE/BY MCU
#define REMOTE_CONNECTION_OFF    0				//STORES INFORMATION ABOUT CONTROLING MODE: BY PHONE/BY MCU


//CODES OF COMMANDS WHICH CAN BE SEND FROM PHONE VIA BLUETOOTH
#define SERVO_VERTICAL_POSITION_UP_COMMAND 		1
#define SERVO_VERTICAL_POSITION_DOWN_COMMAND 	2
#define SERVO_HORIZONTAL_POSITION_UP_COMMAND 	3
#define SERVO_HORIZONTAL_POSITION_DOWN_COMMAND 	4
#define ENABLE_REMOTE_CONTROL_COMMAND 			11
#define DISABLE_REMOTE_CONTROL_COMMAND 			12
#define	GET_SERVO_POSITION_COMMAND				13

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//SENSOR RELATED VARIABLES
uint16_t voltage_sensor_1; //LEFT-TOP SENSOR
uint16_t voltage_sensor_2; //RIGHT-TOP SENSOR
uint16_t voltage_sensor_3; //LEFT-DOWN SENSOR
uint16_t voltage_sensor_4; //RIGHT-DOWN SENSOR
volatile uint32_t adc_sensor_values[4];// [0] LEFT-TOP, [1] RIGHT-TOP, [2] LEFT-DOWN, [3] RIGHT-DOWN
ADC_ChannelConfTypeDef sConfig = {0};

//SERVOS RELATED VARIABLES
volatile uint8_t horizontal_servo_position = SERVO_NEUTRAL_POSITION;
volatile uint8_t vertical_servo_position = SERVO_NEUTRAL_POSITION;
servo_handle_t servo_horizontal = {&htim3, TIM_CHANNEL_1, SERVO_NEUTRAL_POSITION};
servo_handle_t servo_vertical = {&htim3, TIM_CHANNEL_3, SERVO_NEUTRAL_POSITION};

//COMMUNICATION RELATED VARIABLES AND USER MESSAGES
static const char *invalid_command_message = "Invalid command \n";
static const char *remote_ctrl_on_message = "Connected \nRemote control enabled\n";
static const char *remote_ctrl_off_message = "Remote control disabled\nDisconnected \n";
static const char *not_connected_message = "You're not connected to device. Use connect button.";
static const char *error_message = "Error\nInit. failed";
static char get_position_message[40];
char message[30];
char phone_rcvd_msg[4];

//REST OF THE VARIABLES
uint8_t uart1_buffer;
uint8_t count = 0;
volatile uint32_t delay_counter;//USED IN delay() FUNCTION
volatile uint8_t control_mode = REMOTE_CONNECTION_OFF;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
void ADC_CONFIG_CH0(void);
void ADC_CONFIG_CH1(void);
void ADC_CONFIG_CH4(void);
void ADC_CONFIG_CH8(void);
void COMPARE_SENSOR_VALUES(void);
void DECODE_MESSAGE(uint8_t command_code);
void REFRESH_INFO_ON_LCD(void);
void LCD_DISPLAY_MESSAGE(const char* message);
void ERROR_INFO_ON_LCD(void);
void delay(uint32_t delay_in_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  lcd16x2_i2c_init(&hi2c1);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  SERVO_SET_NEUTRAL_POSITION(&servo_horizontal);
  SERVO_SET_NEUTRAL_POSITION(&servo_vertical);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  HAL_UART_Receive_IT(&huart1, &uart1_buffer, BUFFER_SIZE);

	  	  ADC_CONFIG_CH0();
	  	  HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	  adc_sensor_values[0] = HAL_ADC_GetValue(&hadc1);
	  	  HAL_ADC_Stop(&hadc1);

	  	  ADC_CONFIG_CH1();
	  	  HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	  adc_sensor_values[1] = HAL_ADC_GetValue(&hadc1);
	  	  HAL_ADC_Stop(&hadc1);

	  	  ADC_CONFIG_CH4();
	  	  HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	  adc_sensor_values[2] = HAL_ADC_GetValue(&hadc1);
	  	  HAL_ADC_Stop(&hadc1);

	  	  ADC_CONFIG_CH8();
	  	  HAL_ADC_Start(&hadc1);
	  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  	  adc_sensor_values[3] = HAL_ADC_GetValue(&hadc1);
	  	  HAL_ADC_Stop(&hadc1);

	  	  if(control_mode == REMOTE_CONNECTION_OFF){
	  	  COMPARE_SENSOR_VALUES();
	  	  }

	  	  REFRESH_INFO_ON_LCD();
		  delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16800;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1260;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}


/* USER CODE BEGIN 4 */


/**
  * @brief CONFIGURATION OF CHANNEL0 AS A CURRENT ADC CHANNEL
  * @param None
  * @retval None
  */
void ADC_CONFIG_CH0(void){

		  sConfig.Channel = ADC_CHANNEL_0;
		  sConfig.Rank = 1;
		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
}

/**
  * @brief CONFIGURATION OF CHANNEL1 AS A CURRENT ADC CHANNEL
  * @param None
  * @retval None
  */
void ADC_CONFIG_CH1(void){

		  sConfig.Channel = ADC_CHANNEL_1;
		  sConfig.Rank = 1;
		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
}

/**
  * @brief CONFIGURATION OF CHANNEL2 AS A CURRENT ADC CHANNEL
  * @param None
  * @retval None
  */
void ADC_CONFIG_CH4(void){

		  sConfig.Channel = ADC_CHANNEL_4;
		  sConfig.Rank = 1;
		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
}

/**
  * @brief CONFIGURATION OF CHANNEL4 AS A CURRENT ADC CHANNEL
  * @param None
  * @retval None
  */
void ADC_CONFIG_CH8(void){

		  sConfig.Channel = ADC_CHANNEL_8;
		  sConfig.Rank = 1;
		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
}

/**
  * @brief 	COMPARISON OF LIGHT INTENSITY. AFTER COMPARISON, FUNCTION DECIDES IF
  * POSITION SHOULD BE CHANGED OR NOT, AND CALLS FUNCTIONS FROM SERVO LIBRARY IF
  * CHANGE OF POSITION IS NEEDED
  * @param None
  * @retval None
  */
void COMPARE_SENSOR_VALUES(void){

	uint32_t temp;
	//1st AND 2nd SENSOR HORIZONTAL COMPARISION
	if(adc_sensor_values[0] >= adc_sensor_values[1]){

		temp = adc_sensor_values[0] - adc_sensor_values[1];

		if(temp >= REGULATION_SENSITIVITY && servo_horizontal.position < 180){
			SERVO_SET_POSITION_UP(&servo_horizontal);
		}

	}else{

		temp = adc_sensor_values[1] - adc_sensor_values[0];

		if(temp >= REGULATION_SENSITIVITY && servo_horizontal.position > 0){
			SERVO_SET_POSITION_DOWN(&servo_horizontal);
		}
	}

	//3rd AND 4th SENSOR HORIZONTAL COMPARISION
	if(adc_sensor_values[2] >= adc_sensor_values[3]){

		temp = adc_sensor_values[2] - adc_sensor_values[3];

		if(temp >= REGULATION_SENSITIVITY && servo_horizontal.position < 180){
			SERVO_SET_POSITION_UP(&servo_horizontal);
		}

	}else{

		temp = adc_sensor_values[3] - adc_sensor_values[2];

		if(temp >= REGULATION_SENSITIVITY && servo_horizontal.position > 0){
			SERVO_SET_POSITION_DOWN(&servo_horizontal);
		}
	}

	//1st AND 3rd VERTICAL SENSOR COMPARISION
	if(adc_sensor_values[0] >= adc_sensor_values[2]){

		temp = adc_sensor_values[0] - adc_sensor_values[2];

		if(temp >= REGULATION_SENSITIVITY && servo_vertical.position < 180){
			SERVO_SET_POSITION_UP(&servo_vertical);
		}

	}else{

		temp = adc_sensor_values[2] - adc_sensor_values[0];

		if(temp >= REGULATION_SENSITIVITY && servo_vertical.position > 0){
			SERVO_SET_POSITION_DOWN(&servo_vertical);
		}
	}

	//2nd AND 4th VERTICAL SENSOR COMPARISION
	if(adc_sensor_values[1] >= adc_sensor_values[3]){

		temp = adc_sensor_values[1] - adc_sensor_values[3];

		if(temp >= REGULATION_SENSITIVITY && servo_vertical.position < 180){
			SERVO_SET_POSITION_UP(&servo_vertical);
		}

	}else{

		temp = adc_sensor_values[3] - adc_sensor_values[1];

		if(temp >= REGULATION_SENSITIVITY  && servo_vertical.position > 0){
			SERVO_SET_POSITION_DOWN(&servo_vertical);
		}
	}

}

/**
  * @brief DECODING MESSAGE(COMMAND) SENT FROM PHONE VIA HC-05 MODULE, AND
  * CALLING FUNCTION LINKED WITH PROCESSED COMMAND CODE
  * @param COMMAND CODE SENT FROM PHONE
  * @retval None
  */
void DECODE_MESSAGE(uint8_t command_code){

	switch(command_code){

		case ENABLE_REMOTE_CONTROL_COMMAND:
			control_mode = REMOTE_CONNECTION_ON;
			HAL_UART_Transmit(&huart1, (uint8_t*)remote_ctrl_on_message, strlen(remote_ctrl_on_message), HAL_MAX_DELAY);
			break;

		case DISABLE_REMOTE_CONTROL_COMMAND:
			control_mode = REMOTE_CONNECTION_OFF;
			HAL_UART_Transmit(&huart1, (uint8_t*)remote_ctrl_off_message, strlen(remote_ctrl_off_message), HAL_MAX_DELAY);
			break;

		default:
			if(control_mode == REMOTE_CONNECTION_OFF){
			HAL_UART_Transmit(&huart1, (uint8_t*)not_connected_message, strlen(not_connected_message), HAL_MAX_DELAY);
			}
			break;
	}


	if(control_mode == REMOTE_CONNECTION_ON){

		switch(command_code){

			case SERVO_HORIZONTAL_POSITION_UP_COMMAND:

				SERVO_SET_POSITION_UP(&servo_horizontal);
				break;

			case SERVO_HORIZONTAL_POSITION_DOWN_COMMAND:

				SERVO_SET_POSITION_DOWN(&servo_horizontal);
				break;

			case SERVO_VERTICAL_POSITION_UP_COMMAND:

				SERVO_SET_POSITION_UP(&servo_vertical);
				break;

			case SERVO_VERTICAL_POSITION_DOWN_COMMAND:

				SERVO_SET_POSITION_DOWN(&servo_vertical);
				break;

			case GET_SERVO_POSITION_COMMAND:
				sprintf(get_position_message, "POSITION Horizontal: %d, Vertical: %d \n\r", servo_horizontal.position, servo_vertical.position);
				HAL_UART_Transmit(&huart1, (uint8_t*)get_position_message, strlen(get_position_message), HAL_MAX_DELAY);
				break;

			default:
				if(command_code != ENABLE_REMOTE_CONTROL_COMMAND){
				HAL_UART_Transmit(&huart1, (uint8_t*)invalid_command_message, strlen(invalid_command_message), HAL_MAX_DELAY);
				}
				break;

		}
	}

}

/**
  * @brief USART2 INITIALIZATION, USART2 IS USED TO COMMUNICATE WITH HC-05 MODULE
  * @param None
  * @retval None
  */
static void USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART INTERRUPT RECEIVE COMPLETE CALLBACK, RECEIVING COMMANDS FROM PHONE
  * VIA HC-05
  * @param None
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1){

		if(uart1_buffer != '\r' ){
			phone_rcvd_msg[count] = uart1_buffer;
			count++;
			HAL_UART_Receive_IT(&huart1, &uart1_buffer, BUFFER_SIZE);
		}else{
			phone_rcvd_msg[count] = '\0';
			uint8_t command_code;
			command_code = atoi(phone_rcvd_msg);
			DECODE_MESSAGE(command_code);

			memset(&phone_rcvd_msg, 0, sizeof(phone_rcvd_msg));
			count = 0;
			HAL_UART_Receive_IT(&huart1, &uart1_buffer, BUFFER_SIZE);
		}

	}
}

/**
  * @brief DISPLAYING ACTUAL POSITION OF SERVOS ON LCD
  * @param None
  * @retval None
  */
void REFRESH_INFO_ON_LCD(void){

	lcd16x2_i2c_clear();

	lcd16x2_i2c_setCursor(0,0);
	lcd16x2_i2c_printf("Hor. pos,: %d", servo_horizontal.position);

	lcd16x2_i2c_setCursor(1,0);
	lcd16x2_i2c_printf("Ver. pos,: %d", servo_vertical.position);

}

/**
  * @brief DISPLAYING INFORMATION ABOUT INITIALIZATION FAIL
  * @param None
  * @retval None
  */
void ERROR_INFO_ON_LCD(void){

	lcd16x2_i2c_clear();

	lcd16x2_i2c_setCursor(0,0);
	lcd16x2_i2c_printf("ERROR");

	lcd16x2_i2c_setCursor(1,0);
	lcd16x2_i2c_printf("INIT. FAILED");

}

/**
  * @brief DELAY FUNCTION
  * @param DESIRED DELAY TIME IN ms
  * @retval None
  */
void delay(uint32_t delay_in_ms){

	delay_counter = delay_in_ms;
	while(delay_counter > 0);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  HAL_UART_Transmit(&huart1, (uint8_t*)error_message, strlen(error_message), HAL_MAX_DELAY);
  void ERROR_INFO_ON_LCD(void);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
