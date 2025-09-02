/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static bool flagss = false;  // GLOBAL variables are PV
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //const char * hm = "Don't know what happened I'm still HIM\n";
  uint32_t adc_val = 0u; // unsigned possible interger values from 2^32 set to 0
  char buff[100] ="";
  float voltage = 0.0f;
  uint16_t adc_max = 4095;  // 12 bit adc , so 2 ^ n max number
  float volt_ref = 3.3f; //
  float volts_per_bit = volt_ref/(float)adc_max; //
  char *hw = "Dont you forget, Bitch I'm the mANN\r\n";

  HAL_StatusTypeDef ok_notok = HAL_BUSY;

  HAL_UART_Transmit_IT(&huart,&tx_buffer_s,BUFFER_LEN); // lets the compiler know we are ready to transmit info talking about the huart2 handle,
  	  	  	  	  	  	  	  	  	  	  	  	  	   //tx_buffer_s is where the  received byte is transmitted and last argument is the size

  HAL_UART_Receive_IT(&huart,&rx_data_s, 1); // lets the compiler know we are ready to receive with the huart2 handle, rxData is where its stored
  	  	  	  	  	  	  	  	  	  	  	 // last arguement is the number of bytes to be stored
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	 (void)HAL_ADC_Start(&hadc1); // when using ADC always check initialize and start

	  	 ok_notok = HAL_ADC_PollForConversion(&hadc1,100);  // Blocks (waits) until the ADC finishes converting or until the timeout expires.
	  	 if(ok_notok == HAL_OK){
			 adc_val =  HAL_ADC_GetValue(&hadc1); // once the adc is ready, it starts another conversion
			 voltage = volts_per_bit * adc_val;  // REAL voltage per bit value * whatever analog value it is at that time
			 snprintf(buff, 100, "Analog Voltage: %.6f V and Digital Voltage: %lu\r\n", voltage, adc_val);  // snprintf to convert numbers to char
			 HAL_GPIO_WritePin(GREENLED_GPIO_Port, GREENLED_Pin, GPIO_PIN_RESET);
			 ok_notok = HAL_UART_Transmit(&huart2, (uint8_t *)buff, 100, 100);
			 HAL_Delay(1000);

	  	 }else{
	  		 HAL_GPIO_WritePin(GREENLED_GPIO_Port, GREENLED_Pin, GPIO_PIN_SET);
	  	 }
	  	if(flagss == true){
	  		 HAL_UART_Transmit(&huart2, (uint8_t *)hw, 37, 100);
	  		 flagss = false;
	  	}

	  	 HAL_Delay(200);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREENLED_GPIO_Port, GREENLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTOn_Pin */
  GPIO_InitStruct.Pin = BUTTOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREENLED_Pin */
  GPIO_InitStruct.Pin = GREENLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREENLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);   //  This function handles UART interrupt request. supposed to be inside RX interrupt
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  flagss = true;  // so when the interrupt is pressed, it sets flag to 1
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BUTTOn_Pin);

  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

// UART Transmit call back function : When you call:   When you call: HAL_UART_Transmit_IT
//The UART peripheral triggers a TX interrupt once the last byte has shifted out.
// HAL_UART_IRQHandler(&huart2) sees that the TX complete flag is set, clears it, and then calls:UART transmit function
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  	 memset(tx_buffer_s, '\0', (size)BUFFER_LEN); // empties the tx_buffer_s to be ready for next transmission
}

// UART receive call back function : this function gets called inside  HAL_UART_IRQHandler(&huart2);v
// and does the evaluating for the UART RX interrupt
// when you call HAL_UART_Transmit_IT
// the UART periphial triggers the rx interrupt when the requested number of bytes arrives.
//  HAL_UART_IRQHandler(&huart2) (inside the HAL library) checks the hardware flags, sees that RX is complete,
//and then calls: HAL_UART_RxCpltCallback(&huart2);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(rx_data_s == '\r') // if we get the enter character,(rx_data_s is where ti will be stored )
		{

			char * utg = "U that guy!";
			if(strcmp(utg, rx_buffer_s) == 0){	// if the typed up sentence matches this
				const char * respond = "They gone find soon enough and by then it will be too late";// assign this response
				strcpy((char *)tx_buffer_s, response);												// to the tx_buffer_s to send it
			}
		else{
			const char * error = " what are you talking about LMAO ";// assign this response to the tx_bufffer_s
			strcpy((char *)tx_buffer_s, error); 					//  ^^
		}

	 	 HAL_UART_Transmit(&huart2, tx_buffer_s, (uint16_t)BUFFER_LEN, 100); // Once the if statement evaluation is done, it will transmit the tx_bufffer_s
	 	 memset(tx_buffer_s, '\0', (size)BUFFER_LEN); // empties the tx_buffer_s to be ready for next transmission
	 	 memset(rx_buffer_s, '\0', (size)BUFFER_LEN); // same for receive data buffer
	 	 counter_s = 0; // reset the counter

		}else{
			// if there is data coming into the rx_buffer that isn't the  "enter" key then it will add it to the rx_buffer
			rx_buffer_s[counter_s] = rx_data_s;
		}
		// gets ready for new data in rx_buffer pointer
		HAL_UART_Receive_IT(&huart,&rx_data_s, 1); // lets the compiler know we are ready to receive with the huart2 h
}//      	  	  	  	  	  	  	  	  	  	  	 // last arguement is the number of bytes to be stored
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
#ifdef USE_FULL_ASSERT
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
