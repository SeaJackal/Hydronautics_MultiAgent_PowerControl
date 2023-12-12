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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
typedef enum
{
	PowerMashineState_OFF,
	PowerMashineState_BoatComputerON,
	PowerMashineState_UWaveON
} PowerMashineState;

typedef enum
{
	MotorMashineState_OFF,
	MotorMashineState_ON,
	MotorMashineState_BLOCKED
} MotorMashineState;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PowerMashineState power_state = PowerMashineState_OFF;
MotorMashineState motor_state = MotorMashineState_OFF;
uint8_t state_changed = 1;

bool button_flag = false;
bool button4 = false;
bool button5 = false;
bool button6 = false;
bool button7 = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(state_changed) //Async control
		{
			switch(motor_state) //Motor State Mashine next state
			{
				case MotorMashineState_ON:
					if(button_flag || button4)
						motor_state = MotorMashineState_BLOCKED;
					else if((HAL_GPIO_ReadPin(GPIOA,RP0_Pin)!=GPIO_PIN_SET) || (HAL_GPIO_ReadPin(GPIOA,RP1_Pin)!=GPIO_PIN_RESET))
						motor_state = MotorMashineState_OFF;
					break;
				case MotorMashineState_OFF:
					if(button_flag || button4)
						motor_state = MotorMashineState_BLOCKED;
					else if((HAL_GPIO_ReadPin(GPIOA,RP0_Pin)==GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOA,RP1_Pin)==GPIO_PIN_RESET))
						motor_state = MotorMashineState_ON;
					break;
				case MotorMashineState_BLOCKED:
					if(button_flag || button5)
						motor_state = MotorMashineState_OFF;
					break;
			}
			
			// Power state mashine next state
			if(button_flag) 
			{
				if(power_state == PowerMashineState_OFF)
					power_state = PowerMashineState_BoatComputerON;
			}
			else if(button5 || button7) 
				power_state = PowerMashineState_BoatComputerON;			
			else if(button6) //STATE 3 ( RP, WIFI, UWB, UWave)
				power_state = PowerMashineState_UWaveON;
			else if(button4) //STATE 5 ( ALL OFF)
				power_state = PowerMashineState_OFF;
			
			state_changed = 0;
			button_flag = false;
			button4 = false;
			button5 = false;
			button6 = false;
			button7 = false;
		}
		else // Sync control (Raspberry Pi)
		{
			switch(motor_state)
			{
				case MotorMashineState_ON:if((HAL_GPIO_ReadPin(GPIOA,RP0_Pin)!=GPIO_PIN_SET) || (HAL_GPIO_ReadPin(GPIOA,RP1_Pin)!=GPIO_PIN_RESET))
						motor_state = MotorMashineState_OFF;
					break;
				case MotorMashineState_OFF:
					if((HAL_GPIO_ReadPin(GPIOA,RP0_Pin)==GPIO_PIN_SET) && (HAL_GPIO_ReadPin(GPIOA,RP1_Pin)==GPIO_PIN_RESET))
						motor_state = MotorMashineState_ON;
					break;
				case MotorMashineState_BLOCKED:
					break;
			}
		}
		// Power state mashine out
		switch(power_state)
		{
			case PowerMashineState_OFF:
				HAL_GPIO_WritePin(GPIOA,EN_5V_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,EN_12V_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,EN_12V_ISO_Pin, GPIO_PIN_RESET);
				break;
			case PowerMashineState_BoatComputerON:
				HAL_GPIO_WritePin(GPIOA,EN_5V_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,EN_12V_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,EN_12V_ISO_Pin, GPIO_PIN_RESET);
				break;
			case PowerMashineState_UWaveON:
				HAL_GPIO_WritePin(GPIOA,EN_5V_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,EN_12V_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,EN_12V_ISO_Pin, GPIO_PIN_SET);
				break;
		}
		// Motor state mashine out
		switch(motor_state)
		{
			case MotorMashineState_ON:
				HAL_GPIO_WritePin(GPIOA,EN_VMA_Pin, GPIO_PIN_SET);
				break;
			case MotorMashineState_OFF:
				HAL_GPIO_WritePin(GPIOA,EN_VMA_Pin, GPIO_PIN_RESET);
				break;
			case MotorMashineState_BLOCKED:
				HAL_GPIO_WritePin(GPIOA,EN_VMA_Pin, GPIO_PIN_RESET);
				break;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_5V_Pin|EN_VMA_Pin|EN_12V_Pin|EN_12V_ISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_5V_Pin EN_VMA_Pin EN_12V_Pin EN_12V_ISO_Pin */
  GPIO_InitStruct.Pin = EN_5V_Pin|EN_VMA_Pin|EN_12V_Pin|EN_12V_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RP0_Pin RP1_Pin */
  GPIO_InitStruct.Pin = RP0_Pin|RP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if(GPIO_Pin == GPIO_PIN_4)
   {
      button4 = true;
   }

   if(GPIO_Pin == GPIO_PIN_5)
   {
	   button5 = true;
   }

   if(GPIO_Pin == GPIO_PIN_6)
   {
	   button6 = true;
   }

   if(GPIO_Pin == GPIO_PIN_7)
   {
	   button7 = true;
   }


   if(GPIO_Pin == GPIO_PIN_3)
   {
	   button_flag = true;
   }
	 
	 state_changed = 1;
}
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
