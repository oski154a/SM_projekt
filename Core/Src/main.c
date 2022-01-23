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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>

#include "arm_math.h"
//#include "lcd_config.h"
//#include "lcd2.h"
//#include "LCD_HD44780.h"
#include "LCD.h"

#define BMP2_VER_2021

#ifdef BMP2_VER_2021
#include "bmp2_config.h"
#endif
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
uint8_t Received[5];
uint8_t Data[5];
uint16_t msg_len;
uint16_t msg_len2;
uint16_t Heater_PWM_Duty;
float temperature_current;
float temperature_reference;
float temperature_error;
float BMP280_press;
float PWM_Control_Heater;
float PWM_Control_Fan;
const float Tp = 0.009;
char deg_symbol = 248;

arm_pid_instance_f32 PID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		uint8_t tx_buffer[32];

		if(Data[0]=='C')
		{
			sscanf((char*)&Data[1], "%f", &PWM_Control_Fan);
			if(PWM_Control_Fan>= 0 && PWM_Control_Fan<= 1000)
			{
				int resp_len = sprintf((char*)tx_buffer, "Fan DUTY: %i%%\r\n", (int)(PWM_Control_Fan/10));
				HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
				HAL_UART_Receive_IT(&huart3, Data, msg_len);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_Control_Fan);
			}
			else
			{
				int resp_len = sprintf((char*)tx_buffer, "Wrong DUTY\r\n");
				HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
				HAL_UART_Receive_IT(&huart3, Data, msg_len);
			}
		}
		else if(Data[0]=='H')
		{
			sscanf((char*)&Data[1], "%f", &PWM_Control_Heater);
			if(PWM_Control_Heater>= 0 && PWM_Control_Heater<= 1000)
			{
				int resp_len = sprintf((char*)tx_buffer, "Heater DUTY: %i%%\r\n", (int)(PWM_Control_Heater/10));
				HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
				HAL_UART_Receive_IT(&huart3, Data, msg_len2);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_Control_Heater);
			}
			else
			{
				int resp_len = sprintf((char*)tx_buffer, "Wrong DUTY\r\n");
				HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
				HAL_UART_Receive_IT(&huart3, Data, msg_len);
			}

		}
		else
		{
			sscanf((char*)&Data[0], "%f", &temperature_reference);
			int resp_len = sprintf((char*)tx_buffer, "Temp Ref: %f\r\n", temperature_reference);
			HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
			HAL_UART_Receive_IT(&huart3, Data, msg_len);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		char str_buffer[100];
		int n;

#ifdef BMP2_VER_2021
		float temp = BMP2_ReadTemperature_degC(&hbmp2_1);
		float temp_ref = temperature_reference;
		n = sprintf(str_buffer, "{\"Current Temperature\": %2.02f *C} {\"Reference Temperature\": %2.02f *C}\r\n", temp, temp_ref);
#endif

		str_buffer[n] = '\n';
		HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);
	}
}

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	PWM_Control_Heater = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_Control_Heater);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	PWM_Control_Fan = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_Control_Fan);

	PID.Kp = 1.3;
	PID.Ki = 0.001*Tp;
	PID.Kd = 3.3/Tp;
	arm_pid_init_f32(&PID, 1);

	temperature_reference = 30.00;

	LCD_init();
	LCD_write_command(LCD_CLEAR_INSTRUCTION);
//	LCD_write_text("Temp ref: ");
//	LCD_printf("%f",temperature_reference);
//	LCD_goto_line(1);
//	LCD_write_text("Temp cur: ");
//	LCD_printf("%f",temperature_current);
//	LCD_printf("temp: ", "%f",temperature_reference);
	msg_len = strlen("C000\r");
	msg_len2 = strlen("H000\r");

#ifdef BMP2_VER_2021
	BMP2_Init(&hbmp2_1);
#endif

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_UART_Receive_IT(&huart3, Data, msg_len);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	//	LCD_write_data(temperature_reference);
	//	LCD_printf_line("tempref");
		//	LCD_write_text("Temp ref");
		//	LCD_goto_line(1);
	//		LCD_write_text("Temp cur");

		//	LCD_printInt(&hlcd1, 5);

		//			LCD_Cls();
		//			LCD_Locate(0,0);
		//			LCD_String(" STM32");
		//			LCD_Locate(0,1);
		//			LCD_String("www.msalamon.pl ");

		temperature_current = BMP2_ReadTemperature_degC(&hbmp2_1);
		temperature_error = temperature_reference - temperature_current;
		PWM_Control_Heater = 999.0*arm_pid_f32(&PID, temperature_error);

		//Saturation limit
		if(PWM_Control_Heater < 0)
		{
			Heater_PWM_Duty = 0;
		}
		else if(PWM_Control_Heater > 999.0)
		{
			Heater_PWM_Duty = 999;
		}
		else
		{
			Heater_PWM_Duty = (uint16_t)PWM_Control_Heater;
		}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Heater_PWM_Duty);

		//				if(temperature_reference < temperature_current)
		//				{
		//					PWM_Control_Fan = 1000;
		//					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_Control_Fan);
		//				}
		//				else if(temperature_reference > temperature_current)
		//				{
		//					PWM_Control_Fan = 0;
		//					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_Control_Fan);
		//				}

			LCD_write_text("Temp ref:");
			LCD_printf("%2.02f",temperature_reference);
			LCD_printf(" C");
			LCD_goto_line(1);
			LCD_write_text("Temp cur:");
			LCD_printf("%2.02f",temperature_current);
			LCD_write_text(" C");
			LCD_goto_line(2);

		HAL_Delay(10);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
