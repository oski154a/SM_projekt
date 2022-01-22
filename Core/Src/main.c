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
//#include "lcd.h"
#include "LCD_HD44780.h"

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
#define PWM_OUTPUT_TimerType   TIM_HandleTypeDef*
#define PWM_OUTPUT_ChannelType uint16_t


typedef struct {
	PWM_OUTPUT_TimerType Timer;
	PWM_OUTPUT_ChannelType Channel;
	float Duty;
} PWM_OUTPUT_HandleTypeDef;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef PWM_OUTPUT_HandleTypeDef HEATER_HandleTypeDef;

#define HEATER_Init     PWM_OUTPUT_Init
#define HEATER_SetPower PWM_OUTPUT_SetDuty
uint8_t Received[5];
uint8_t Data[5];
uint8_t Received2[5];
uint8_t Data2[16];
char cmd_msg[] = "000\n";
char text[4];
_Bool rx_flag = 0;
uint16_t msg_len;
uint16_t msg_len2;
uint16_t Heater_PWM_Duty;
float temperature_current;
float temperature_reference;
float temperature_error;
float BMP280_press;
float PWM_Control_Heater;
float PWM_Control_Fan;
float control3;
float control_temp;
float uchyb;
int a,b,c,d;
char znak[4];
char znak2[] = "0000";
int power;
HEATER_HandleTypeDef hheater1 = {
		.Timer = &htim3, .Channel = TIM_CHANNEL_1, .Duty = 0
};

const float Tp = 0.009;
const float fs = 1/Tp;
arm_pid_instance_f32 PID;

typedef float float32_t;

typedef struct{
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
	float32_t dt;
}pid_parameters_t;

typedef struct{
	pid_parameters_t p;
	float32_t previous_error, previous_integral;
}PID_TypeDef;

PID_TypeDef pid1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float32_t calculate_discrete_pid(PID_TypeDef* pid, float32_t setpoint, float32_t measured){
	float32_t u=0, P, I, D, error, integral, derivative;

	error = setpoint-measured;

	//proportional part
	P = pid->p.Kp * error;

	//integral part
	integral = pid->previous_integral + (error+pid->previous_error) ; //numerical integrator without anti-windup
	pid->previous_integral = integral;
	I = pid->p.Ki*integral*(pid->p.dt/2.0);

	//derivative part
	derivative = (error - pid->previous_error)/pid->p.dt; //numerical derivative without filter
	pid->previous_error = error;
	D = pid->p.Kd*derivative;

	//sum of all parts
	u = P  + I + D; //without saturation

	return u;
}


void PWM_OUTPUT_Init(PWM_OUTPUT_HandleTypeDef* hpwmout)
{
	HAL_TIM_PWM_Start(hpwmout->Timer, hpwmout->Channel);
}

/**
 * @brief Sets duty of PWM output.
 * @param[in] hpwmout : PWM output handler
 * @param[in] duty    : PWM duty in percents
 * @return None
 */
void PWM_OUTPUT_SetDuty(PWM_OUTPUT_HandleTypeDef* hpwmout, float duty)
{
	hpwmout->Duty = duty;
	int COMPARE = (duty * (__HAL_TIM_GET_AUTORELOAD(hpwmout->Timer)+1)) / 100;
	__HAL_TIM_SET_COMPARE(hpwmout->Timer, hpwmout->Channel, COMPARE);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//	float duty_heater;
	//	float duty_cooler;
	//	double value;



	if(huart->Instance == USART3)
	{
		uint8_t tx_buffer[64];

		if(Data[0]=='C')
		{
			sscanf((char*)&Data[1], "%f", &PWM_Control_Fan);
			if(PWM_Control_Fan>= 0 && PWM_Control_Fan<= 1000)
			{
				//duty_cooler = PWM_Control_Fan/10;
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
				//duty_heater = PWM_Control_Heater/10;
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
		//		else if(isdigit(Data[0]))
		//		{
		//		temperature_reference = atof(Data);
		//		}
		//		else if(scanf("%lf", &Data[0]) == 1)
		//		{
		//			int resp_len = sprintf((char*)tx_buffer, "It's float: %f\n", Data);
		//			HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
		//			HAL_UART_Receive_IT(&huart3, Data, msg_len2);
		//		 //   sprintf("It's float: %f\n", Data);
		//		}
		//		else
		//		{
		//			int resp_len = sprintf((char*)tx_buffer, "It's NOT float ... \n", Data);
		//						HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
		//						HAL_UART_Receive_IT(&huart3, Data, msg_len2);
		//					    sprintf("It's float: %f\n", Data);
		//		//    printf("It's NOT float ... \n");
		//		}
		else
		{
			sscanf((char*)&Data[0], "%f", &temperature_reference);
			int resp_len = sprintf((char*)tx_buffer, "Temp Ref: %f\r\n", temperature_reference);
			HAL_UART_Transmit(&huart3, tx_buffer, resp_len, 10);
			HAL_UART_Receive_IT(&huart3, Data, msg_len);
		}

		//  //	control = atof(cmd_msg);
		//  //	HEATER_SetPower(&hheater1, control);
		//  //	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, control);
		//
		//	//  control2 = atof(cmd_msg);
		////	 	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, control2);
		//	// 	  		HAL_UART_Receive_IT(&huart3, (uint8_t*)cmd_msg, sizeof(cmd_msg));
		//
		//  	rx_flag = 1;
		//
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	{
		char str_buffer[100];
		int n;


		//  				//Pomiar temperatury i ciśnienia
#ifdef BMP2_VER_2021
		float temp = BMP2_ReadTemperature_degC(&hbmp2_1);
		float temp_ref = temperature_reference;

//		uint16_t number_of_samples=1000;
//		float32_t dt=0.001, setpoint=temperature_reference, measured=0, pid_output;
//		pl pid1 = { .p.Kp=0.59, .p.Ki=0.0007, .p.Kd=2.88, .p.dt=dt, .previous_error=0, .previous_integral=0};
//
//		PWM_Control_Heater = 999.0*calculate_discrete_pid(&pid1, setpoint, temperature_current);
//
//					//Saturation limit
//					if(PWM_Control_Heater < 0)
//					{
//						Heater_PWM_Duty = 0;
//					}
//					else if(PWM_Control_Heater > 999.0)
//					{
//						Heater_PWM_Duty = 999;
//					}
//					else
//					{
//						Heater_PWM_Duty = (uint16_t)PWM_Control_Heater;
//					}
//
//					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Heater_PWM_Duty);



		//				float press = BMP2_ReadPressure_hPa(&hbmp2_1);
		//n = sprintf(str_buffer, "{\"Temp\":%2.02f} {\"Press\":%4.02f} \r\n", temp, press);
		n = sprintf(str_buffer, "{\"Current Temperature\": %2.02f *C} {\"Reference Temperature\": %2.02f *C}\r\n", temp, temp_ref);
		//		n = sprintf(str_buffer, "%2.02f\r\n", temp);

#endif

		str_buffer[n] = '\n';
		HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);

		//        temperature_error = temperature_reference - temperature_current;
		//
		//        PWM_Control_Heater = arm_pid_f32(&PID, temperature_error);
		//


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

	HEATER_Init(&hheater1);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	PWM_Control_Fan = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_Control_Fan);

	//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	//	control3 = 0;
	//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, control3);

//			PID.Kp = 6.0;
//		    PID.Ki = 0.001*Tp;
//		    PID.Kd = 3.0/Tp;

	//1.3
	//0.0004
	//3.3

				PID.Kp = 1.3;
			    PID.Ki = 0.001*Tp;
			    PID.Kd = 3.3/Tp;
		    arm_pid_init_f32(&PID, 1);

		    temperature_reference = 30.00;

			uint16_t number_of_samples=1000;
			float32_t dt=0.001, setpoint=temperature_reference, measured=0, pid_output;
			PID_TypeDef pid1 = { .p.Kp=0.59, .p.Ki=0.0007, .p.Kd=2.88, .p.dt=dt, .previous_error=0, .previous_integral=0};


//	LCD_Init(&hlcd1);

	char stringtemp[4];
	msg_len = strlen("C000\r");
	msg_len2 = strlen("H000\r");

#ifdef BMP2_VER_2021
	BMP2_Init(&hbmp2_1);
#endif

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim7);
	//  HAL_UART_Receive_DMA(&huart3, (uint8_t*)cmd_msg, strlen(cmd_msg));
	HAL_UART_Receive_IT(&huart3, Data, msg_len);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	//	LCD_printInt(&hlcd1, 5);

//			LCD_Cls();
//			LCD_Locate(0,0);
//			LCD_String(" STM32");
//			LCD_Locate(0,1);
//			LCD_String("www.msalamon.pl ");

		//ZAD 1-4
		temperature_current = BMP2_ReadTemperature_degC(&hbmp2_1);

//		PWM_Control_Heater = 999.0*calculate_discrete_pid(&pid1, temperature_reference, temperature_current);

		temperature_error = temperature_reference - temperature_current;
		PWM_Control_Heater = 999.0*arm_pid_f32(&PID, temperature_error);
//
//				//Saturation limit
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

		//		LCD_printf(&hlcd1, "test", stringtemp);

		//		temperature_error = temperature_reference - temperature_current;
		//
		//		        PWM_Control_Heater = arm_pid_f32(&PID, temperature_error);
		//
		//		        //Saturation limit
		//		        if(PWM_Control_Heater < 0)
		//		        {
		//		        	Heater_PWM_Duty = 0;
		//		        }
		//		        else if(PWM_Control_Heater > 999.0)
		//		        {
		//		        	Heater_PWM_Duty = 999;
		//		        }
		//		        else
		//		        {
		//		        	Heater_PWM_Duty = (uint16_t)PWM_Control_Heater;
		//		        }
		//
		//				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Heater_PWM_Duty);

		HAL_Delay(10);

		//	  if(HAL_UART_Receive(&huart3, (uint8_t*)cmd_msg, strlen(cmd_msg), 100) == HAL_OK)
		//	  {
		//		  control2 = atof(cmd_msg);
		//		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, control2);
		//	  }
		//  control2 = atof(cmd_msg);
		//	 	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, control2);

		//ZAD 5 TEMP
		//	  if(HAL_UART_Receive(&huart3, (uint8_t*)text, strlen(text), 100) == HAL_OK)
		//	        {
		//	      	  a = text[0] - '0';
		//	      	  b = text[1] - '0';
		//	      	  c = text[2] - '0';
		//	      	  d = text[3] - '0';
		//	      	  control2= a*1000+b*100+c*10+d*1;
		//	      	  if(control2<=1000&&control2>=0)
		//	      	  {
		//
		//	      		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, control2);
		//	     // 		BMP280_temp = BMP2_ReadTemperature_degC(&hbmp2_1);
		//	    //  		HAL_Delay(250);
		//	      	  }
		//	         }

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
