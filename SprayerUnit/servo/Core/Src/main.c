/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include "eepromConf.h"
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
int i=25;
extern uint8_t buffer_rx[2];
int Servo_A_pos=25;
int Servo_B_pos=25;
uint8_t step_delay=70;
uint32_t dat[10]={};
uint8_t byte0=0,byte1=0,byte2=0,byte3=0;
uint32_t str[]={'H','E','L','L','O'};
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_RxCpltCallback(&huart1);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart2,(uint8_t * )"START",5,10);
  HAL_Delay(500);
 /* if(EE_Format())
  {
	  HAL_UART_Transmit(&huart2,(uint8_t * )"ERASE DONE",10,10);
  }
  if(EE_Writes(0x10,5,str))
  {
  HAL_UART_Transmit(&huart2,(uint8_t * )"WRITE DONE",10,10);
  }
  if(EE_Reads(0x10,5,dat))
  {
  byte0 = ((dat[0] >> 24) & 0xFF) ;
  byte1 = ((dat[0] >> 16) & 0xFF) ;
  byte2 = ((dat[0] >> 8 ) & 0XFF);
  byte3 = (dat[0] & 0XFF);
  HAL_UART_Transmit(&huart2,(uint8_t * )"READ DONE : ",12,10);
  HAL_UART_Transmit(&huart2,&byte0,1,10);
  HAL_UART_Transmit(&huart2,&byte1,1,10);
  HAL_UART_Transmit(&huart2,&byte2,1,10);
  HAL_UART_Transmit(&huart2,&byte3,1,10);
  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  AttachDcmotor();
	  AttachServo();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void AttachDcmotor(void)
{

 if(buffer_rx[0]=='5' && buffer_rx[1]=='5')
   {
     if(HAL_GPIO_ReadPin(GPIOA, LIMIT_SW_2_Pin))
     {
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1A_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1B_Pin, GPIO_PIN_SET);
     }
     else
     {
    	 //HAL_GPIO_WritePin(GPIOB, MOTOR_1A_Pin, GPIO_PIN_RESET);
    	// HAL_GPIO_WritePin(GPIOB, MOTOR_1B_Pin, GPIO_PIN_RESET);
    	 HAL_UART_Transmit(&huart2,(uint8_t * )"SW2 Reached\r\n",13,100);
    	 HAL_Delay(100);
     }
   }

 if(buffer_rx[0]=='6' && buffer_rx[1]=='6')
   {
     if(HAL_GPIO_ReadPin(GPIOA, LIMIT_SW_1_Pin))
     {
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1A_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1B_Pin, GPIO_PIN_RESET);

     }
     else
     {
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1A_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1B_Pin, GPIO_PIN_RESET);
    	 HAL_UART_Transmit(&huart2,(uint8_t * )"SW1 Reached\r\n",13,10);
    	 HAL_Delay(100);
      }

    }

 if(buffer_rx[0]=='1' && buffer_rx[1]=='0')
    {
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1A_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(GPIOB, MOTOR_1B_Pin, GPIO_PIN_RESET);
    }
}

void AttachServo(void)
{

   if(buffer_rx[0]=='3' && buffer_rx[1]=='3')
   {
      if(Servo_A_pos>24 && Servo_A_pos<126)
      {

		  htim2.Instance->CCR1 = Servo_A_pos;  // duty cycle is .5 ms
		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		  Servo_A_pos=Servo_A_pos+1;
		  HAL_Delay(step_delay);
		  if(Servo_A_pos==126)
			  {Servo_A_pos=125;}

      }

   }
   else if(buffer_rx[0]=='4' && buffer_rx[1]=='4')
   {
      if(Servo_A_pos>24 && Servo_A_pos<126)
      {

		  htim2.Instance->CCR1 = Servo_A_pos;  // duty cycle is .5 ms
		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		  Servo_A_pos=Servo_A_pos-1;
		  HAL_Delay(step_delay);
		  if(Servo_A_pos==24)
		  { Servo_A_pos=25;}
      }
   }
   else if(buffer_rx[0]=='1' && buffer_rx[1]=='1')
   {
      if(Servo_B_pos>24 && Servo_B_pos<126)
      {

		  htim1.Instance->CCR1 = Servo_B_pos;  // duty cycle is .5 ms
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		  Servo_B_pos=Servo_B_pos+1;
		  HAL_Delay(step_delay);
		  if(Servo_B_pos==126)
		  			  {Servo_B_pos=125;}
      }

   }
   else if(buffer_rx[0]=='2' && buffer_rx[1]=='2')
   {
     if(Servo_B_pos>24 && Servo_B_pos<126)
      {
		 htim1.Instance->CCR1 = Servo_B_pos;  // duty cycle is .5 ms
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		  Servo_B_pos=Servo_B_pos-1;
		  HAL_Delay(step_delay);
		  if(Servo_B_pos==24)
		 		  { Servo_B_pos=25;}

      }
   }

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
