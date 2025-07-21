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
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "include.h"
#include "F_Resolve.h"
#define PID_M_PI 3.1415926
float tran_coef = PID_M_PI*60;

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
float w1;
float w2;
float w3;
void pid_pos_classic_1(float set_pos_x,float set_pos_y){
	w1 = set_pos_x/tran_coef;
	w2 = -set_pos_x*cos(PID_M_PI/3)/tran_coef -set_pos_y*sin(PID_M_PI/3)/tran_coef;
	w3 = -set_pos_x*cos(PID_M_PI/3)/tran_coef +set_pos_y*sin(PID_M_PI/3)/tran_coef;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
extern uint8_t step;
	PID_devices_Init();
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	
	CAN_Start(&hcan1);
	CAN_Start(&hcan2);
	HAL_TIM_Base_Start_IT(&htim1);
	//**********************************
//	velocity t_v={0,0};
//	velocity *ptr_t_v = &t_v;
//	double vl_omega = 0;
//	set_velocity(&t_v,1000,2000);
//	set_vl_omega(vl_omega,0);
//	set_vl_wheel(vl_wheel_ave,t_v,0,0);
	//*************************************

	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  delta_move();
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		pid_pos_classic_1(1000.0f,0);
    CAN1_CMD_1(pid_call_1(w1*8191*3591/187,1),pid_call_1(w2*8191*3591/187,2),pid_call_1(w3*8191*3591/187,3),0);
		HAL_Delay(1); 

//		  CAN1_CMD_1(0,0,pid_call_1(15000,3),0);
		//*********************************************
//��������
//		 CAN1_CMD_1(PID_velocity_realize_1(vl_wheel_ave[0],1),
//		            PID_velocity_realize_1(vl_wheel_ave[1],2),
//		            PID_velocity_realize_1(vl_wheel_ave[2],3),0);

//		  HAL_Delay(1);              //��ʱ����̫��
		//***************************************************
		//delta����
//		c
		//***************************************************
//		if(step>=1&&step<=5)
//		{		
//	    CAN1_CMD_1(pid_call_1(-15910,1),pid_call_1(-15910,2),pid_call_1(-15910,3),0);
//		}
//		
//		else if(step>=10&&step<=20)
//		{		
//	    CAN1_CMD_1(pid_call_1(-62103,1),pid_call_1(-32103,2),pid_call_1(-52103,3),0);
//		}
//		
//		else if(step>=30&&step<=40)
//		{		
//      CAN1_CMD_1(pid_call_1(-15910,1),pid_call_1(-15910,2),pid_call_1(-15910,3),0);
//		}
//		else if(step>40&&step<=89)
//		{		
//      CAN1_CMD_1(pid_call_1(0,1),pid_call_1(0,2),pid_call_1(0,3),0);
//		}
//		

		//****************************************************************
//		if(step==0)
//		{		
//			CAN1_CMD_1(0,0,0,pid_call_1(-15910,4));
//		}
//		
//		else if(step==1)
//		{		
//			CAN1_CMD_1(0,0,0,pid_call_1(-32103,4));
//		}
//		
//		else if(step==2)
//		{		
//			CAN1_CMD_1(0,0,0,pid_call_1(-47341,4));
//		}
//		
//		else if(step==3)
//		{		
//			CAN1_CMD_1(0,0,0,pid_call_1(-32103,4));
//		}
//		
//		else if(step==4)
//		{		
//			CAN1_CMD_1(0,0,0,pid_call_1(-15910,4));
//		}
		//*******************************************************************
// CAN1_CMD_1(pid_call_1(3000,1),pid_call_1(3000,2),pid_call_1(3000,3),0);
// CAN1_CMD_1(0,0,0,pid_call_1(3000,4));
//  CAN1_CMD_1(0,0,0,0);
//     CAN1_CMD_1(pid_call_1(-32103,1),pid_call_1(-32103,2),pid_call_1(-32103,3),0);
		 HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
