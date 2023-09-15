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
#include "math.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include "FLASH_SECTOR_F4.h"
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */

	/******** FLOW SENSOR **********/
	float Flow, sum_Flow, Flow_final;
	int pulse, sizeF_filter = 0;
	int time_count;
	int pulse_done = 0;
	
	uint16_t encoder_flow = 0;
	unsigned long currentTime=0;
	unsigned long cloopTime=0;
  float l_per_min;
	
	/***** THOI GIAN SETUP ******/
		uint16_t time_xay /*= 19000*/; //thoi gian thuc la 21s
		uint16_t time_u_cafe /*= 10000*/; //thoi gian thuc la 8s
		uint16_t time_xa_cafe /*= 20000*/; //thoi gian thuc la 30s


	/***** PWM MOTOR ****/
	int PWM_step = 100;
	int PWM_step_slow = 50;
	int PWM_bom = 380;
	

 /********************** TEMPATURE VAR *********************/
		/* adc vars */
		uint16_t ADC_raw[3];
		uint16_t ADC_raw_Tcoi[2];
		int counter_tim4 = 0;
		
		/* ntc vars */
		float Ntc_temp = 0, Ntc_temp_coi = 0;
		float Ntc_R, Ntc_R_coi;
		float Temp_sum[100], Tempcoi_sum_10[20]; 
		float TEMP, TEMP_coi;
		float size = 20.0;
		int counter_filter = 0, counter_filter_coi = 0;
		float sum=0, sum_tempcoi=0;
		
		/* R1 resistance */
		#define NTC_UP_R 10000.0f
		
		/* constants of Steinhart-Hart equation */
		#define A 0.000857096f
		#define B 0.000260663f
		#define D 0.000000123f
		
		uint8_t Sch_ms = 255;
/************************************************************/
 
	/********* PID TEMPATURE DEFINATION *********/
	/*** BOILER PID TEMPATURE ***/
		float error = 0;
		float Kp = 1200/*1800.0*/, Ki = 120.0 , Kd = 100.0;
		float	Pre_error, pre_pre_Error; 
		float Output, pre_output;
		float P_part, I_part, D_part;
		double Total;
		
	/*** COI PID TEMPATURE ***/
		float error_coi = 0.0;
		float Kp1 = 800.0/*1800.0*/, Ki1 = 120.0 , Kd1 = 100.0;
		float	Pre_error_coi, pre_pre_Error_coi; 
		float Output_coi, pre_output_coi;
		float P_part_coi, I_part_coi, D_part_coi;
		double Total_coi;
		
		#define T_sample 0.1
	/*********************************************/

	/************** STEPPER MOTOR VAR *************/
		//0.34s quay 1 vong
		float count=0;
		float x1=800*36, x2=800*55.75, x3=800*56.25, delta_x;
		float x4 = 800*55.75, x5=800*85;
		int count_tim1 = 0;
		uint8_t atx1 = 0, atx2 = 0, atx3 =0, atx4=0, atx5=0, count_wash=0;
	/*********************************************/
	
	/*************** PRESSURE VALUE ****************/
		float Pressure_val;
		int Press_raw = 0;
		float Max_P = 1.4, Min_P = 0.0;
		float Min_ADC = 455.0, Max_ADC = 4095.0;
		int filter_count_P = 0, size_filter = 15;
		float P_value_sum[15];
		float Sum_P = 0, P_bar;
	/***********************************************/
	
	/*************** DWIN GUI VALUE *********************/
		#define RxData_SIZE		9
		#define MainBuf_SIZE	36

		uint8_t DataStorage[4]={92, 19, 8, 25};
		uint8_t RxData[RxData_SIZE];
		uint8_t MainBuf[MainBuf_SIZE];
		uint8_t TxData[]={0x5A, 0xA5, 0x05, 0x82, 0x50, 0x00, 0x00, 0x00,0x5A, 0xA5, 0x05, 0x82, 0x51, 0x00, 0x00, 0x00};
		uint8_t TxData_Graph[]={0x5A, 0xA5, 0x13, 0x82, 0x03, 0x10, 0x5A, 0xA5, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08,
														0x03, 0x02, 0x00, 0x00, 0x00, 0x03, 0x5A, 0xA5, 0x05, 0x82, 0x50, 0x00, 0x00, 0x00,
														0x5A, 0xA5, 0x05, 0x82, 0x51, 0x00, 0x00, 0x00, 0x5A, 0xA5, 0x05, 0x82, 0x52, 0x00, 0x00, 0x00};
		
		uint8_t ClearGraph_VarDisplay[]={0x5A, 0xA5, 0x05, 0x82, 0x50, 0x00, 0x00, 0x00,0x5A, 0xA5, 0x05, 0x82, 0x51, 0x00, 0x00, 0x00,
																		0x5A, 0xA5, 0x05, 0x82, 0x52, 0x00, 0x00, 0x00,
																		0x5A, 0xA5, 0x05, 0x82, 0x03, 0x01, 0x00, 0x00, 0x5A, 0xA5, 0x05, 0x82, 0x03, 0x07, 0x00, 0x00};
		
																		
		uint8_t TxDataSend1[]={0x5A, 0xA5, 0x05, 0x82, 0x20, 0x00, 0x00, 0x00, 
													 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x02, 0x00, 0x00, 
													 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x04, 0x00, 0x00,
													 0x5A, 0xA5, 0x05, 0x82, 0x20, 0x06, 0x00, 0x00};
		
		uint8_t BackToPage0Cmd[]={0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, 0x00};
													 
		int conTx=0;
		uint8_t countSend1times=0;
		volatile uint32_t count_DMA=0;
		volatile uint32_t count_TX=0;

		uint16_t oldPos = 0;
		uint16_t newPos = 0;

		int turn_off = 1, turn_on=0;
		int TEMP_int, P_bar_int, Flow_final_int;
		int change_to_page=0;
		int tedone=0, predone=0;

		uint8_t highByte, lowByte;
																		
		int TEMP_setup=0, Time_xay_cafe=0, Time_xa_cafe=0, Time_u_cafe = 0;
																		
		uint8_t Data_read_flash[10];
		uint32_t FLASH_ADD = 0x08010000;
		uint8_t xay, steam, water, water_vale, run_test;
	/**********************************************/
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM11_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM2_slow_F()
{
	  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */


  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

void send_data_1time()
{
	TxDataSend1[7] = time_xay/100;
	TxDataSend1[15] = time_u_cafe/100;
	TxDataSend1[23] = time_xa_cafe/100;
	TxDataSend1[31] = TEMP_setup;
	HAL_UART_Transmit(&huart5, TxDataSend1, sizeof(TxDataSend1), 100);
}
void send_data()
{
	if(change_to_page==0)
	{
		TEMP_int = TEMP*(pow(10,1));
		P_bar_int = P_bar*(pow(10,1));
		TxData[6]=(uint8_t)((TEMP_int>>8) & 0xFF);
		TxData[7]=(uint8_t)((TEMP_int>>0) & 0xFF);
		TxData[14]=(uint8_t)((P_bar_int>>8) & 0xFF);
		TxData[15]=(uint8_t)((P_bar_int>>0) & 0xFF);
		HAL_UART_Transmit_DMA(&huart5, TxData, sizeof(TxData));
	}
	
	if(change_to_page==2 && atx2==0)
	{
		Flow_final_int = Flow_final*(pow(10,1));
		P_bar_int = P_bar*(pow(10,1));
		TEMP_int = TEMP*(pow(10,1));
		ClearGraph_VarDisplay[6]=(uint8_t)((Flow_final_int>>8) & 0xFF);
		ClearGraph_VarDisplay[7]=(uint8_t)((Flow_final_int>>0) & 0xFF);
		ClearGraph_VarDisplay[14]=(uint8_t)((P_bar_int>>8) & 0xFF);
		ClearGraph_VarDisplay[15]=(uint8_t)((P_bar_int>>0) & 0xFF);
		ClearGraph_VarDisplay[22]=(uint8_t)((TEMP_int>>8) & 0xFF);
		ClearGraph_VarDisplay[23]=(uint8_t)((TEMP_int>>0) & 0xFF);
		HAL_UART_Transmit_DMA(&huart5, ClearGraph_VarDisplay, sizeof(ClearGraph_VarDisplay));
	}
	
	if(change_to_page==2 && atx2==1)
	{
		Flow_final_int = Flow_final*(pow(10,1));
		P_bar_int = P_bar*(pow(10,1));
		TEMP_int = TEMP*(pow(10,1));
		//Draw Graph
		TxData_Graph[12]=(uint8_t)((P_bar_int>>8) & 0xFF);
		TxData_Graph[13]=(uint8_t)((P_bar_int>>0) & 0xFF);
		TxData_Graph[14]=(uint8_t)((P_bar_int>>8) & 0xFF);
		TxData_Graph[15]=(uint8_t)((P_bar_int>>0) & 0xFF);
		TxData_Graph[18]=(uint8_t)((Flow_final_int>>8) & 0xFF);
		TxData_Graph[19]=(uint8_t)((Flow_final_int>>0) & 0xFF);
		TxData_Graph[20]=(uint8_t)((Flow_final_int>>8) & 0xFF);
		TxData_Graph[21]=(uint8_t)((Flow_final_int>>0) & 0xFF);
		//Display variables
		TxData_Graph[28]=(uint8_t)((Flow_final_int>>8) & 0xFF);
		TxData_Graph[29]=(uint8_t)((Flow_final_int>>0) & 0xFF);
		TxData_Graph[36]=(uint8_t)((P_bar_int>>8) & 0xFF);
		TxData_Graph[37]=(uint8_t)((P_bar_int>>0) & 0xFF);
		TxData_Graph[44]=(uint8_t)((TEMP_int>>8) & 0xFF);
		TxData_Graph[45]=(uint8_t)((TEMP_int>>0) & 0xFF);
		
		HAL_UART_Transmit_DMA(&huart5, TxData_Graph, sizeof(TxData_Graph));
	}
}


double PID_control_temperature1(float current, float setpoint)
{
	error = setpoint - current;
	P_part = Kp * error;
	//Total = Total + error*T_sample;
	I_part = 0.5 * Ki * T_sample * (error + Pre_error);
	D_part = (Kd/T_sample)*(error - Pre_error);
	
	Output =  P_part + I_part + D_part ;
	//pre_pre_Error = Pre_error;
	Pre_error = error;
	
	if (Output>=10000) Output=9500;	
	if (Output<0)  Output=0;	
	
//	pre_output = Output;
	
	return Output;
}

double PID_control_temperature_coi(float current1, float setpoint1)
{
	error_coi = setpoint1 - current1;
	P_part_coi = Kp1 * error_coi;
	//Total = Total + error*T_sample;
	I_part_coi = 0.5 * Ki1 * T_sample * (error_coi + Pre_error_coi);
	D_part_coi = (Kd1/T_sample)*(error_coi - Pre_error_coi);
	
	Output_coi =  P_part_coi + I_part_coi + D_part_coi ;
	//pre_pre_Error = Pre_error;
	Pre_error_coi = error_coi;
	
	if (Output_coi>=10000.0) Output_coi=9500;	
	if (Output_coi<0.0)  Output_coi=0;	
	
//	pre_output = Output;
	
	return Output_coi;
}

void delay_timer11_ms(__IO uint32_t nCount)
{
		while(nCount!=0)
		{
			HAL_TIM_Base_Start_IT(&htim11);
			__HAL_TIM_SET_COUNTER(&htim11,0);
			while(__HAL_TIM_GET_COUNTER(&htim11)<=1001)
			{ }
			nCount--;
		}
}


void VesinhMay()
{
		/************** MO VAN RUA *****************/
	HAL_TIM_Base_Stop(&htim2);
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)!=0 && run_test==1/*HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==1*/ 
			/*&& HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0*/)
		{
			HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

			while(count<x4)
			{
				HAL_TIM_Base_Start_IT(&htim2);
				TIM2->CCR1=PWM_step;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			}
			if(atx4==1)
			{
				HAL_TIM_Base_Stop_IT(&htim2);
				TIM2->CCR1=0;
				count=x4;
				
				//Mo voi nuoc
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
				//Mo van rua
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
				//Mo bom
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
				TIM2->CCR2=PWM_bom;
				HAL_Delay(5000);
				//Ngat voi nuoc
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
				//Ngat van rua
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
				//Ngat bom
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
				TIM2->CCR2=0;
			}
			
			while(count_wash<2)
			{
				while(count<x5)
				{
					HAL_TIM_Base_Start_IT(&htim2);
					TIM2->CCR1=PWM_step;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				}
				if(atx5==1)
				{
					HAL_TIM_Base_Stop_IT(&htim2);
					TIM2->CCR1=0;
					count=x4;
					atx4=0;
					atx5=0;
					HAL_Delay(1000);
				}
				while(count<x5)
				{
					HAL_TIM_Base_Start_IT(&htim2);
					TIM2->CCR1=PWM_step;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				}
				if(atx5==1)
				{
					HAL_TIM_Base_Stop_IT(&htim2);
					TIM2->CCR1=0;
					count=x4;
					atx5=0;
					//Ngat voi nuoc
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
					//Ngat van rua
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					//Ngat bom
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
					TIM2->CCR2=PWM_bom;
					HAL_Delay(5000);
					//Ngat voi nuoc
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
					//Ngat van rua
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					//Ngat bom
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
					TIM2->CCR2=0;
				}
				while(count<x5)
				{
					HAL_TIM_Base_Start_IT(&htim2);
					TIM2->CCR1=PWM_step;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				}
				if(atx5==1)
				{
					HAL_TIM_Base_Stop_IT(&htim2);
					TIM2->CCR1=0;
					count=x4;
					atx5=0;
					HAL_Delay(1000);
				}
				count_wash++;
				if(count_wash<2)
				{
					while(count<x5)
					{
						HAL_TIM_Base_Start_IT(&htim2);
						TIM2->CCR1=PWM_step;
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
					}
					if(atx5==1)
					{
						HAL_TIM_Base_Stop_IT(&htim2);
						TIM2->CCR1=0;
						count=x4;
						//Mo voi nuoc
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
						//Mo van rua
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
						//Mo bom
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
						TIM2->CCR2=PWM_bom;
						HAL_Delay(5000);
						//Ngat voi nuoc
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
						//Ngat van rua
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
						//Ngat bom
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						TIM2->CCR2=0;
					}
				}
			}
	
				//Ngat van xa va bom
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); // ngat voi nuoc
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //ngat van xa cafe
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); //ngat van xa cafe
				HAL_Delay(200);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); //ngat bom
				TIM2->CCR2=0;
			count_wash=0;
			run_test=0;
	}
}

void Pha_cafe()
{
		/***Go back Home***/
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)==0 && turn_off == 1 && turn_on == 0) 
		{
			//htim2.Init.Period = 200-1;
			//HAL_TIM_Base_Start_IT(&htim2);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			TIM2->CCR1=PWM_step;
		}
		
		/***** May da ve home ****/
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)!=0)
		{
			TIM2->CCR1=0;
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
			count = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		
		/***Start button pressed***/ 
		if(turn_on==1 && turn_off==0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)!=0) 
		{
//		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)!=0 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==0
//			&& HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)!=1) 
//		{
		
			HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
			
			//chay toi vi tri nhan cafe (vi tri x1)
			while(count<x1)
			{
				HAL_TIM_Base_Start_IT(&htim2);
				TIM2->CCR1=PWM_step;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			}
			if(atx1==1)
			{
				TIM2->CCR1=0;
				HAL_TIM_Base_Stop_IT(&htim2);
				count=x1;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); //Xay cafe 
				HAL_Delay(time_xay);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			
			HAL_Delay(1000);
		
			//ep cafe (vi tri x2)
			while(count<x2)
			{
				HAL_TIM_Base_Start_IT(&htim2);
				TIM2->CCR1=PWM_step;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			}
			if(atx2==1)
			{
				HAL_TIM_Base_Stop_IT(&htim2);
				TIM2->CCR1=0;
				count=x2;
			}
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);  
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); //bat van nuoc tu boiler

			HAL_Delay(2000);
			
			//lui ve 1 doan
			while(count<x3)
			{
				HAL_TIM_Base_Start_IT(&htim2);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				TIM2->CCR1=PWM_step;
			}
			if(atx3==1)
			{
				TIM2->CCR1=0;
				HAL_TIM_Base_Stop_IT(&htim2);
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //bat van xa cafe
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//				TIM2->CCR2=PWM_bom;
//				HAL_Delay(2000);
//				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//				TIM2->CCR2=0;
				
				HAL_Delay(time_u_cafe); //thoi gian u cafe
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //bat van xa cafe
				//bat bom
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
				for(int j = 0; j<=PWM_bom; j+=1)
				{
					TIM2->CCR2=j;	
					HAL_Delay(10);
				}
				
				HAL_Delay(time_xa_cafe - 3000 - 4000);
				
				for(int j=PWM_bom; j>=0; j--)
				{
					TIM2->CCR2 = j;
					HAL_Delay(10);
				}
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_Delay(2000);

				//ngat cac van roi chay ve home
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
				HAL_Delay(3000);
				HAL_UART_Transmit(&huart5, BackToPage0Cmd, sizeof(BackToPage0Cmd), 100);
				change_to_page=0;
				turn_off = 1;
				turn_on = 0;
				atx1=0;
				atx2=0;
				atx3=0;
			}
		}

}

void read_pressure()
{
	Press_raw = ADC_raw[2];
	Pressure_val = (Max_P*(Press_raw - Min_ADC) + Min_P*(Max_ADC - Press_raw)) / (Max_ADC - Min_ADC) ;
	
	P_value_sum[filter_count_P] = Pressure_val;
	filter_count_P++;
	
	for(int i = 0; i<size_filter; i++)
	{
		Sum_P += P_value_sum[i];
	}
	if(filter_count_P >= size_filter)
	{
		filter_count_P = 0;
	}
	P_bar = (Sum_P/size_filter)*10.0;
	Sum_P = 0;
}

void read_temp_coi()
{
			/* calc ntc res */
			Ntc_R_coi = ((NTC_UP_R)/((4095.0/ADC_raw[1]) - 1));
			
			/* temp */ 
			float Ntc_Ln_coi = log(Ntc_R_coi);
			
			/* calc temperature */
			Ntc_temp_coi = (1.0/(A + B*Ntc_Ln_coi + D*Ntc_Ln_coi*Ntc_Ln_coi*Ntc_Ln_coi)) - 273.5;
			
			Tempcoi_sum_10[counter_filter_coi] = Ntc_temp_coi;
			counter_filter_coi++;
	
			for(int i=0; i<size; i++)
			{
				sum_tempcoi += Tempcoi_sum_10[i];
			}
		  if(counter_filter_coi>=size)
			{
				counter_filter_coi=0;
			}		
			TEMP_coi=sum_tempcoi/size;
			sum_tempcoi=0;
			
}

void read_Temp_boiler()
{
	/* calc ntc res */
			Ntc_R = ((NTC_UP_R)/((4095.0/ADC_raw[0]) - 1));
			
			/* temp */ 
			float Ntc_Ln = log(Ntc_R);
			
			/* calc temperature */
			Ntc_temp = (1.0/(A + B*Ntc_Ln + D*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.5;
			
			Temp_sum[counter_filter] = Ntc_temp;
			sum += Temp_sum[counter_filter];
			counter_filter++;

		  if(counter_filter>=100)
			{
				counter_filter=0;
				TEMP=sum/100;
				sum=0;
			}		
			PID_control_temperature1(TEMP, 92);
			TIM3->CCR3 = Output;
}

void Debug()
{
	/****** Bat XAY ******/
	if(xay==1)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
	else if(xay==0)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
	
	/*** Mo van Boiler kem nuoc ***/
	if(water==1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		TIM2->CCR2 = PWM_bom;
	}
	else if(water==0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		TIM2->CCR2 = 0;
	}
	
	/****** Mo van Boiler xa bot ap ******/
	if(steam==1) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	else if(steam==0)	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

	/*** Mo van rua co mo nuoc ***/
	if(water_vale==1)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		TIM2->CCR2 = PWM_bom;
	}
	else if(water_vale==0)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		TIM2->CCR2 = 0;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_UART5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start(&hadc1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim6);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Stop(&htim2);
	
		//Doc Flash
		FLASH_ADD = 0x08010000;
		HAL_FLASH_Unlock();
		for(int i = 0; i<=4; i++)
		{
			Data_read_flash[i]=*(__IO uint8_t*)FLASH_ADD;
			FLASH_ADD++;
		}
		TEMP_setup = Data_read_flash[0];
		time_xay = Data_read_flash[1]*100;
		time_u_cafe = Data_read_flash[2]*100;
		time_xa_cafe = Data_read_flash[3]*100;
				
		FLASH_ADD = 0x08010000;
		HAL_FLASH_Lock();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Pha_cafe();
		VesinhMay();
		Debug();
	/************** MO VAN XA BOT AP *************/
//		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)==1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)!=0)
//		{
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
////			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
////			TIM2->CCR2=PWM_bom;
//		}
//		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)==0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)!=0)
//		{
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//			TIM2->CCR2=PWM_bom;
//		}
	
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//			TIM2->CCR2=PWM_bom;
//			HAL_Delay(5000);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//			TIM2->CCR2=0;
//			HAL_Delay(5000);
		
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */


  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 840-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 25000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 10000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 84-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD0 PD1 PD3
                           PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		count++;
		if(count>=x1) atx1=1;
		if(count>=x2) atx2=1;
		if(count>=x3) atx3=1;
		if(count>=x4) atx4=1;
		if(count>=x5) atx5=1;
	}
	
	if(htim->Instance == TIM5)
	{
			Flow = pulse*1.0;
			pulse = 0;
			time_count = 0;

			Flow_final = Flow*100.668/200.0; // ml/s
	}
	
	if(htim->Instance == TIM4)
	{
		/* ADC DMA Start */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_raw, 3);
		read_temp_coi();
		read_pressure();
		read_Temp_boiler();
		
	}
	
	if(htim->Instance == TIM6)
	{
		send_data();
	}
	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	count_TX++;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, MainBuf, MainBuf_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	count_DMA++;
	
	for(int i=0; i<=MainBuf_SIZE; i++)
	{
		if((MainBuf[i]==0x04 && MainBuf[i+1]==0x00) || (MainBuf[i]==0x04 && MainBuf[i+1]==0x5A) 
			|| (MainBuf[i]==0x08 && MainBuf[i+1]==0x04)) change_to_page=1;
		if((MainBuf[i]==0x11 && MainBuf[i+1]==0x06) || (MainBuf[i]==0x11 && MainBuf[i+1]==0x05))	
		{
			change_to_page=2;
			turn_on=1;
			turn_off=0;
		}
		if(MainBuf[i]==0x11 && MainBuf[i+1]==0x07) change_to_page=3;
	}
	
	if(change_to_page==1)
	{
		if(countSend1times<1)
		{
			send_data_1time();
			countSend1times=1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,MainBuf,MainBuf_SIZE);
		for(int i=0; i<=MainBuf_SIZE; i++)
		{
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x02)	
			{
				change_to_page=0;
				countSend1times=0;
			}
			if(MainBuf[i]==0x20 && MainBuf[i+1]==0x06)	TEMP_setup=MainBuf[i+4];
			if(MainBuf[i]==0x20 && MainBuf[i+1]==0x04)	Time_xa_cafe=MainBuf[i+4];
			if(MainBuf[i]==0x20 && MainBuf[i+1]==0x02)	Time_u_cafe=MainBuf[i+4];
			if(MainBuf[i]==0x06 && MainBuf[i+2]==0x20 && MainBuf[i+3]==0x00)	Time_xay_cafe=MainBuf[i+6];
			
			
			if((MainBuf[i]==0x04 && MainBuf[i+1]==0xA5) || (MainBuf[i]==0x04 && MainBuf[i+2]==0x5B) 
				|| (MainBuf[i]==0x04 && MainBuf[i+2]==0x5D) || (MainBuf[i]==0x60 && MainBuf[i+1]==0x01))
			{
				//Setting Screen receive Data
				if(TEMP_setup!=0)
				{
					DataStorage[0]=TEMP_setup;
				}
				else if(TEMP_setup==0)
				{
					FLASH_ADD = 0x08010000;
					HAL_FLASH_Unlock();
					DataStorage[0]=*(__IO uint8_t*)FLASH_ADD;
				}
				if(Time_xay_cafe!=0)
				{
					DataStorage[1]=Time_xay_cafe;
				}
				else if(Time_xay_cafe==0)
				{
					FLASH_ADD = 0x08010001;
					HAL_FLASH_Unlock();
					DataStorage[1]=*(__IO uint8_t*)FLASH_ADD;
				}
				if(Time_u_cafe!=0)
				{
					DataStorage[2]=Time_u_cafe;
				}
				else if(Time_u_cafe==0)
				{
					FLASH_ADD = 0x08010002;
					HAL_FLASH_Unlock();
					DataStorage[2]=*(__IO uint8_t*)FLASH_ADD;
				}
				if(Time_xa_cafe!=0)
				{
					DataStorage[3]=Time_xa_cafe;
				}
				else if(Time_xa_cafe==0)
				{
					FLASH_ADD = 0x08010003;
					HAL_FLASH_Unlock();
					DataStorage[3]=*(__IO uint8_t*)FLASH_ADD;
				}
				
				//Ghi vo Flash
				HAL_FLASH_Unlock();
				FLASH_ADD = 0x08010000;
				FLASH_Erase_Sector(4, FLASH_VOLTAGE_RANGE_3); 
				for(int i = 0; i<=4; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADD, DataStorage[i]);
					FLASH_ADD++;
				}
				HAL_FLASH_Lock();
				FLASH_ADD = 0x08010000;
				
				//Doc Flash
				FLASH_ADD = 0x08010000;
				HAL_FLASH_Unlock();
				for(int i = 0; i<=4; i++)
				{
					Data_read_flash[i]=*(__IO uint8_t*)FLASH_ADD;
					FLASH_ADD++;
				}
				TEMP_setup = Data_read_flash[0];
				time_xay = Data_read_flash[1]*100;
				time_u_cafe = Data_read_flash[2]*100;
				time_xa_cafe = Data_read_flash[3]*100;
				
				FLASH_ADD = 0x08010000;
				HAL_FLASH_Lock();

			}
		}
		
	}
	
	if(change_to_page==2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,MainBuf,MainBuf_SIZE);
		for(int i=0; i<=MainBuf_SIZE; i++)
		{
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x04)	change_to_page=0;
		}
		memset(MainBuf,0,MainBuf_SIZE);
	}
	
	if(change_to_page==3)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,MainBuf,MainBuf_SIZE);
		for(int i=0; i<=MainBuf_SIZE; i++)
		{
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x02)	change_to_page=0;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x08)	xay=1;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x0C) 	xay=0;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x09)	water=1;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x0D) 	water=0;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x0A)	steam=1;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x0E) 	steam=0;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x0B)	water_vale=1;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x0F) 	water_vale=0;
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x11) 	NVIC_SystemReset();
			if(MainBuf[i]==0x11 && MainBuf[i+1]==0x03)	run_test=1;
		}
	}
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5,MainBuf,MainBuf_SIZE);

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
