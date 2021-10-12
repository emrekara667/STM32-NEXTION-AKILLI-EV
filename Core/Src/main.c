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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"    // 'sprintf' kullanmak için
#include "string.h"   // 'memset' kullanmak için
#include <stdlib.h>   // 'atoi' kullanmak için
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Dht11
#define DHT11_PORT GPIOC      //DHT11_PIN port bilgisi
#define DHT11_PIN GPIO_PIN_1  //DHT11_PIN pin bilgisi

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[50],tx_buffer[50];

uint8_t cmdEnd[3] = {0xFF, 0xFF, 0xFF};           //Mesajın sonuna eklenir

char str[10];

char component_id[50],slider_deger[50];          //Gelen veriyi component_id ve slider_deger olarak iki parçaya ayırıyoruz.
char *token ;                                    //Uart dan verileri tokenlere ayiriyoruz.

int slider_value_1 = 0, slider_value_2 = 0;                        //Slider degerini metindenden tamsayı değere çeviriyoruz.

uint8_t servo_1 = 0, servo_2,role_1 = 0, role_2 = 0;               //Servo ve röleleri aç-kapa yapabilmek için değişken.

//DHT11 icin degiskenler
uint16_t durum=0,Humidity=0,Temperature=0;
uint16_t tempVal=0,humVal=0;
uint8_t dhtVal[2] ;
uint8_t mData[40];
uint16_t mTime1 = 0, mTime2 = 0;
uint16_t mbit = 0;
uint8_t  parityVal = 0, genParity = 0;
uint16_t if_sarti= 0;
//Dht11 degiskenler sonu
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

//DHT11 icin fonksiyon tanimlari
void delay(uint16_t time);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_Read (uint16_t *sicaklik, uint16_t *nem);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);      //DHT 11 gecikme hesaplama için
  HAL_TIM_Base_Start_IT(&htim2);   //DHT 11 1 saniyede bir deger okumak icin

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);   // slider1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);   // slider2

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // 1. servo
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // 2. servo

  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer, sprintf((char*)tx_buffer,"t0.txt=\"baglanti kuruldu\""), 100);
  HAL_UART_Transmit(&huart2, cmdEnd, 3, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_UART_Receive(&huart2, rx_buffer, 50, 100);
	  memcpy(str, rx_buffer, 50);

	  token = strtok(str, " ");
	  strcpy(component_id, token);
	  token = strtok(NULL, " ");
	  strcpy(slider_deger, token);



	  if(strcmp(component_id, "AB") == 0)
	  {
		  if(servo_1 % 2 == 0)
		  {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2500);
		  }
		  servo_1++;
		  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		  memset(rx_buffer, 0, sizeof(rx_buffer));
		  memset(str, 0, sizeof(str));
		  memset(slider_deger, 0, sizeof(slider_deger));
		  memset(component_id, 0, sizeof(component_id));
	  }
	  else if(strcmp(component_id, "CD") == 0)
	  {
		  if(servo_2 % 2 == 0)
		  {
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
		  }
		  else
		  {
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2500);
		  }
		  servo_2++;
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		  memset(rx_buffer, 0, sizeof(rx_buffer));
		  memset(str, 0, sizeof(str));
		  memset(slider_deger, 0, sizeof(slider_deger));
		  memset(component_id, 0, sizeof(component_id));
	  }
	  else if(strcmp(component_id, "EF") == 0)
	  	  {
	  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	  		  memset(rx_buffer, 0, sizeof(rx_buffer));
	  		  memset(str, 0, sizeof(str));
	  		  memset(slider_deger, 0, sizeof(slider_deger));
	  		  memset(component_id, 0, sizeof(component_id));
	  	  }
	  else if(strcmp(component_id, "GH") == 0)
	  	  {

	  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  		  memset(rx_buffer, 0, sizeof(rx_buffer));
	  		  memset(str, 0, sizeof(str));
	  		  memset(slider_deger, 0, sizeof(slider_deger));
	  		  memset(component_id, 0, sizeof(component_id));
	  	  }
	  else if(strcmp(component_id, "WX") == 0)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		  slider_value_1 = atoi(slider_deger);


		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, slider_value_1);
		 HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer, sprintf((char*)tx_buffer,"t0.txt=\"%%%d\"",slider_value_1), 100);
		 //Metin icerisinde % yazdirmak icin basina % koymalısın örnek: %%
		 HAL_UART_Transmit(&huart2, cmdEnd, 3, 100);
		  memset(rx_buffer, 0, sizeof(rx_buffer));
		  memset(str, 0, sizeof(str));
		  memset(slider_deger, 0, sizeof(slider_deger));
		  memset(component_id, 0, sizeof(component_id));


	  }
	  else if(strcmp(component_id, "YZ") == 0)
	  {

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		  slider_value_2 = atoi(slider_deger);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, slider_value_2);
		  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer, sprintf((char*)tx_buffer,"t1.txt=\"%%%d\"",slider_value_2), 100);
		  HAL_UART_Transmit(&huart2, cmdEnd, 3, 100);
		  memset(rx_buffer, 0, sizeof(rx_buffer));
		  memset(str, 0, sizeof(str));
		  memset(slider_deger, 0, sizeof(slider_deger));
		  memset(component_id, 0, sizeof(component_id));
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65534;
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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//DHT11 fonksiyon tanimlari
void delay(uint16_t time){

	__HAL_TIM_SET_COUNTER(&htim6,0);

	while(__HAL_TIM_GET_COUNTER(&htim6)< time);

}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){


	GPIO_InitTypeDef DHT11_DATA={0};

	DHT11_DATA.Pin=GPIO_Pin;
	DHT11_DATA.Mode=GPIO_MODE_OUTPUT_PP;
	DHT11_DATA.Pull=GPIO_NOPULL;
	DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx,&DHT11_DATA);

}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){


	GPIO_InitTypeDef DHT11_DATA={0};

	DHT11_DATA.Pin=GPIO_Pin;
	DHT11_DATA.Mode=GPIO_MODE_INPUT;
	DHT11_DATA.Pull=GPIO_NOPULL;
	DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOx,&DHT11_DATA);

}

uint8_t DHT11_Read (uint16_t *sicaklik, uint16_t *nem){

  for(int a=0;a<40;a++) mData[a]=0;
   mTime1 = 0, mTime2 = 0, durum=0, tempVal=0, humVal=0, parityVal = 0, genParity = 0,  mbit = 0;

     Set_Pin_Output(DHT11_PORT,DHT11_PIN);
	 HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
    delay(18000);
   	Set_Pin_Input(DHT11_PORT,DHT11_PIN);

	 __HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
    mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);


	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{

		return 0;
	}




	for(int j = 0; j < 40; j++)
	{
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 > 20 && mTime1 < 30)
		{
			mbit = 0;
		}
		else if(mTime1 > 60 && mTime1 < 80)
		{
			 mbit = 1;
		}

		//set i th data in data buffer
		mData[j] = mbit;

	}

	//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++)
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;


	dhtVal[0]= tempVal;
	dhtVal[1] = humVal;

	 *sicaklik=tempVal;
	 *nem=humVal;




	return 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	  durum = DHT11_Read(&Temperature, &Humidity);
		  if(durum == 1)
		  {
			  if_sarti = 1;
		  }
		  else
		  {
			  if_sarti = 0;
		  }
		  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer, sprintf((char*)tx_buffer,"page2.t2.txt=\"%d C\"",Temperature), 100);
		  HAL_UART_Transmit(&huart2, cmdEnd, 3, 100);
		  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buffer, sprintf((char*)tx_buffer,"page2.t3.txt=\"%%%d\"",Humidity), 100);
		  HAL_UART_Transmit(&huart2, cmdEnd, 3, 100);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
