/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
const uint8_t* hello =  "\nHello nemo2.space tracker p gnss relay\n\n\0" ;
const char* full_cold_start = "$PAIR007*3D\r\n\0" ;
const char* save_nvram = "$PAIR513*3D\r\n\0" ;
const char* no_vtg = "PAIR062,5,0\0" ;
const char* nmea_br_9600 = "PAIR864,0,0,9600\0" ;
const char* get_nmea_br = "PAIR865,0,0\0" ;
const char* nmea_end = "\r\n" ;
uint8_t res[250] = { 0 } ;
uint8_t gnss_rx_byte = 0 ;
uint8_t terminal_rx_byte = 0 ;
uint8_t len = 0 ;
//TIM
uint16_t tim_seconds_ths_sys_shutdown = 10 ;
uint16_t tim_seconds = 0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void my_gnss_sw_on ( void ) ;
void my_gnss_sw_off ( void ) ;
unsigned char q1_check_xor ( const uint8_t* , uint8_t ) ;
void send_command ( const char* , bool ) ;
void my_ant_sw_pos ( uint8_t ) ;
void my_tim_init ( void ) ;
void my_tim_start ( void ) ;
void my_tim_stop ( void ) ;
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
  MX_USART5_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay ( 5000 ) ;
  my_tim_init () ;
  HAL_UART_Transmit ( &huart2 , hello , strlen ( hello ) , UART2_TX_TIMEOUT ) ;

  send_command ( get_nmea_br , false ) ;

//  char cs = q1_check_xor ( nmea_br_9600 , strlen ( nmea_br_9600 ) ) ;
//  sprintf ( res , "$%s*%X\r\n\0" , nmea_br_9600 , cs ) ;
//  len = strlen ( (char*) res ) ;
//  HAL_UART_Transmit ( &huart2 , res , len , UART2_TX_TIMEOUT ) ; // muszę dodać 6
//  my_gnss_sw_on() ;
//  my_tim_start () ;
//  HAL_Delay ( 1000 ) ;
//  HAL_UART_Transmit ( &huart5 , res , len , UART2_TX_TIMEOUT ) ;
//  HAL_UART_Transmit ( &huart5 , save_nvram , strlen ( save_nvram ) , UART2_TX_TIMEOUT ) ;
//  HAL_UART_Transmit ( &huart5 , &terminal_rx_byte , 1 , UART2_TX_TIMEOUT ) ;


  char cscs = q1_check_xor ( get_nmea_br , strlen ( get_nmea_br ) ) ;
  sprintf ( res , "$%s*%X\r\n\0" , get_nmea_br , cscs ) ;
  len = strlen ( (char*) res ) ;
  HAL_UART_Transmit ( &huart2 , res , len , UART2_TX_TIMEOUT ) ; // muszę dodać 6
  my_gnss_sw_on() ;
  my_tim_start () ;
  HAL_Delay ( 1000 ) ;
  HAL_UART_Transmit ( &huart5 , res , len , UART2_TX_TIMEOUT ) ;
  //HAL_UART_Transmit ( &huart5 , save_nvram , 13 , UART2_TX_TIMEOUT ) ;
  //HAL_UART_Transmit ( &huart5 , &terminal_rx_byte , 1 , UART2_TX_TIMEOUT ) ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive ( &huart5 , &gnss_rx_byte , 1 , UART5_RX_TIMEOUT ) ;
	  if ( gnss_rx_byte )
	  {
		  HAL_UART_Transmit ( &huart2 , &gnss_rx_byte , 1 , UART5_TX_TIMEOUT ) ;
		  gnss_rx_byte = 0 ;
	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim6.Init.Prescaler = 16000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_SW_CTL1_Pin|RF_SW_CTL2_Pin|GNSS_RST_Pin|GNSS_PWR_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RF_SW_CTL1_Pin RF_SW_CTL2_Pin GNSS_PWR_SW_Pin */
  GPIO_InitStruct.Pin = RF_SW_CTL1_Pin|RF_SW_CTL2_Pin|GNSS_PWR_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GNSS_RST_Pin */
  GPIO_InitStruct.Pin = GNSS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ** ANT SW Operations
void my_ant_sw_pos ( uint8_t pos )
{
	if ( pos == 1 ) // Włączenie GNSS czyli ustawienie RF_SW_CTL1 = LOW i RF_SW_CTL2 = HIGH
	{
		HAL_GPIO_WritePin ( RF_SW_CTL1_GPIO_Port , RF_SW_CTL1_Pin , GPIO_PIN_RESET ) ;
		HAL_GPIO_WritePin ( RF_SW_CTL2_GPIO_Port , RF_SW_CTL2_Pin , GPIO_PIN_SET ) ;
	}
	else if ( pos == 2 )
	{
		HAL_GPIO_WritePin ( RF_SW_CTL1_GPIO_Port , RF_SW_CTL1_Pin , GPIO_PIN_SET ) ;
		HAL_GPIO_WritePin ( RF_SW_CTL2_GPIO_Port , RF_SW_CTL2_Pin , GPIO_PIN_RESET ) ;
	}
}

// ** GNSS Operations
void my_gnss_sw_on ( void )
{
	my_ant_sw_pos ( 1 ) ;
	HAL_GPIO_WritePin ( GNSS_PWR_SW_GPIO_Port , GNSS_PWR_SW_Pin , GPIO_PIN_SET ) ;
	HAL_GPIO_WritePin ( GNSS_PWR_SW_GPIO_Port , GNSS_RST_Pin , GPIO_PIN_SET ) ;
	MX_USART5_UART_Init () ;
}
void my_gnss_sw_off ( void )
{
	my_ant_sw_pos ( 2 ) ;
	HAL_GPIO_WritePin ( GNSS_PWR_SW_GPIO_Port , GNSS_PWR_SW_Pin , GPIO_PIN_RESET ) ;
	HAL_GPIO_WritePin ( GNSS_PWR_SW_GPIO_Port , GNSS_RST_Pin , GPIO_PIN_RESET ) ;
	HAL_UART_DeInit ( &huart5 ) ;

}
unsigned char q1_check_xor ( const uint8_t *m , uint8_t l )
{
	unsigned char result = 0 ;
	unsigned int i = 0 ;

	if ( ( NULL == m ) || ( l < 1 ) )
	{
		return 0;
	}
	for ( i = 0 ; i < l ; i++ )
	{
		result ^= *( m + i ) ;
	}
	return result ;
}

void my_tim_init (void )
{
	__HAL_TIM_CLEAR_IT ( &htim6 , TIM_IT_UPDATE ) ;
}

void my_tim_start (void )
{
	tim_seconds = 0 ;
	HAL_TIM_Base_Start_IT ( &htim6 ) ;
}

void my_tim_stop (void )
{
	HAL_TIM_Base_Stop_IT ( &htim6 ) ;
}

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM6 )
	{
		tim_seconds++ ;
		if ( tim_seconds > tim_seconds_ths_sys_shutdown )
		  {
			  //HAL_NVIC_SystemReset () ;
			  HAL_PWREx_EnterSHUTDOWNMode () ;
		  }
	}
}

void send_command ( const char* c1 , bool save_nram )
{
	size_t len_c1 = strlen ( c1 ) ;
	char cs = q1_check_xor ( c1 , len_c1 ) ;
	char* c2 = (char*) malloc ( ( len_c1 + 7 ) * sizeof ( char ) ) ; // Dodanie 7 wynika z konieczności uwzględnienia znaków: prexi $ i sufix *XX\n\r\0 , gdzie XX to checksum
	size_t len_c2 = strlen ( c2 ) ;
	sprintf ( c2 , "$%s*%X\r\n\0" , c1 , cs ) ;
	size_t len_2 = strlen ( c2 ) ;

	HAL_UART_Transmit ( &huart2 , c2 , len_c2 , UART2_TX_TIMEOUT ) ;
	my_gnss_sw_on() ;
	my_tim_start () ;
	HAL_Delay ( 1000 ) ;
	HAL_UART_Transmit ( &huart5 , c2 , len_c2 , UART2_TX_TIMEOUT ) ;
	if ( save_nram )
		HAL_UART_Transmit ( &huart5 , save_nvram , strlen ( save_nvram ) , UART2_TX_TIMEOUT ) ;
	HAL_UART_Transmit ( &huart5 , &terminal_rx_byte , 1 , UART2_TX_TIMEOUT ) ;
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
