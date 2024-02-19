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
const uint8_t* goodby =  "\nGoodby nemo2.space tracker p gnss relay\n\n\0" ;
const char* save_nvram = "$PAIR513*3D\r\n\0" ;
const char* set_nmea_br_9600 = "PAIR864,0,0,9600\0" ;
const char* get_nmea_br = "PAIR865,0,0\0" ;
const char* get_nav_mode = "PAIR081\0" ;
const char* get_sbas_status = "PAIR411\0" ; // PAIR_SBAS_GET_STATUS

const char* get_fix_outputrate = "PAIR051\0" ; // PAIR_COMMON_GET_FIX_RATE

const char* set_gnss_search_mode = "PAIR066,1,0,1,0,1,0\0" ; // PAIR_COMMON_SET_GNSS_SEARCH_MODE GPS_Enabled , GLONASS_Disabled , Galileo_Enabled , BDS_Disabled , QZSS_Enabled
const char* get_gnss_search_mode = "PAIR067\0" ; // PAIR_COMMON_SET_GNSS_SEARCH_MODE

const char* get_gga_outputrate = "PAIR063,0\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GGA
const char* get_gll_outputrate = "PAIR063,1\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GLL
const char* get_gsa_outputrate = "PAIR063,2\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GSA
const char* get_gsv_outputrate = "PAIR063,3\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GSV
const char* get_rmc_outputrate = "PAIR063,4\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE RMC
const char* get_vtg_outputrate = "PAIR063,5\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE VTG

const char* set_gga_0_outputrate = "PAIR062,0,0\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GGA
const char* set_gll_1_outputrate = "PAIR062,1,1\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GLL
const char* set_gsa_1_outputrate = "PAIR062,2,1\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GSA
const char* set_gsv_1_outputrate = "PAIR062,3,1\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE GSV
const char* set_rmc_1_outputrate = "PAIR062,4,1\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE RMC
const char* set_vtg_0_outputrate = "PAIR062,5,0\0" ; // PAIR_COMMON_GET_NMEA_OUTPUT_RATE VTG

const char* nmea_end = "\r\n" ;
char		my_gnss_command_response[2][250] = { {0} , {0} } ;
uint8_t res[250] = { 0 } ;
uint8_t gnss_rx_byte = 0 ;
uint8_t terminal_rx_byte = 0 ;
uint8_t len = 0 ;
//TIM
uint16_t tim_seconds_ths_sys_shutdown = 10 ;

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
void send_command_save_nram ( void ) ;
void get_command_result ( void ) ;
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

  //send_command ( get_nav_mode , false ) ; // $PAIR081,0*2F
  //HAL_Delay ( 1000 ) ;
  //send_command ( get_sbas_status , false ) ; // $PAIR411,1*23
  //HAL_Delay ( 1000 ) ;
  //send_command ( get_fix_outputrate , false ) ; // $PAIR051,1000*13
  //HAL_Delay ( 1000 ) ;

  //send_command ( set_nmea_br_9600 , false ) ;
  //HAL_Delay ( 1000 ) ;
  //send_command ( get_nmea_br , false ) ; // $PAIR865,9600*12
  //HAL_Delay ( 1000 ) ;
  //send_command ( set_gnss_search_mode , false ) ;
  //HAL_Delay ( 1000 ) ;
  send_command ( get_gnss_search_mode , false ) ; // $PAIR067,1,0,1,0,1,0*3A
  HAL_Delay ( 1000 ) ;
  //send_command ( set_gga_0_outputrate , false ) ;
  //HAL_Delay ( 1000 ) ;
  send_command ( get_gga_outputrate , false ) ;
  HAL_Delay ( 1000 ) ;
  //send_command ( set_vtg_0_outputrate , false ) ;
  //HAL_Delay ( 1000 ) ;
  send_command ( get_vtg_outputrate , false ) ;
  HAL_Delay ( 1000 ) ;
  //send_command_save_nram () ;
  HAL_UART_Transmit ( &huart2 , goodby , strlen ( goodby ) , UART2_TX_TIMEOUT ) ;
  HAL_PWREx_EnterSHUTDOWNMode () ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*HAL_UART_Receive ( &huart5 , &gnss_rx_byte , 1 , UART5_RX_TIMEOUT ) ;
	  if ( gnss_rx_byte )
	  {
		  HAL_UART_Transmit ( &huart2 , &gnss_rx_byte , 1 , UART5_TX_TIMEOUT ) ;
		  gnss_rx_byte = 0 ;
	  }*/

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
	HAL_Delay ( 1000 ) ;
	HAL_UART_Transmit ( &huart5 , c2 , len_c2 , UART2_TX_TIMEOUT ) ;
	free ( c2 ) ;
	get_command_result () ;
	if ( save_nram )
		send_command_save_nram () ;
}

void send_command_save_nram ()
{
	size_t len_c2 = strlen ( save_nvram ) ;

	HAL_UART_Transmit ( &huart2 , save_nvram , len_c2 , UART2_TX_TIMEOUT ) ;
	my_gnss_sw_on() ;
	HAL_Delay ( 1000 ) ;
	HAL_UART_Transmit ( &huart5 , save_nvram , len_c2 , UART2_TX_TIMEOUT ) ;
	HAL_UART_Transmit ( &huart5 , &terminal_rx_byte , 1 , UART2_TX_TIMEOUT ) ;
	get_command_result () ;
}

void get_command_result ()
{
	my_gnss_command_response[0][0] = '\0' ;
	my_gnss_command_response[1][0] = '\0' ;
	if ( my_gnss_get_pair ( my_gnss_command_response ) )
	{
		for (uint8_t i = 0 ; i < 2 ; i++ )
		{
			if ( my_gnss_command_response[i][0] != '\0' ) ;
			{
				HAL_UART_Transmit ( &HUART_DBG , my_gnss_command_response[i] , strlen ( my_gnss_command_response[i] ) , UART2_TX_TIMEOUT ) ;
				HAL_UART_Transmit ( &HUART_DBG , "\r\n" , 2 , UART2_TX_TIMEOUT ) ;
			}
		}
	}
	HAL_UART_Transmit ( &HUART_DBG , "\r\n" , 2 , UART2_TX_TIMEOUT ) ;
}

void my_gnss_receive_byte ( uint8_t* rx_byte , bool verbose )
{
	HAL_UART_Receive ( &HUART_GNSS , rx_byte , 1 , UART_RX_TIMEOUT ) ;
	if ( verbose )
		HAL_UART_Transmit ( &HUART_DBG , rx_byte , 1 , UART_TX_TIMEOUT ) ;
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
