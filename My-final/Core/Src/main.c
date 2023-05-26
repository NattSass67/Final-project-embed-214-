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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_12
#define TRIG_PORT GPIOC
#define ECHO_PIN GPIO_PIN_10
#define ECHO_PORT GPIOC

//#define DT_PIN GPIO_PIN_6
//#define DT_PORT GPIOA
//#define SCK_PIN GPIO_PIN_4
//#define SCK_PORT GPIOA

#define DT_PIN GPIO_PIN_9
#define DT_PORT GPIOB
#define SCK_PIN GPIO_PIN_8
#define SCK_PORT GPIOB
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int readVal;
int32_t hx711_value;
static uint8_t GAIN;	//Gain for clock cycles.


void delay_us (uint16_t us) //delay function
{
__HAL_TIM_SET_COUNTER(&htim1,0);  // setting the delay counter to 0.
while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // while loop till the counter reaches the delay given (us).
}

void hx711_powerUp(void) //Power up function
{
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET); //writing to the pin and setting it to 0.
}
void hx711_setGain(uint8_t gain)  //the values should be 32, 64 or 128
{
	if(gain < 64) GAIN = 2; //32, channel B
	else if(gain < 128) GAIN = 3; //64, channel A
	else GAIN = 1; //128, channel A
}


void hx711_init(void) //initializes the hx711 module by calling 2 functions.
{
  hx711_setGain(128); //setting gain to 128, as this was our best result after trying with other gains.
	hx711_powerUp(); //power up the hx711 module.
}



int32_t hx711_get_value(void) //getting the weight from the module.
{
	uint32_t data = 0; //the data (weight) is firstly set to 0.
	uint8_t dout; // this is to show whether at this bit, if theres a number that should be recorded.
	int32_t filler; //to fill the rest of the 32 bits.
	int32_t ret_value; //final value to return after adding the filling and the data6 together.

	for (uint8_t i = 0; i < 24; i++) //read 24 bit data + set gain and start next conversion
	{

		HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET); //set the clock pin to 1.
		delay_us(1); //delay
			dout = HAL_GPIO_ReadPin(DT_PORT,DT_PIN); //read from the dout pin in variable dout.
			data = data << 1; //shift the data by 1 to make sure we are in correct position depending on the counter.
			if (dout) //if this bit has an output (value of 1) .
			{
				data++; //it sets the data value at this position as 1 as well.
			}
		HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET); //set clock pin to 0.
		delay_us(1); //delay
	}

	for( int i = 0; i < GAIN; i ++ ) //this for loop is for the gain, to add more clock cycles based on the gain.
	{
		HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET); //set clock pin to 1.
		delay_us(1); //delay
		HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET); //set clock pin to 0.
		delay_us(1); //delay, here we are making a clock cycle.
	}

	if( data & 0x800000 ) //here we are checking if theres values in the 24 bits by anding.
		filler = 0xFF000000; //if there are values we add 1's to the last 8 bits which are needed as this is a 32-bit adc.
	else
		filler = 0x00000000; //however, if nothing is in the data we just add 0's.

	ret_value = filler + data; //the return value is the addition of the data with the filler to have the 32-bits.
	return ret_value; //returning the value to be printed.
}

uint8_t hx711_is_ready(void) //making sure that the HX711 module is ready
{
	return HAL_GPIO_ReadPin(DT_PORT,DT_PIN) == GPIO_PIN_RESET; //reading a value from it, the reseting it.
}

uint8_t rxBuffer[1];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		HAL_UART_Transmit(&huart2, rxBuffer, sizeof(rxBuffer),1000);

        //rxDataAvailable = 1;
        //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_UART_Receive_IT(&huart6, rxBuffer, sizeof(rxBuffer));


}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

        //txDataSent = 1;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//HAL_Delay(5000);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
   HAL_UART_Receive_IT(&huart6, rxBuffer, 1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   HAL_Delay(500);
   TIM2->CCR1 =1500;


   uint8_t buffer[8];

    int count=0;

    int check=0;
    hx711_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(check==1){
		 //uint8_t sends[1]="M";
		 uint8_t sends[16]="METAL-detected\r\n";
		 HAL_UART_Transmit_IT(&huart6, sends, sizeof(sends));
		 HAL_Delay(100);
		 check=0;

	  }
	  if(check==2){
		  //uint8_t sends[1]="P";
		  uint8_t sends[18]="PLASTIC-detected\r\n";
		  HAL_UART_Transmit_IT(&huart6, sends, sizeof(sends));
		  HAL_Delay(100);
		  check=0;
	  }
	  if(check==3){

	  }


		  if( hx711_is_ready()) //if the HX711 module is ready.
			  {
			 // HAL_UART_Transmit(&huart2, "ready", 5, HAL_MAX_DELAY);
			  hx711_value = hx711_get_value()/900-267; //we store the value coming from the get value function.
		 //	  printf("Weight: %d g \n\r", (((hx711_value)/1000)+263)); //printing the weight after we calibrated it manually.
			  }

	 	  if (hx711_value >= 11){
	 		 //HAL_UART_Transmit(&huart2, "yes", 3, HAL_MAX_DELAY);
	 		  HAL_ADC_Start(&hadc1);
	 	  	  	if(HAL_ADC_PollForConversion(&hadc1,1000)==HAL_OK){
	 	  	  		readVal=HAL_ADC_GetValue(&hadc1);
	 	  	  		int i=0;
	 	  	  		while(i<5){

	 	  	  			HAL_ADC_Start(&hadc1);
	 	  	  			 if(HAL_ADC_PollForConversion(&hadc1,1000)==HAL_OK){
	 	  	  				 if(HAL_ADC_GetValue(&hadc1)<readVal){
	 	  	  					readVal=HAL_ADC_GetValue(&hadc1);
	 	  	  				 }
//	 	  	  				sprintf(buffer,"%hu\r\n",readVal);
//	 	  	  				HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	 	  	  			  	  i++;
	 	  	  			  	  HAL_Delay(50);
	 	  	  			 }
	 	  	  		}
	 	  	  		if(readVal<2000){
	 	  	  			HAL_Delay(500);
	 	  	  			TIM2->CCR1 = 1000; //500 is 0
	 //	  	  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);
	 	  	  			check=1;


	 	  	  		}
	 	  	  		else{
	 	  	  			HAL_Delay(500);
	 	  	  			TIM2->CCR1 = 2000;
	 	  	  			check=2;
	 	  	  		}
	 	  	  		sprintf(buffer,"%hu\r\n",readVal);
	 	  	  		HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
	 	  	  	}
	 	  	  HAL_Delay(1500);
	 	  	  TIM2->CCR1 = 1500;




	 	  }
	 	// HAL_UART_Transmit(&huart2, "no", 2, HAL_MAX_DELAY);

	 	  //ADC TEST
//	 	 HAL_ADC_Start(&hadc1);
//		if(HAL_ADC_PollForConversion(&hadc1,1000)==HAL_OK){
//			readVal=HAL_ADC_GetValue(&hadc1);
//			sprintf(buffer,"%hu\r\n",readVal);
//			HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
//		}

	 	  //ultrasonic
	 	 HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  __HAL_TIM_SET_COUNTER(&htim1, 0);
		  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  // wait for the echo pin to go high
		  while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
		  Value1 = __HAL_TIM_GET_COUNTER (&htim1);

		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  // wait for the echo pin to go low
		  while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
		  Value2 = __HAL_TIM_GET_COUNTER (&htim1);

		  Distance = (Value2-Value1)* 0.034/2;
		  HAL_Delay(50);
		  if(Distance>20){
			  count=0;
		  }
		  else{
			  count++;
		  }

		  if(count>=5){
			 uint8_t sendt[6]="FULL\r\n";
			  HAL_UART_Transmit_IT(&huart6, sendt, sizeof(sendt));
			  HAL_Delay(100);
			  count=0;
		  }
		  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)

		  HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
