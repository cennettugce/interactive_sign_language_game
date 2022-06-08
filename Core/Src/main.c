/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "fsmc.h"
#include "openx07v_c_lcd.h"
#include "stdio.h"
#include "TJ_MPU6050.h"

#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "tf_model.h"
#include "tf_model_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t adcData[5]={0};
uint8_t counter = 0;
uint8_t gameCounter = 0;
char currentLetter;
int score;
uint8_t lives;
uint8_t state=0;
GPIO_PinState thumb;
GPIO_PinState middle;
uint8_t scoreTable[6]={0};
uint8_t tableCounter;
float y_val[26];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim10;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_DMA_Init(void);
static void MX_RNG_Init(void);
static void MX_ADC1_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
char generateLetter();
void gameOver();
void display(char , uint8_t , int , uint8_t );
uint8_t correctLetter(char, float y_val[]);
int largest(float arr[], int);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim10 )
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    counter++;
    gameCounter++;

  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ScaledData_Def myAccelScaled, myGyroScaled;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;

	ai_float deneme[11];
	char buf[50];

	int buf_len = 0;
	ai_error ai_err;
	ai_i32 nbatch;
	//uint32_t timestamp;

	//ai_float data_buf[11] = {1142,	787	,807,	2213,	1942,	1062,	-7,	2,	-195,	-833,	541};
	//ai_float data_buf[11] = {1123,1496,644,2239,1956,918,306,312,93,207,259};
	ai_float data_buf[11] = {0};
	//Chunk of memory used to hold intermediate values for the (model)
	AI_ALIGNED(4) ai_u8 activations[AI_TF_MODEL_DATA_ACTIVATIONS_SIZE];

	//Buffers used to store input and output tensors
	AI_ALIGNED(4) ai_i8 in_data[AI_TF_MODEL_IN_1_SIZE_BYTES];
	AI_ALIGNED(4) ai_i8 out_data[AI_TF_MODEL_OUT_1_SIZE_BYTES];

	//Pointer to our model
	ai_handle tf_model = AI_HANDLE_NULL;

	//Initialize wrapper structs that hold pointers to data and info about the data (tensor height, width, channels)
	ai_buffer ai_input[AI_TF_MODEL_IN_NUM] = AI_TF_MODEL_IN;
	ai_buffer ai_output[AI_TF_MODEL_OUT_NUM] = AI_TF_MODEL_OUT;

	//Set working memory and get weights/biases from model
	ai_network_params ai_params = {
			AI_TF_MODEL_DATA_WEIGHTS(ai_tf_model_data_weights_get()),
			AI_TF_MODEL_DATA_ACTIVATIONS(activations)
	};

	//Set pointers wrapper structs to our data buffers
	//ai_input[0].n_batches = 1;
	ai_input[0].data = AI_HANDLE_PTR(in_data);
	//ai_output[0].n_batches = 1;
	ai_output[0].data = AI_HANDLE_PTR(out_data);

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
  MX_TIM10_Init();
  MX_DMA_Init();
  MX_RNG_Init();
  MX_ADC1_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  BSP_LCD_Init();
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

  MPU6050_Init(&hi2c1);
  	//2. Configure Accel and Gyro parameters
  	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
  	myMpuConfig.ClockSource = Internal_8MHz;
  	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
  	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
  	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
  	MPU6050_Config(&myMpuConfig);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcData, 5);


    ai_err = ai_tf_model_create(&tf_model, AI_TF_MODEL_DATA_CONFIG);
    if (ai_err.type != AI_ERROR_NONE)
    {
  	  buf_len = sprintf(buf, "Error: Could not create LR instance\r\n");
  	 // HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	  while(1);
    }

    //Initialize the model
    if (!ai_tf_model_init(tf_model, &ai_params)){
  	  buf_len = sprintf(buf, "Error: Could not initialize LR \r\n");
  	 // HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	  while(1);
    }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  MPU6050_Get_Accel_Scale(&myAccelScaled);
	  MPU6050_Get_Gyro_Scale(&myGyroScaled);

	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET){
		  lives = 3;
		  counter = 0;
		  gameCounter = 0;
		  score = 0;
		  HAL_Delay(10);
		  HAL_TIM_Base_Start_IT(&htim10);
		  HAL_RNG_GenerateRandomNumber_IT(&hrng);
		  currentLetter = generateLetter();
		  state=1;
		  BSP_LCD_Clear(LCD_COLOR_MAGENTA);
	  }

	  if(lives==0||gameCounter==60){
		  gameOver();
	  }

	  if(counter>9){
		  counter=0;
		  currentLetter = generateLetter();
		  lives--;
		  BSP_LCD_DisplayStringAt(40, 20, (uint8_t *)"TIMEOUT!", CENTER_MODE);
		  HAL_Delay(1000);
		  BSP_LCD_Clear(LCD_COLOR_MAGENTA);
	  }

	  if(state==1&&correctLetter(currentLetter, y_val)==1){
		  score++;
		  currentLetter = generateLetter();
		  counter=0;
		  BSP_LCD_DisplayStringAt(40, 20, (uint8_t *)"CORRECT!", CENTER_MODE);
		  HAL_Delay(1000);
		  BSP_LCD_Clear(LCD_COLOR_MAGENTA);
	  }
	  if(state==1){
		  display(currentLetter, counter, score, lives);
	  }else{
		  char scoreS[15];
		  	sprintf(scoreS, "player1: %d ", scoreTable[0]);
		    BSP_LCD_DisplayStringAt(40, 52, (uint8_t *)scoreS, LEFT_MODE);
		  	sprintf(scoreS, "player2: %d ", scoreTable[1]);
		    BSP_LCD_DisplayStringAt(40, 82, (uint8_t *)scoreS, LEFT_MODE);
		  	sprintf(scoreS, "player3: %d ", scoreTable[2]);
		    BSP_LCD_DisplayStringAt(40, 112, (uint8_t *)scoreS, LEFT_MODE);
		  	sprintf(scoreS, "player4: %d ", scoreTable[3]);
		    BSP_LCD_DisplayStringAt(40, 142, (uint8_t *)scoreS, LEFT_MODE);
		  	sprintf(scoreS, "player5: %d ", scoreTable[4]);
		    BSP_LCD_DisplayStringAt(40, 172, (uint8_t *)scoreS, LEFT_MODE);
		  	sprintf(scoreS, "player6: %d ", scoreTable[5]);
		    BSP_LCD_DisplayStringAt(40, 202, (uint8_t *)scoreS, LEFT_MODE);
		      BSP_LCD_SetFont(&Font24);
		      BSP_LCD_DisplayStringAt(40, 20, (uint8_t *)"Press Button",LEFT_MODE);
		      BSP_LCD_DrawRect(30, 50, 230, 180);
		      BSP_LCD_DrawHLine(30, 80, 230);
		      BSP_LCD_DrawHLine(30, 110, 230);
		      BSP_LCD_DrawHLine(30, 140, 230);
		      BSP_LCD_DrawHLine(30, 170, 230);
		      BSP_LCD_DrawHLine(30, 200, 230);

	  }

	  data_buf[0] = (ai_float)adcData[0];
	  data_buf[1] = (ai_float)adcData[1];
	  data_buf[2] = (ai_float)adcData[2];
	  data_buf[3] = (ai_float)adcData[3];
	  data_buf[4] = (ai_float)adcData[4];
	  data_buf[5] = (ai_float)myAccelScaled.x;
	  data_buf[6] = (ai_float)myAccelScaled.y;
	  data_buf[7] = (ai_float)myAccelScaled.z;
	  data_buf[8] = (ai_float)myGyroScaled.x;
	  data_buf[9] = (ai_float)myGyroScaled.y;
	  data_buf[10] = (ai_float)myGyroScaled.z;



	  //Fill input buffer
	  for (uint32_t i=0; i < AI_TF_MODEL_IN_1_SIZE; i++)
	  {
		  ((ai_float*)in_data)[i] = data_buf[i]; 	//2.0f will change to sensor data
		  deneme[i]=((ai_float*)in_data)[i];
	  }

	  //Timestamp from video

	  //Perform inference
	  nbatch = ai_tf_model_run(tf_model, &ai_input[0], &ai_output[0]);
	  if (nbatch != 1)
	  {
		  buf_len = sprintf(buf, "Error: Could not initialize LR \r\n");
		//  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
	  }

	  //Read output (predicted y) of the model
	  for(int i=0; i<26; i++){
		  y_val[i] = ((float*)out_data)[i];
	  }



	  //Print output
	 // buf_len = sprintf(buf, "Output: %f\r\n", y_val);
	 // HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 42000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC5 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 2;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 5;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
char generateLetter(){
	char letter;
	uint32_t randNum=0;
	randNum = HAL_RNG_GetRandomNumber(&hrng)%26;

			     if(randNum == 0) (letter = 'A');
			     else if (randNum == 1) (letter = 'B');
			     else if (randNum == 2) (letter = 'C');
			     else if (randNum == 3) (letter = 'D');
			     else if (randNum == 4) (letter = 'E');
			     else if (randNum == 5) (letter = 'F');
			     else if (randNum == 6) (letter = 'G');
			     else if (randNum == 7) (letter = 'H');
			     else if (randNum == 8) (letter = 'I');
			     else if (randNum == 9) (letter = 'J');
			     else if (randNum == 10) (letter = 'K');
			     else if (randNum == 11) (letter = 'L');
			     else if (randNum == 12) (letter = 'M');
			     else if (randNum == 13) (letter = 'N');
			     else if (randNum == 14) (letter = 'O');
			     else if (randNum == 15) (letter = 'P');
			     else if (randNum == 16) (letter = 'Q');
			     else if (randNum == 17) (letter = 'R');
			     else if (randNum == 18) (letter = 'S');
			     else if (randNum == 19) (letter = 'T');
			     else if (randNum == 20) (letter = 'U');
			     else if (randNum == 21) (letter = 'V');
			     else if (randNum == 22) (letter = 'W');
			     else if (randNum == 23) (letter = 'Y');
			     else if (randNum == 24) (letter = 'X');
			     else if (randNum == 25) (letter = 'Z');
		return letter;
}

void gameOver(){
	HAL_TIM_Base_Stop_IT(&htim10);
	lives=3;
	state=0;
	gameCounter=0;
	scoreTable[tableCounter]=score;
	if(tableCounter<6){
		tableCounter++;
	}else{
		tableCounter=0;
	}
  	BSP_LCD_Clear(LCD_COLOR_MAGENTA);


}

void display(char letter, uint8_t counter, int score, uint8_t lives){
			char counterS[15];
			char scoreS[15];
			char livesS[15];
			sprintf(counterS, "counter: %d ", counter);
			sprintf(scoreS, "score: %d ", score);
			sprintf(livesS, "lives: %d ", lives);


		    BSP_LCD_SetFont(&Font24);
		    BSP_LCD_DisplayChar(150, 20, letter);
		    BSP_LCD_DisplayStringAt(40, 80, (uint8_t *)counterS, CENTER_MODE);
		    BSP_LCD_DisplayStringAt(40, 140, (uint8_t *)scoreS, CENTER_MODE);
		    BSP_LCD_DisplayStringAt(40, 200, (uint8_t *)livesS, CENTER_MODE);

}

uint8_t correctLetter(char letter, float y_val[]){
	uint8_t result=0;

	thumb=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
	middle=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

	 	 	 	 	 if(letter=='A') {
	 	 	 	 		 if(largest(y_val, 26)==0){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if  (letter == 'B') {
	 	 	 	 		 if(largest(y_val, 26)==1){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				    else if (letter == 'C'){
	 	 	 	 		 if(largest(y_val, 26)==2){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'D'){
	 	 	 	 		 if(largest(y_val, 26)==3){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'E'){
	 	 	 	 		 if(largest(y_val, 26)==4){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter =='F'){
	 	 	 	 		 if(largest(y_val, 26)==5){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'G'){
	 	 	 	 		 if(largest(y_val, 26)==6){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter =='H'){
	 	 	 	 		 if(largest(y_val, 26)==7){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if  (letter == 'I'){
	 	 	 	 		 if(largest(y_val, 26)==8){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'J'){
	 	 	 	 		 if(largest(y_val, 26)==9){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'K'){
	 	 	 	 		 if(largest(y_val, 26)==10){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'L'){
	 	 	 	 		 if(largest(y_val, 26)==11){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'M'){
	 	 	 	 		 if(largest(y_val, 26)==12){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'N'){
	 	 	 	 		 if(largest(y_val, 26)==13){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'O'){
	 	 	 	 		 if(largest(y_val, 26)==14){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'P'){
	 	 	 	 		 if(largest(y_val, 26)==15){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'Q'){
	 	 	 	 		 if(largest(y_val, 26)==16){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'R'){
	 	 	 	 		 if(largest(y_val, 26)==17){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'S'){
	 	 	 	 		 if(largest(y_val, 26)==18){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'T'){
	 	 	 	 		 if(largest(y_val, 26)==19){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'U'){
	 	 	 	 		 if(largest(y_val, 26)==20){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'V'){
	 	 	 	 		 if(largest(y_val, 26)==21){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'W'){
	 	 	 	 		 if(largest(y_val, 26)==22){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'X'){
	 	 	 	 		 if(largest(y_val, 26)==23){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'Y'){
	 	 	 	 		 if(largest(y_val, 26)==24){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
				     else if (letter == 'Z'){
	 	 	 	 		 if(largest(y_val, 26)==25){
	 	 	 	 			 result=1;
	 	 	 	 		 }
	 	 	 	 	 }
	 	 	 return result;
}

int largest(float arr[], int n)
{
    int i;
    int max_i;
    // Initialize maximum element
    float max = arr[0];

    // Traverse array elements
    // from second and compare
    // every element with current max
    for (i = 1; i < n; i++)
        if (arr[i] > max){
            max = arr[i];
            max_i=i;
        }

    return max_i;
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
