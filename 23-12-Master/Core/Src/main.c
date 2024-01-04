/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @tacgia           : Phan Quoc Viet
  * @gmail         : phanquocviet29@gmail.com
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
#define spi_enable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
#define spi_disable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
#define S2_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
#define S2_H HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#define S3_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
#define S3_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
#define BEGIN (digital[2] == 1 && TONG == 0 && digital[1] == 0 && digital[3] == 0 && -4 < pos && pos < 4 && TONG == 0)
#define NOR1 (digital[2] == 1 && digital[1] == 0 && digital[3] == 0 && TONG == 1) // 00100
#define NOR2 (digital[0] == 0 && digital[2] == 1 && digital[1] == 1 && digital[3] == 0 && digital[4] == 0 && TONG == 1) // 01100
#define NOR3 (digital[0] == 0 && digital[1] == 0 && digital[2] == 1  && digital[3] == 1 && digital[4] == 0 && TONG == 1) // 00110
#define OVERRIGTH (digital[0] == 0 && digital[1] == 0 && digital[2] == 1 && digital[3] == 1 && digital[4] == 1 && TONG == 1) // 00010
#define OVERLEFT (digital[0] == 0 && digital[1] == 1 &&  digital[2] == 0  && digital[3] == 0 && digital[4] == 0 && TONG == 1) //01000
#define THKHUNG1 (digital[0] == 1 && digital[1] == 1 && digital[2] == 0 && TONG == 1) //11000
#define THKHUNG2 (digital[2] == 0 && digital[3] == 1 && digital[4] == 1  && TONG == 1)  //00011
#define THKHUNG3 (digital[0] == 1 && digital[1] == 1 && digital[2] == 0 && TONG == 1) //11100
#define THKHUNG4 (digital[2] == 0 && digital[3] == 1 && digital[4] == 1  && TONG == 1)  //00111
#define FIRSTOP (digital[0] == 0 && digital[1] == 1 && digital[2] == 1 && digital[3] == 1 && digital[4] == 0 && Color == 0 && TONG == 1) // 01110
#define GREEN (  digital[2] == 1 && digital[3] == 1 && digital[4] == 0 && blue == 2 && Color == 1  ) //BLUE
#define GREEN1 ( digital[1] == 1 && digital[2] == 1 && digital[3] == 1 && blue == 2 && Color == 1 ) //BLUE
#define GREEN2 (digital[1] == 1 && digital[2] == 1 && digital[3] == 0  && blue == 2 && Color == 1 ) //BLUE
#define GREEN3 ( digital[1] == 1 && digital[2] == 0 && digital[3] == 1  && blue == 2 && Color == 1 ) //BLUE
#define RED  (digital[1] == 0 && digital[2] == 1 && digital[3] == 1  && blue == 3 && Color == 1  ) //RED
#define RED1  ( digital[1] == 1 && digital[2] == 1 && digital[3] == 1 && blue == 3 && Color == 1 ) //RED
#define RED2  (digital[1] == 1 && digital[2] == 1 && digital[3] == 0 && blue == 3 && Color == 1 ) //RED
#define RED3  ( digital[1] == 1 && digital[2] == 0   && digital[3] == 0 && blue == 3 && Color == 1 ) //RED
#define KOLINE1 (digital[1] == 0 && digital[2] == 0 && digital[3] == 0 && digital[4] == 0 && digital[0] == 0 ) // 00000
#define KOLINE2 ( digital[1] == 1 && digital[2] == 1 && digital[3] == 1 && digital[4] == 1 && digital[0] == 1 ) // 11111
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//tcrt5000 value
uint16_t u16_ADCScanVal[5],ADCValue[5];
float calib[5];
uint8_t digital[5],pre_digital[5]; 
uint16_t TONG = 0; 
uint16_t TH ;


//color value
extern uint16_t tick ;
unsigned short	icValue = 0;
unsigned short	preIcValue = 0;
unsigned short	T = 0;
unsigned short	f[3] ;
uint16_t Color = 0;
uint16_t blue = 0;
uint16_t cnt = 0;
uint16_t time =0;

uint8_t i; //bien dem vi tri sensor
float sum=0; //tong gia tri adc
uint16_t threshold=2900; //nguong doc line den
uint8_t limit_pos=34; //gioi han sai so vi tri
uint8_t limit_left=0,limit_right=0; //gioi han cam bien do line

//input value
float vR= 300; //van toc dai mong muon (mm/s)
float wR; //van toc goc mong muon (rad/s)
float tsamp=0.01; //thoi gian lay mau (s)

//PID
float kp=0.4; 
float ki=0;
float kd=0.024; 
float b=186.5; //khoang cach tam 2 banh (mm)
float D=67; //duong kinh banh xe (mm)
float L=48.65; //khoang cach tu truc dong co den mat cam bien
uint16_t ymax=4095;//gia tri lon nhat cam bien tren line den
uint16_t ymin=160; //gia tri nho nhat cam bien tren nen trang
uint16_t high[5] = {4095,4095,4095,4095,4095};
uint16_t low[5] = {160,150,140,150,130};
	
//calculate value
float pre_e2=0,e2=0; //sai so e2
float inte2=0; //khau tich phan
float vleft=0,vright=0,wleft=0,wright=0; //van toc banh trai va banh phai
float v=0,w=0; //van toc dai va van toc goc

//position value
float weight; //trong so
float slope=0.3959; //do doc gia tri
float intercept=-0.7642; //giao diem truc hoanh
float pos=0; //gia tri vi tri tinh toan

//transmit value
uint8_t txBuffer[4] ; //mang truyen du lieu
//uint8_t dem=0;
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */


// color

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void read_ADC(void)
{
	//trai qua phai: 0->4
	sum=0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)u16_ADCScanVal,5);	
	for (i=0;i<5;i++)
	{
		pre_digital[i]=digital[i];
		ADCValue[i]=u16_ADCScanVal[i];
		if (ADCValue[i] > threshold) digital[i]=1;
		else digital[i]=0;
	}	
	calib[0] = ymin + 1*(ADCValue[0]-low[0]) ;
	calib[1] = ymin + 0.997*(ADCValue[1]-low[1]) ;
	calib[2] = ymin + 0.995*(ADCValue[2]-low[2]) ;
	calib[3] = ymin + 0.997*(ADCValue[3]-low[3]) ;
	calib[4] = ymin + 0.992*(ADCValue[4]-low[4]) ;
	sum=calib[0]+calib[1]+calib[2]+calib[3]+calib[4];
	pos = ((-34*calib[0])+(-17*calib[1])+(0*calib[2])+(17*calib[3])+(34*calib[4]))/sum;
}

void calculate_error(void)
{
	pre_e2=e2;
	e2=pos;
	w = kp*e2 + kd*(e2-pre_e2)/tsamp;
  vleft = 0.5*(2*vR - w*b);
  vright = 0.5*(2*vR + w*b);
	wleft=(vleft*60*2/D)/(2*3.14);
	wright=(vright*60*2/D)/(2*3.14);
}

void transmit_value(void)
{
	if (wleft >255) wleft=255;
	else if (wleft<0) wleft=0;
	if (wright > 255) wright=255;
	else if (wright<0) wright=0;
	txBuffer[0] = wleft ;
	txBuffer[1] = wright;
	if (pos<0)	txBuffer[2] = -pos;
	else	txBuffer[2]=pos+100;
	txBuffer[3] = TH;
	spi_enable;			
	HAL_SPI_Transmit_IT(&hspi1,txBuffer,4);	
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == hspi1.Instance)
	{
		spi_disable;
	}
}

void filter(int s2, int s3) // func turn on of color sensor
	{
	if(s2 == 0 && s3 == 0){
		S2_L; S3_L;
	}
	if(s2 == 0 && s3 == 1){
		S2_L; S3_H;
	}
	if(s2 == 1 && s3 == 0){
		S2_H; S3_L;
	}
	if(s2 == 1 && s3 == 1){
		S2_H; S3_H;
	}
}


void full(void)
{
	read_ADC();
	calculate_error();
}

void run(void)
{
		HAL_Delay(12);
		full();
		//Bat dau chay
		if(BEGIN)
		{
			HAL_Delay(1500);
			full();
			if(BEGIN) //start
			{
			TH = 1;
			full();
			transmit_value();
			TONG = 1;
			}
		}
		else if( NOR1 || NOR2 || NOR3 || THKHUNG1 || THKHUNG2 || THKHUNG3 || THKHUNG4 ) //Chay binh thuong
		{
			full();
			TH = 1;
			transmit_value();
		}
		else if( OVERLEFT || OVERRIGTH) 
		{
			full();
			TH = 1;
			transmit_value();
		}
		else if(FIRSTOP) // stop and read line
			{
			TH = 2;
			transmit_value();	
			Color = 1;
  		HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
			}
		else if(GREEN || GREEN1 || GREEN2 || GREEN3) // turn right
			{

			TH = 3  ;
			transmit_value();
				
		}
		else if(RED || RED1 || RED2 || RED3) // turn left
			{
			TH = 4;
			transmit_value();
			
			}
		else if (KOLINE1 || KOLINE2 )
		{
			wleft = 0;
			wright = 0;
			TH = 1;
			transmit_value();
		}
}
/* USER CODE END PFP */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // detect color
	{
   if (htim->Instance == TIM2) 
		{
   if(tick == 100)
			{
			filter(0,0); //red
			icValue = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);
			T = icValue - preIcValue;
			preIcValue = icValue;
			f[0] = 1000000 / T;
			}

			if(tick == 200)
			{
			filter(0,1); //blue
			icValue = __HAL_TIM_GetCompare(&htim2, TIM_CHANNEL_1);
			T = icValue - preIcValue;
			preIcValue = icValue;
			f[2] = 1000000 / T;
			}
		 if(f[0] < 30000   )
			{
				blue = 2; // blue		
				time++;
			}
			else if (f[0] > 40000 )
			{
				blue = 3; // red
				time++;
			}

			if(time == 60000) 
			{
				cnt++;
				if(cnt == 10) 
				HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_1);
			}
			
			}
    
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		run();
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
