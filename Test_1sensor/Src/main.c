/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile unsigned int time_sensor1=0, time_sensor2=0, time_sensor3=0, time_sensor4=0; //timer for echo pulse from sensors
volatile unsigned int echo_sensor1=0, echo_sensor2=0, echo_sensor3=0, echo_sensor4=0;
volatile unsigned int en_sensor1=0, en_sensor2=0, en_sensor3=0, en_sensor4=0;
volatile float distance1=0, distance2=0, distance3=0, distance4=0;
volatile float alpha=0; 
volatile double current_speed_left=50, current_speed_right=50;
unsigned int TIM_Period=399;
unsigned int upper_limit_sensor=90;
volatile unsigned int count_spin=0,count_lost=0,count_track=0;
volatile int error_Position=0,error_Distance=0;
uint8_t receivebuffer[7],isTracking;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
//Fuzzy--------------------------------------------------------------------------
double mftrap(double x,double L,double C1, double C2, double R){
	double y;
	if (x<L)
		y=0;
	else if (x<C1)
		y=(x-L)/(C1-L);
	else if (x<C2)
		y=1;
	else if (x<R)
		y=(R-x)/(R-C2);
	else
		y=0;
	return y;
}

double max(double num1, double num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

//Motor Value--------------------------------------------------------------------
//Left Motor
double Defuzzification_Obstacle_L(double alpha,double v)
{
	double alpha_NB,alpha_NS,alpha_ZE,alpha_PS,alpha_PB,v_LO,v_ME,v_HI;
	alpha_NB=mftrap(alpha,-60,-60,-45,-30);
	alpha_NS=mftrap(alpha,-45,-30,-30,0);
	alpha_ZE=mftrap(alpha,-30,0,0,30);
	alpha_PS=mftrap(alpha,0,30,30,45);
	alpha_PB=mftrap(alpha,30,45,60,60);

	v_LO=mftrap(v,0,0,60,75);
	v_ME=mftrap(v,60,75,75,90);
	v_HI=mftrap(v,75,90,100,100);

	double dv_NB=-30;
	double dv_NM=-20;
	double dv_NS=-10;
	double dv_ZE=0;
	double dv_PS=10;
	double dv_PM=20;
	double dv_PB=30;

	//RULES
	double beta1=alpha_NB*v_LO; //PB
	double beta2=alpha_NB*v_ME; //PM
	double beta3=alpha_NB*v_HI; //PS
	double beta4=alpha_NS*v_LO; //PM
	double beta5=alpha_NS*v_ME; //PS
	double beta6=alpha_NS*v_HI; //ZE
	double beta7=alpha_ZE*v_LO; //PS
	double beta8=alpha_ZE*v_ME; //ZE
	double beta9=alpha_ZE*v_HI; //NS
	double beta10=alpha_PS*v_LO; //ZE
	double beta11=alpha_PS*v_ME; //NS
	double beta12=alpha_PS*v_HI; //NM
	double beta13=alpha_PB*v_LO; //NS
	double beta14=alpha_PB*v_ME; //NM
	double beta15=alpha_PB*v_HI; //NB

	double beta_PB=beta1;
	double beta_PM=max(beta2,beta4);
	double beta_PS=max(max(beta3,beta5),beta7);
	double beta_ZE=max(max(beta6,beta8),beta10);
	double beta_NS=max(max(beta9,beta11),beta13);
	double beta_NM=max(beta12,beta14);
	double beta_NB=beta15;

	double sumBeta=beta_NB+beta_NM+beta_NS+beta_ZE+beta_PS+beta_PM+beta_PB;
	double dv=(dv_NB*beta_NB+dv_NM*beta_NM+dv_NS*beta_NS+dv_ZE*beta_ZE+dv_PS*beta_PS+dv_PM*beta_PB+dv_PB*beta_PB)/sumBeta;
	return dv;
}

double Defuzzification_Track_L(double ePosition,double eDistance)
{
	double eP_NB,eP_NS,eP_ZE,eP_PS,eP_PB,eD_NE,eD_ZE,eD_PO;
	eP_NB=mftrap(ePosition,-180,-180,-100,-50);
	eP_NS=mftrap(ePosition,-100,-50,-50,0);
	eP_ZE=mftrap(ePosition,-50,0,0,50);
	eP_PS=mftrap(ePosition,0,50,50,100);
	eP_PB=mftrap(ePosition,50,100,180,180);


	eD_NE=mftrap(eDistance,-100,-100,-50,0);
	eD_ZE=mftrap(eDistance,-50,0,0,50);
	eD_PO=mftrap(eDistance,0,50,100,100);

	double dv_NB=-30;
	double dv_NM=-20;
	double dv_NS=-10;
	double dv_ZE=0;
	double dv_PS=10;
	double dv_PM=20;
	double dv_PB=30;

	//RULES
	double beta1=eP_NB*eD_NE; //PB
	double beta2=eP_NB*eD_ZE; //PM
	double beta3=eP_NB*eD_PO; //PS
	double beta4=eP_NS*eD_NE; //PM
	double beta5=eP_NS*eD_ZE; //PS
	double beta6=eP_NS*eD_PO; //ZE
	double beta7=eP_ZE*eD_NE; //PS
	double beta8=eP_ZE*eD_ZE; //ZE
	double beta9=eP_ZE*eD_PO; //NS
	double beta10=eP_PS*eD_NE; //ZE
	double beta11=eP_PS*eD_ZE; //NS
	double beta12=eP_PS*eD_PO; //NM
	double beta13=eP_PB*eD_NE; //NS
	double beta14=eP_PB*eD_ZE; //NM
	double beta15=eP_PB*eD_PO; //NB

	double beta_PB=beta1;
	double beta_PM=max(beta2,beta4);
	double beta_PS=max(max(beta3,beta5),beta7);
	double beta_ZE=max(max(beta6,beta8),beta10);
	double beta_NS=max(max(beta9,beta11),beta13);
	double beta_NM=max(beta12,beta14);
	double beta_NB=beta15;

	double sumBeta=beta_NB+beta_NM+beta_NS+beta_ZE+beta_PS+beta_PM+beta_PB;
	double dv=(dv_NB*beta_NB+dv_NM*beta_NM+dv_NS*beta_NS+dv_ZE*beta_ZE+dv_PS*beta_PS+dv_PM*beta_PB+dv_PB*beta_PB)/sumBeta;
	return dv;
}

//Right Motor
double Defuzzification_Obstacle_R(double alpha,double v)
{
	double alpha_NB,alpha_NS,alpha_ZE,alpha_PS,alpha_PB,v_LO,v_ME,v_HI;
	alpha_NB=mftrap(alpha,-60,-60,-45,-30);
	alpha_NS=mftrap(alpha,-45,-30,-30,0);
	alpha_ZE=mftrap(alpha,-30,0,0,30);
	alpha_PS=mftrap(alpha,0,30,30,45);
	alpha_PB=mftrap(alpha,30,45,60,60);

	v_LO=mftrap(v,0,0,60,75);
	v_ME=mftrap(v,60,75,75,90);
	v_HI=mftrap(v,75,90,100,100);

	double dv_NB=-30;
	double dv_NM=-20;
	double dv_NS=-10;
	double dv_ZE=0;
	double dv_PS=10;
	double dv_PM=20;
	double dv_PB=30;

	//RULES
	double beta1=alpha_NB*v_LO; //NS
	double beta2=alpha_NB*v_ME; //NM
	double beta3=alpha_NB*v_HI; //NB
	double beta4=alpha_NS*v_LO; //ZE
	double beta5=alpha_NS*v_ME; //NS
	double beta6=alpha_NS*v_HI; //NM
	double beta7=alpha_ZE*v_LO; //PS
	double beta8=alpha_ZE*v_ME; //ZE
	double beta9=alpha_ZE*v_HI; //NS
	double beta10=alpha_PS*v_LO; //PM
	double beta11=alpha_PS*v_ME; //PS
	double beta12=alpha_PS*v_HI; //ZE
	double beta13=alpha_PB*v_LO; //PB
	double beta14=alpha_PB*v_ME; //PM
	double beta15=alpha_PB*v_HI; //PS	

	double beta_PB=beta13;
	double beta_PM=max(beta10,beta14);
	double beta_PS=max(max(beta7,beta11),beta15);
	double beta_ZE=max(max(beta4,beta8),beta12);
	double beta_NS=max(max(beta1,beta5),beta9);
	double beta_NM=max(beta2,beta6);
	double beta_NB=beta3;

	double sumBeta=beta_NB+beta_NM+beta_NS+beta_ZE+beta_PS+beta_PM+beta_PB;
	double dv=(dv_NB*beta_NB+dv_NM*beta_NM+dv_NS*beta_NS+dv_ZE*beta_ZE+dv_PS*beta_PS+dv_PM*beta_PB+dv_PB*beta_PB)/sumBeta;
	return dv;
}
double Defuzzification_Track_R(double ePosition,double eDistance)
{
	double eP_NB,eP_NS,eP_ZE,eP_PS,eP_PB,eD_NE,eD_ZE,eD_PO;
eP_NB=mftrap(ePosition,-180,-180,-100,-50);
	eP_NS=mftrap(ePosition,-100,-50,-50,0);
	eP_ZE=mftrap(ePosition,-50,0,0,50);
	eP_PS=mftrap(ePosition,0,50,50,100);
	eP_PB=mftrap(ePosition,50,100,180,180);

	eD_NE=mftrap(eDistance,-100,-100,-50,0);
	eD_ZE=mftrap(eDistance,-50,0,0,50);
	eD_PO=mftrap(eDistance,0,50,100,100);

	double dv_NB=-30;
	double dv_NM=-20;
	double dv_NS=-10;
	double dv_ZE=0;
	double dv_PS=10;
	double dv_PM=20;
	double dv_PB=30;

	//RULES
	double beta1=eP_NB*eD_NE; //NS
	double beta2=eP_NB*eD_ZE; //NM
	double beta3=eP_NB*eD_PO; //NB
	double beta4=eP_NS*eD_NE; //ZE
	double beta5=eP_NS*eD_ZE; //NS
	double beta6=eP_NS*eD_PO; //NM
	double beta7=eP_ZE*eD_NE; //PS
	double beta8=eP_ZE*eD_ZE; //ZE
	double beta9=eP_ZE*eD_PO; //NS
	double beta10=eP_PS*eD_NE; //PM
	double beta11=eP_PS*eD_ZE; //PS
	double beta12=eP_PS*eD_PO; //ZE
	double beta13=eP_PB*eD_NE; //PB
	double beta14=eP_PB*eD_ZE; //PM
	double beta15=eP_PB*eD_PO; //PS	

	double beta_PB=beta13;
	double beta_PM=max(beta10,beta14);
	double beta_PS=max(max(beta7,beta11),beta15);
	double beta_ZE=max(max(beta4,beta8),beta12);
	double beta_NS=max(max(beta1,beta5),beta9);
	double beta_NM=max(beta2,beta6);
	double beta_NB=beta3;

	double sumBeta=beta_NB+beta_NM+beta_NS+beta_ZE+beta_PS+beta_PM+beta_PB;
	double dv=(dv_NB*beta_NB+dv_NM*beta_NM+dv_NS*beta_NS+dv_ZE*beta_ZE+dv_PS*beta_PS+dv_PM*beta_PB+dv_PB*beta_PB)/sumBeta;
	return dv;
}


//Ham xuat % dong co-------------------------------------------------------------
void SetPWM_withDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, int dutyCycle){
	/*This function allow to Write PWM in duty cycle with timer and channel parameters*/
	int32_t pulse_length = TIM_Period*dutyCycle/100;	//range: 250 - 400 
	__HAL_TIM_SET_COMPARE(htim, Channel, pulse_length);
};
void SetPWM_Forward_Backward(int value, uint16_t rightmotor)
{
	if (rightmotor==1)
	{
		if (value>=50)
		{
			SetPWM_withDutyCycle(&htim1,TIM_CHANNEL_1,value);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,0);
		}
		if (value<50)
		{
			SetPWM_withDutyCycle(&htim1,TIM_CHANNEL_1,99-value);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,1);
		}
	}
	else
	{
		if (value>=50)
		{
			SetPWM_withDutyCycle(&htim1,TIM_CHANNEL_3,value);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,0);
		}
		if (value<50)
		{
			SetPWM_withDutyCycle(&htim1,TIM_CHANNEL_3,99-value);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,1);
		}
	}
}
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 41;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Prescaler = 41;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD3 PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==htim3.Instance)
	{	
		//Set trigger signal
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim2);
		
		//Get data from Raspberrry through SPI
		HAL_SPI_Receive_DMA(&hspi1,&receivebuffer[0],7);
		isTracking=receivebuffer[0];
		//if found its owner-----------------------------
		if (isTracking==1)
		{
			count_track++;
			count_lost=0;
			count_spin=0; //reset spin counter after lost
		
			if (count_track>=10)
			{
				error_Position= (int16_t)(((int16_t)receivebuffer[2]<<8)|(int16_t)receivebuffer[1]);
				error_Distance= (int16_t)(((int16_t)receivebuffer[4]<<8)|(int16_t)receivebuffer[3]);
			
				//Calculate distance
				distance1=echo_sensor1*0.0001*340/2/0.5;
				distance2=echo_sensor2*0.0001*340/2/0.5;
				distance3=echo_sensor3*0.0001*340/2/0.5;
				distance4=echo_sensor4*0.0001*340/2/0.5;	
				
				//When there is no obstacle
				if (distance1>upper_limit_sensor && distance2>upper_limit_sensor && distance3>upper_limit_sensor && distance4>upper_limit_sensor)
				{
				current_speed_left=current_speed_left+Defuzzification_Track_L(error_Position,error_Distance);
				current_speed_right=current_speed_right+Defuzzification_Track_R(error_Position,error_Distance);
				}
				//When there is obstacle
				else
				{			
					alpha=(-distance1*60-distance2*30+distance3*30+distance4*60)/(distance1+distance2+distance3+distance4);
					current_speed_left=current_speed_left+Defuzzification_Obstacle_L(alpha,current_speed_left);	
					current_speed_right=current_speed_right+Defuzzification_Obstacle_R(alpha,current_speed_right);		
				}
				

				//Scale to range 0->99
				if (current_speed_left>99) current_speed_left=99;
				if (current_speed_left<0) current_speed_left=0;
				if (current_speed_right>99) current_speed_right=99;
				if (current_speed_right<0) current_speed_right=0;
				
				//Control 2 motors
				SetPWM_Forward_Backward((int)current_speed_left,0);
				SetPWM_Forward_Backward((int)current_speed_right,1);
			}
			
			else
			{
				SetPWM_Forward_Backward((int)50,0);
				SetPWM_Forward_Backward((int)50,1);
			}			
		}
		
		//Cannot find its owner------------------------
		else
		{
			count_lost++;
			if (count_lost>=10)
			{
				count_spin++;
				
				if (count_spin>=100) //after spin 10s ->stop
				{
					SetPWM_Forward_Backward((int)50,0);
					SetPWM_Forward_Backward((int)50,1);
				}
				else
				{			
					count_track=0;
					if (current_speed_left>current_speed_right) //after turn right -> spin left
					{
						SetPWM_Forward_Backward((int)25,0);
						SetPWM_Forward_Backward((int)75,1);
						
					}
					else //after turn left -> spin right
					{
						SetPWM_Forward_Backward((int)75,0);
						SetPWM_Forward_Backward((int)25,1);
						
					}
				}
			}
			else
			{
				SetPWM_Forward_Backward((int)50,0);
				SetPWM_Forward_Backward((int)50,1);
			}
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
