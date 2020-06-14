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
  * the "License"; You may not u	se this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	SENSOR SONAR
	TRIGGER: D9 --- ECHO: B9
	TRIGGER: D10 --- ECHO: B10
	TRIGGER: D11 --- ECHO: B11
		
	-------------------------------
	MOTOR DC SERVO - pwm: PE9,   DIR: PE11
	M1 ->> OUT4 -->> M4 -->> PE11 (PWM)
	M2 ->> OUT3 -->> M3 -->> PE9	(DIR)
	
	-------------------------------
	MOTOR RC SERVO
	PWM --- PA5 (MAU CAM)
	ENCODER:
	PC6,7
	-------------------------------
		UART2(STM) ---- (USB)
		PA2 (TX) ---- RX 
		PA3 (RX) ---- TX
		
*/

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx.h"
//#include "dwt_delay.h" 
#include "math.h"
#include "dwt_delay.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

// vr for uart
float k[2];    // k[0] truoc cham, k[1] sau cham -> k[0],k[1] (float)
uint8_t receivebuffer[5], transmitData[2];
double data[4] = {0,0,0,0};

// _____Vr for Servo_________
float SERVO_MICROS_MAX = 2.7;
float SERVO_MICRO_MIN = 0.55;
int32_t pulse_length;
float x=0.2;
_Bool enableSendSpeed = 0, isStart = 0;
uint16_t pre_enc=0;
float dt=0.1F;

//_____Vr for control________
uint16_t f=0;
float pre_cte, int_cte=0, err=0, steer;
double pi=3.1415927;
float error;
uint8_t flag_stop;
uint8_t ss=0;
float angle =0, pre_angle=0;
uint16_t a1;

//_____Fuzzy_________
uint16_t DC_PWM = 0;
double setpoint;
double pre_Dv =0;
double v = 0;
int16_t DeltaDC_PWM = 0;

//_____Sonarsensor_________
volatile float distance1=0, distance2=0, distance3=0;
signed int upper_limit_sensor=20;
uint16_t local_time=0;
float Ts=0.1;
// _____PID_________
double int_DC_PWM = 0;
double diff_DC_PWM = 0;
uint16_t enc=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void PWM_servo(int16_t rota);
void PWM_DC_Servo(int16_t value);
void delay (uint32_t us);
float twiddle(float *p, float *dp, float err);
float Controller(float errorRcv, uint8_t Sttspeed);

// ------------------------Ham xuat dong co (PA5)----------------------------------
void PWM_servo(int16_t rota)
{
	rota+=90.0F;
	  pulse_length=(((SERVO_MICROS_MAX-SERVO_MICRO_MIN)*(float)rota)/180.0F+  SERVO_MICRO_MIN)*50.0F;
//	  pulse_length= (2.0F*(float)rota/180.0F+0.5F)*50.0F; //25->125
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int16_t)pulse_length);
}

// control dc servo ----- pwm: PE9,   DIR: PE11
void PWM_DC_Servo(int16_t value)
{
	// tien
	if (value>=0)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	}
	// lui
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -value);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		
	}
	a1=value;
}


// ------------------------Read Sonar Sensor----------------------------------
float hcsr04_read (uint16_t GPIO_TRIGGER, uint16_t GPIO_ECHO)
{
	local_time=0;
	HAL_GPIO_WritePin(GPIOD, GPIO_TRIGGER, GPIO_PIN_SET);  // pull the TRIG pin low
	HAL_Delay(1);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	HAL_GPIO_WritePin(GPIOD, GPIO_TRIGGER, GPIO_PIN_RESET);  // pull the TRIG pin HIGH
	
	// read the time for which the pin is high
	
	while (!(HAL_GPIO_ReadPin(GPIOB, GPIO_ECHO)));  // wait for the ECHO pin to go high

	while (HAL_GPIO_ReadPin(GPIOB, GPIO_ECHO))    // while the pin is high
	 {
		 
		local_time++;   // measure time for which the pin is high
		 // DWT_Delay(2);
	 }
	return local_time*0.00876/2;
}

// get velocity
float velocity()
{
	 enc= TIM8->CNT;
	 TIM8->CNT=0;
	return(enc*1.3*pi/2387.0F);
}

// ------------------------Fuzzy Controller----------------------------------

double mu(double x,double L,double C1,double C2,double R){
	double y;
    if (x < L)
        y = 0;
    else if (x<C1)
        y = (x-L)/(C1-L);
    else if (x<C2)
        y = 1;
    else if (x<R)
        y = (R-x)/(R-C2);
    else
        y = 0;
    return y;
}

double max(double num1, double num2)
{
    return (num1 > num2 ) ? num1 : num2;
}
	
// Fuzzification for Servo
double Defuzzification_Servo(double Phi, double DPhi){
    // Fuzzification DPhi
    double DPhi_NE = mu(DPhi,-100, -100, -7, 0);
    double DPhi_ZE = mu(DPhi,-7, -3 , 3, 7);
    double DPhi_PO = mu(DPhi, 3, 7, 100, 100);

    // Fuzzification Phi
    double Phi_NB = mu(Phi, -90, -90, -45, -25);
    double Phi_NS = mu(Phi, -45, -25, -15, -5);
    double Phi_ZE = mu(Phi, -15, -5, 5, 15);
    double Phi_PS = mu(Phi, 5, 15, 25, 45);
    double Phi_PB = mu(Phi, 25, 45, 90, 90);

    // Defuzzication for output
    double Servo_NB = -60;
    double Servo_NM = -40;
    double Servo_NS = -20;
    double Servo_ZE = 	0;
    double Servo_PS =  20;
    double Servo_PM =  40;
    double Servo_PB =  60;

    // Rules
    double Servo1 = Phi_NB * DPhi_NE;
    double Servo2 = Phi_NB * DPhi_ZE;
    double Servo3 = Phi_NB * DPhi_PO;

    double Servo4 = Phi_NS * DPhi_NE;
    double Servo5 = Phi_NS * DPhi_ZE;
    double Servo6 = Phi_NS * DPhi_PO;

    double Servo7 = Phi_ZE * DPhi_NE;
    double Servo8 = Phi_ZE * DPhi_ZE;
    double Servo9 = Phi_ZE * DPhi_PO;

    double Servo10 = Phi_PS * DPhi_NE;
    double Servo11 = Phi_PS * DPhi_ZE;
    double Servo12 = Phi_PS * DPhi_PO;

    double Servo13 = Phi_PB * DPhi_NE;
    double Servo14 = Phi_PB * DPhi_ZE;
    double Servo15 = Phi_PB * DPhi_PO;

    // Max calculate
    double PB = max(Servo13,Servo14);
    double PM = Servo10;
    double PS = max(Servo15, max(Servo11, Servo7));
    double ZE = max(Servo12, max(Servo8, Servo4));
    double NS = max(Servo1, max(Servo5, Servo9));
    double NM = Servo6;
    double NB = max(Servo3, Servo2);

    double Sum_Servo = PB+PM+PS+ZE+NS+NM+NB;
		double DeltaPWM_Servo = 0;
		if (Sum_Servo != 0)
		{
			DeltaPWM_Servo = (Servo_PB*PB+Servo_PM*PM+Servo_PS*PS+Servo_ZE*ZE+Servo_NS*NS+Servo_NM*NM+Servo_NB*NB)/Sum_Servo;
		}
    return DeltaPWM_Servo;
}

// ------------------------Fuzzy Controller DC_Servo----------------------------------
double Defuzzification_DC_Servo(double Delta_v, double a){
    // Fuzzification a
    double DPhi_NE = mu(a,-1000, -1000, -0.1, -0.05);
    double DPhi_ZE = mu(a,-0.1, -0.05 , 0.05, 0.1);
    double DPhi_PO = mu(a, 0.05, 0.1, 1000, 1000);

    // Fuzzification Delta_v
    double Phi_NB = mu(Delta_v, -6, -6, -0.2, -0.1);
    double Phi_NS = mu(Delta_v, -0.2, -0.1, -0.05, 0);
    double Phi_ZE = mu(Delta_v, -0.05, 0, 0, 0.05);
    double Phi_PS = mu(Delta_v, 0, 0.05, 0.1, 0.2);
    double Phi_PB = mu(Delta_v, 0.1, 0.2, 6, 6);
	

    // Defuzzication for output
    double DCServo_NB = -200;
    double DCServo_NM = -120;
    double DCServo_NS = -60;
    double DCServo_ZE = 0;
    double DCServo_PS = 60;
    double DCServo_PM = 120;
		double DCServo_PB = 200;
    // Rules
    double DCServo1 = Phi_NB * DPhi_NE;
    double DCServo2 = Phi_NB * DPhi_ZE;
    double DCServo3 = Phi_NB * DPhi_PO;

    double DCServo4 = Phi_NS * DPhi_NE;
    double DCServo5 = Phi_NS * DPhi_ZE;
    double DCServo6 = Phi_NS * DPhi_PO;

    double DCServo7 = Phi_ZE * DPhi_NE;
    double DCServo8 = Phi_ZE * DPhi_ZE;
    double DCServo9 = Phi_ZE * DPhi_PO;

    double DCServo10 = Phi_PS * DPhi_NE;
    double DCServo11 = Phi_PS * DPhi_ZE;
    double DCServo12 = Phi_PS * DPhi_PO;

    double DCServo13 = Phi_PB * DPhi_NE;
    double DCServo14 = Phi_PB * DPhi_ZE;
    double DCServo15 = Phi_PB * DPhi_PO;

    // Max calculate
    double PB = max(DCServo15, DCServo14);
    double PM = DCServo12;
    double PS = max(DCServo13, max(DCServo11, DCServo9));
    double ZE = max(DCServo10, max(DCServo8, DCServo6));
    double NS = max(DCServo3, max(DCServo5, DCServo7));
    double NM = DCServo4;
    double NB = max(DCServo1,DCServo2);


    double Sum_DCServo = PB+PM+PS+ZE+NS+NM+NB;
		double PWM_DCServo = 0;
		if (Sum_DCServo!= 0)
		{
			PWM_DCServo = (DCServo_PB*PB+DCServo_PM*PM+DCServo_PS*PS+DCServo_ZE*ZE+DCServo_NS*NS+DCServo_NM*NM+DCServo_NB*NB)/Sum_DCServo;
		}
//	  uint16_t speed;
//		if(Sttspeed==1)
//				speed=800;
//		else if(Sttspeed==0)
//				speed=600;
    return PWM_DCServo;	
}
	// ------------------------PID----------------------------------
void PID_Controller(float setpt)
{
	float Kp = 2.3;
	float Ki = 0.1;
	float Kd = 0.02;
	double Dv = 200*(setpt - v);
	int_DC_PWM = int_DC_PWM + Dv;
	diff_DC_PWM = Dv - pre_Dv;
	
	pre_Dv = Dv;
	
	DC_PWM = -(Kp*Dv + Ki*int_DC_PWM + Kd*diff_DC_PWM);
	
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
//	DWT_Init();
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1);
//	HAL_TIM_Base_Start(&htim4);
	HAL_UART_Receive_DMA(&huart2,&receivebuffer[0],5);
	DWT_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(2*90)/90+0.5);
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
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
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1680-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 5000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// time3 cu moi 100ms gui du lieu 1 lan

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==htim3.Instance)
	{
//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//		ss=(ss+1)%3;
//		if(ss==0)
//		distance1 = hcsr04_read(GPIO_PIN_9, GPIO_PIN_9);
//		else if (ss==1)
//		distance2 = hcsr04_read(GPIO_PIN_10, GPIO_PIN_10);
//		else
//		distance3 =  hcsr04_read(GPIO_PIN_11, GPIO_PIN_11);

		distance1 = 100;
		distance2 = 100;
		distance3 = 100;
		// TRIGGER: D9 --- ECHO: B9
		// TRIGGER: D10 --- ECHO: B10
		// TRIGGER: D11 --- ECHO: B11
	 
		// Get data from Raspberrry through UART
		// receivebuffer=[0: goc, 1: vantoc, 4: isStart , 5: SttSpeed]
		int i=0;
		v = velocity();		
		double vantoc = v*50.F;
		transmitData[0]=(int)vantoc;
		transmitData[1]=(int)((vantoc-transmitData[0])*100);
		if (enableSendSpeed==1)
		{
		HAL_UART_Transmit(&huart2, &transmitData[0], 2, 1);
		}

			k[0] =	(int16_t)(((int16_t)receivebuffer[0]<<8)|(int16_t)receivebuffer[1]); 
			k[1] = (int16_t)(((int16_t)receivebuffer[2]<<8)|(int16_t)receivebuffer[3]);
			angle = k[1]/100 -90.0F;
			setpoint =k[0]/100;
		 
			isStart = receivebuffer[4];
						
			if (receivebuffer[4] == 1)
			{
				enableSendSpeed=1;
			}
		// isStart=1;Fuzzy controller for Servo
		if (isStart==1&&distance1>upper_limit_sensor && distance2>upper_limit_sensor+10 && 
				distance3>upper_limit_sensor)
			{
			float DAngle = angle - pre_angle;
			pre_angle =angle;
			steer = (int)Defuzzification_Servo(angle, DAngle);
			if (steer > 60)
				steer = 60;
			else if (steer < -60)
				steer = -60;
			
			double steer_out = data[3];
			
			for (i =3; i>=1; i--)
			{
				data[i] =data[i-1];

			}
			data[0] = steer;

			PWM_servo(steer_out);
			
			// Fuzzy controller for DC_Servo
			float Dv = setpoint - v;
			float a =Dv - pre_Dv;
			pre_Dv = Dv;
			DeltaDC_PWM = (int)Defuzzification_DC_Servo(Dv, a); 
			DC_PWM = DC_PWM + DeltaDC_PWM;
			if (DC_PWM > 1000)
				DC_PWM = 1000;
			else if (DC_PWM <= 0)
				DC_PWM = 0;
			PWM_DC_Servo(DC_PWM);
			/*
			PID_Controller(0.5);
			PWM_DC_Servo(DC_PWM);
			*/
		}
		else
		{
			PWM_DC_Servo(0);
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
