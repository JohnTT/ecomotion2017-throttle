/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "throttle.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//AllCell_Bat_RTC displayRTC;
//displayBMSTypeDef displayBMS;

static float DIAMETER = 0.50; //50 cm diameter
static float PI = 3.1415926535; //the number pi
static int CLOCKSPEED = 20000; //timer's clock
static const float ADCMAX = 4095.0;
static const int EXPMINADCIN = 1100;
static const int EXPMAXADCIN = 3000;
static const int CURVEFACTOR = 3;
static const double MAXRPM = 29200.0;
static const double MAXCURRENT = 5.0;
static const double MAXDUTYCYCLE = 60.0;
static const double MAXPOWER = 300.0;
float rpmChan1; //revolutions per minute for one of the wheels of the vehicle
float speedChan1; //holds the car's speed from tim1 channel 1
//float rpmChan2; //revolutions per minute for the other wheel of the vehicle
//float speedChan2; //holds the car's speed from tim1 channel 2
int ADCScalingFactor; //should be constant
double NORMALFACTOR; //should be constant
unsigned int counter; //holds tim1 clock
unsigned int analog; //holds the analog value for hadc1
unsigned int tim1Ch1Capture = 70000; //holds the last value from tim1 channel 1
unsigned int tim1Ch1Compare; //holds the compared value from tim1 channel 1
unsigned int tim1Ch1Overflow; //holds the overflow bit for channel 1 when the tim1 clock resets to 0
unsigned int tim1Ch2Capture = 70000; //holds the last value from tim1 channel 2
unsigned int tim1Ch2Compare; //holds the compared value from tim1 channel 2
unsigned int tim1Ch2Overflow; //holds the overflow bit for channel 2 when the tim1 clock resets to 0
char str[32]; //holds string values
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//extern void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//typedef enum {
//	CAN_PACKET_SET_DUTY = 0,
//	CAN_PACKET_SET_CURRENT,
//	CAN_PACKET_SET_CURRENT_BRAKE,
//	CAN_PACKET_SET_RPM,
//	CAN_PACKET_SET_POS,
//	CAN_PACKET_FILL_RX_BUFFER,
//	CAN_PACKET_FILL_RX_BUFFER_LONG,
//	CAN_PACKET_PROCESS_RX_BUFFER,
//	CAN_PACKET_PROCESS_SHORT_BUFFER,
//	CAN_PACKET_STATUS
//} CAN_PACKET_ID;
/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	double Vedder_ERPM, Vedder_Current, Vedder_DutyCycle;
	int32_t send_index = 0;
	uint8_t buffer[4];
	uint8_t controller_id = 0xFF;

	ADCScalingFactor = EXPMAXADCIN - EXPMINADCIN;
	NORMALFACTOR = exp(CURVEFACTOR);
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
//	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_IT(&hadc1); //commence the ADC for interrupt calculations
//	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); //look into peripheral control functions to find out more about configuration
//	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2); //start timer channel 2 for update interrupts
//	HAL_TIM_Base_Start_IT(&htim1); //start the base for update interrupts
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_Delay(1000);
	printf("we Start");
	printf("\n\r");
	while(1){
		// Timer Counter
		//		printf("Counter value: ");
		//		printf(itoa(counter, c, 10));
		//		printf("\n\r");

		HAL_ADC_Start_IT(&hadc1);
		printf("Analog: %u\n\r", analog);

		// Convert analog (ADC1) to ERPM
		Vedder_ERPM = convertToERPM(analog);
		printf("Vedder ERPM: %f\n\r", Vedder_ERPM);

		//Vedder_DutyCycle = convertToDutyCycle(analog);
		printf("Vedder Duty: %f\n\r", Vedder_DutyCycle);


		Vedder_Current = convertToCurrent(analog);
		printf("Vedder Current: %f\n\r", Vedder_Current);

		send_index = 0;
		//buffer_append_int32(&buffer, (uint32_t)(Vedder_DutyCycle), &send_index);
		//buffer_append_int32(&buffer, (uint32_t)(Vedder_ERPM), &send_index);
		//buffer_append_int32(&buffer, (uint32_t)(Vedder_Current), &send_index);
		buffer_append_int32(&buffer, (uint32_t)(Vedder_Current)*1000.0, &send_index);

		// Send ERPM on CAN Bus
		HAL_StatusTypeDef status;
		hcan1.pTxMsg->IDE = CAN_ID_EXT;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		//hcan1.pTxMsg->StdId = 0xFF; // Vedder ID
		//hcan1.pTxMsg->ExtId = ((uint32_t)CAN_PACKET_SET_DUTY << 8) | controller_id;
		//hcan1.pTxMsg->ExtId = ((uint32_t)CAN_PACKET_SET_RPM << 8) | controller_id;
		const uint8_t CAN_PACKET_SET_CURRENT = 1;
		hcan1.pTxMsg->ExtId = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | controller_id;
		//hcan1.pTxMsg->ExtId = ((uint32_t)0 << 8) | controller_id;
		hcan1.pTxMsg->Data[0] = buffer[0];
		hcan1.pTxMsg->Data[1] = buffer[1];
		hcan1.pTxMsg->Data[2] = buffer[2];
		hcan1.pTxMsg->Data[3] = buffer[3];
		hcan1.pTxMsg->DLC = 4;
		status = HAL_CAN_Transmit_IT(&hcan1);
		if (status != HAL_OK) {
			printf("CAN Transmit Error");
			printf("\n\r");
			Error_Handler();
		}
		HAL_Delay(10);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	//HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1); This is the method to call to stop the Input Capture for channel 1 (while using interrupts)
	//HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2); This is the method to call to stop the Input Capture for channel 2 (while using interrupts)
	//HAL_ADC_Stop_IT($hadc1); This is the method to call to stop the Interrupts for the ADC conversion
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 36;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

}

///* CAN1 init function */
//static void MX_CAN1_Init(void)
//{
//
//  hcan1.Instance = CAN1;
//  hcan1.Init.Prescaler = 12;
//  hcan1.Init.Mode = CAN_MODE_NORMAL;
//  hcan1.Init.SJW = CAN_SJW_1TQ;
//  hcan1.Init.BS1 = CAN_BS1_1TQ;
//  hcan1.Init.BS2 = CAN_BS2_1TQ;
//  hcan1.Init.TTCM = DISABLE;
//  hcan1.Init.ABOM = DISABLE;
//  hcan1.Init.AWUM = DISABLE;
//  hcan1.Init.NART = DISABLE;
//  hcan1.Init.RFLM = DISABLE;
//  hcan1.Init.TXFP = DISABLE;
//  if (HAL_CAN_Init(&hcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 8000;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}


/* USER CODE BEGIN 4 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan) {
	printf("Message Sent Successfully: ID = %lx\n\r", hcan->pTxMsg->IDE == CAN_ID_STD ? hcan->pTxMsg->StdId : hcan->pTxMsg->ExtId);
}
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
	printf("\n\r");
	printf("Message Received by: ");
	printf(itoa(hcan->pRxMsg->StdId, str, 10));
	printf("\n\r");
	if (hcan->pRxMsg->StdId == 0x124) {
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, hcan->pRxMsg->Data[0]);
		//printf("Good stuff");
	}


	if (hcan->pRxMsg->IDE == CAN_ID_STD) {
		//parseCANMessage(hcan->pRxMsg);
	}


	HAL_GPIO_TogglePin(LEDx_GPIO_Port, LED0_Pin);
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
}
//void parseCANMessage(CanRxMsgTypeDef *pRxMsg) {
//	static const float _Current_Factor = 0.05;
//	static const float _Voltage_Factor = 0.05;
//	static const float _Impedance_Factor = 0.01;
//	static const float _CellVolt_Factor = 0.01;
//	static const float _Day_Factor = 0.25;
//	static const float _Second_Factor = 0.25;
//	static const float _SOC_Factor = 0.5;
//	static const float _Capacity_Factor = 0.01;
//
//
//	static const uint16_t _Current_Offset = 1600;
//	static const uint16_t _Temp_Offset = 40;
//	static const uint32_t _PwAvailable_Offset = 2000000000;
//	static const float _Year_Offset = 1985;
//
//	masterCAN1_BMSTypeDef bmsBuf;
//
//	switch (pRxMsg->StdId) {
//	case ecoMotion_MasterBMS:
//		memcpy(&bmsBuf, pRxMsg->Data, sizeof(bmsBuf));
//		displayBMS.current = bmsBuf.current *_Current_Factor - _Current_Offset;
//		displayBMS.voltage = bmsBuf.voltage *_Voltage_Factor;
//		displayBMS.temperature = bmsBuf.temperature - _Temp_Offset;
//		displayBMS.bat_percentage = bmsBuf.bat_percentage * _SOC_Factor;
//		break;
//	case ecoMotion_MasterRTC:
//		memcpy(&displayRTC, pRxMsg->Data, sizeof(displayRTC));
//		displayRTC.Day *= _Day_Factor;
//		displayRTC.Second *= _Second_Factor;
//		break;
//	default:
//		break;
//	}
//}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//printf("we ADC Convert");
	analog = HAL_ADC_GetValue(hadc);
	//HAL_ADC_Start_IT(hadc);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	counter = __HAL_TIM_GetCounter(&htim1); //read TIM1 counter value
	if (htim->Instance == TIM1){
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			if (tim1Ch1Capture != 70000){//initial capture, should be better initialized
				if (tim1Ch1Overflow)
					tim1Ch1Compare = 65535 - tim1Ch1Capture + counter; //flip around
				else
					tim1Ch1Compare = counter - tim1Ch1Capture; //going up
			}
			tim1Ch1Overflow = 0; //reset the overflow bit, since we capture-compared
			tim1Ch1Capture = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM1 channel 1 capture value for the next Compare
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			if (tim1Ch2Capture != 70000){//initial capture, should be better initialized
				if (tim1Ch2Overflow)
					tim1Ch2Compare = 65535 - tim1Ch2Capture + counter; //flip around
				else
					tim1Ch2Compare = counter - tim1Ch2Capture; //going up
			}
			tim1Ch2Overflow = 0; //reset the overflow bit, since we capture-compared
			tim1Ch2Capture = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); //read TIM1 channel 1 capture value for the next Compare
		}
		printf("Captured Tim1 Value:");
		printf(itoa(tim1Ch1Compare, str, 10));
		printf("\n\r");
		speedCalc(CLOCKSPEED, DIAMETER, tim1Ch1Compare, &rpmChan1, &speedChan1);
		//__HAL_TIM_SetCounter(htim, 0); //resets the counter after input capture interrupt occurs
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//for counter update event (wrap back to 0)
	//put overflow bit stuff here.
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	printf("we elapsed");
	if((tim1Ch1Overflow && tim1Ch1Capture != 70000) || (tim1Ch2Overflow && tim1Ch2Capture != 70000)){
		Error_Handler();//error handler stuff, nothing for now
	}
	else {
		tim1Ch1Overflow = 1;
		tim1Ch2Overflow = 1;
	}

}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc){
	printf("we ADC Error");
	//do something, maybe
}
int getTim1Prescaler(){
	return HAL_RCC_GetPCLK2Freq() / 20000; //since it is multiplied by 2
}
static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan) {
	uint8_t prescaleFactor = 0;
	switch (periphClock) {
	case 32:
		theHcan->Init.BS1 = CAN_BS1_13TQ;
		theHcan->Init.BS2 = CAN_BS2_2TQ;
		prescaleFactor = 2;
		break;
	case 36:
		theHcan->Init.BS1 = CAN_BS1_14TQ;
		theHcan->Init.BS2 = CAN_BS2_3TQ;
		prescaleFactor = 2;
		break;
	case 45:
		theHcan->Init.BS1 = CAN_BS1_12TQ;
		theHcan->Init.BS2 = CAN_BS2_2TQ;
		prescaleFactor = 3;
		break;
	case 48:
		theHcan->Init.BS1 = CAN_BS1_12TQ;
		theHcan->Init.BS2 = CAN_BS2_3TQ;
		prescaleFactor = 3;
		break;
	}
	theHcan->Init.SJW = CAN_SJW_1TQ;
	switch (bitRate) {
	case 1000:
		theHcan->Init.Prescaler = prescaleFactor * 1;
		break;
	case 500:
		theHcan->Init.Prescaler = prescaleFactor * 2;
		break;
	case 250:
		theHcan->Init.Prescaler = prescaleFactor * 4;
		break;
	case 125:
		theHcan->Init.Prescaler = prescaleFactor * 8;
		break;
	case 100:
		theHcan->Init.Prescaler = prescaleFactor * 10;
		break;
	case 83:
		theHcan->Init.Prescaler = prescaleFactor * 12;
		break;
	case 50:
		theHcan->Init.Prescaler = prescaleFactor * 20;
		break;
	case 20:
		theHcan->Init.Prescaler = prescaleFactor * 50;
		break;
	case 10:
		theHcan->Init.Prescaler = prescaleFactor * 100;
		break;
	}
}
char *itoa (int value, char *result, int base)
{
	// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}
void speedCalc(int clockSpeed, float wheelDiameter, int compareVal, float* rpmVal, float* speedVal){
	*rpmVal = (clockSpeed*1.0) / (compareVal* 2.0) * 60; //revs per min
	*speedVal = (*rpmVal * 2 * PI * wheelDiameter) / 60; //speedChan1, in m/s
	*speedVal = (*speedVal * 3.6); //speedChan1, in km/h
	printf("Revs per min: ");
	printf(itoa(*rpmVal, str, 10));
	printf("\n\r");
	printf("Real Speed: ");
	printf(itoa(*speedVal, str, 10));
	printf("\n\r");
}
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = (uint8_t)((number >> 24) & 0xFF);
	buffer[(*index)++] = (uint8_t)((number >> 16) & 0xFF);
	buffer[(*index)++] = (uint8_t)((number >> 8) & 0xFF);
	buffer[(*index)++] = (uint8_t)((number) & 0xFF);
}
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}
double convertToERPM(int ADCIn)
{
	int shiftedADCIn = ADCIn - EXPMINADCIN;
	if (ADCIn < EXPMINADCIN)
		return 0.0;
	else if (ADCIn > EXPMAXADCIN)
		return MAXRPM;
	double curveOutput = exp(1.0 * CURVEFACTOR * shiftedADCIn / ADCScalingFactor) - 1.0;
	double normalizedOutput = curveOutput / NORMALFACTOR;

	double ERPM = normalizedOutput * MAXRPM;

	return ERPM;
}
double convertToCurrent(int ADCIn)
{
	int shiftedADCIn = ADCIn - EXPMINADCIN;
	if (ADCIn < EXPMINADCIN)
		return 0.0;
	else if (ADCIn > EXPMAXADCIN)
		return MAXCURRENT;
	double curveOutput = exp(1.0 * CURVEFACTOR * shiftedADCIn / ADCScalingFactor) - 1.0;
	double normalizedOutput = curveOutput / NORMALFACTOR;

	double current = normalizedOutput * MAXCURRENT;

	return current;
}
double convertToDutyCycle(int ADCIn)
{
	int shiftedADCIn = ADCIn - EXPMINADCIN;
	if (ADCIn < EXPMINADCIN)
		return 0.0;
	else if (ADCIn > EXPMAXADCIN)
		return MAXDUTYCYCLE;
	double curveOutput = exp(1.0 * CURVEFACTOR * shiftedADCIn / ADCScalingFactor) - 1.0;
	double normalizedOutput = curveOutput / NORMALFACTOR;

	double duty = normalizedOutput * MAXDUTYCYCLE;

	return duty;
}
double convertToPowerCurrent(int ADCIn)
{
    int ADCScalingFactor = EXPMAXADCIN - EXPMINADCIN;
	double NORMALFACTOR = exp(CURVEFACTOR);
    int shiftedADCIn = ADCIn - EXPMINADCIN;
    if (ADCIn < EXPMINADCIN)
        return 0.0;
    else if (ADCIn > EXPMAXADCIN)
        return MAXCURRENT;
    double curveOutput = exp(1.0 * CURVEFACTOR * shiftedADCIn / ADCScalingFactor) - 1.0;
    double normalizedOutput = curveOutput / NORMALFACTOR;

    double power = normalizedOutput * MAXPOWER;
    double voltage = 48.0; // switch to parse can commit

    double current = power / voltage;
    return current;
}

#ifdef _DEBUG_ON
void __io_putchar(uint8_t ch) { //printf function for USART
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}
#endif
static void MX_CAN1_Init(void)
{
	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan1.Instance = CAN;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	setCANbitRate(500, 36, &hcan1);
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	hcan1.pTxMsg = &TxMessage;
	hcan1.pRxMsg = &RxMessage;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	CAN_FilterConfTypeDef canFilterConfig;
	canFilterConfig.FilterNumber = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = 0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.BankNumber = 14;
	if(HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK) {
		Error_Handler();
	}
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = LED0_Pin | LED1_Pin | LED2_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LEDx_GPIO_Port, &GPIO_InitStruct);

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	printf("PROBLEMS!");
	printf("\n\r");
	HAL_StatusTypeDef status;
	do {
		hcan1.pTxMsg->IDE = CAN_ID_STD;
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
		hcan1.pTxMsg->StdId = ecoMotion_Error_Throttle;
		hcan1.pTxMsg->DLC = 0;
		status = HAL_CAN_Transmit_IT(&hcan1);
		HAL_GPIO_TogglePin(LEDx_GPIO_Port, LED3_Pin);
		HAL_Delay(100);
	} while (status != HAL_OK);
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
