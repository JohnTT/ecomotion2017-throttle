/*
 * throttle.h
 *
 *  Created on: Mar 30, 2017
 *      Author: Constellations
 */


#define _CAR 1

#ifdef _CAR

#define _ERRORHANDLER_CAN1TRANSMIT 1
#define _REBROADCAST_ALLCELL 1

#endif

#ifndef _CAR

#define _DEBUG_ON 1

#ifdef _DEBUG_ON
#define _REBROADCAST_ALLCELL 1
#define _CAN_PRINTF 1
#endif

#endif


typedef struct {
	uint16_t current;
	uint16_t voltage;
	uint8_t temperature;
	uint8_t bat_percentage;
} masterCAN1_BMSTypeDef;

typedef struct {
	double current;
	double voltage;
	uint8_t temperature;
	double bat_percentage;
} displayBMSTypeDef;

typedef struct {
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
} AllCell_Bat_RTC;


typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS
} CAN_PACKET_ID;

typedef enum {
	ecoMotion_MotorControl = 0x01,
	ecoMotion_Speed = 0x02,
	ecoMotion_FrontWheels = 0x03,
	ecoMotion_Master_BMS = 0x04,
	ecoMotion_Humidity = 0x05,
	ecoMotion_Temperature = 0x06,
	ecoMotion_Throttle = 0x20,
	ecoMotion_Master = 0x30,
	ecoMotion_MasterBMS = 0x31,
	ecoMotion_MasterRTC = 0x3A,
	ecoMotion_Display = 0x40,
	ecoMotion_Error_Throttle = 0xFFF,
	ecoMotion_Error_Master = 0x0FEF,
	ecoMotion_Error_Display = 0xFDF,
} CAN_DEVICE_ID;

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc);
int getTim1Prescaler();
static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan);
char *itoa (int value, char *result, int base);
void speedCalc(int clockSpeed, float wheelDiameter, int compareVal, float* rpmVal, float* speedVal);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
double convertToERPM(int ADCIn);
double convertToCurrent(int ADCIn);
double convertToDutyCycle(int ADCIn);
void __io_putchar(uint8_t ch);


static const int CAN_ThrottleID;
