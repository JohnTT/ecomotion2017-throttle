/*
 * throttle.h
 *
 *  Created on: Mar 30, 2017
 *      Author: Constellations
 */

#ifndef THROTTLE_H_
#define THROTTLE_H_

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
	ecoMotion_Throttle = 0x00,
	ecoMotion_Master = 0x10,
	ecoMotion_Display = 0x20,
	ecoMotion_Error = 0xFF
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
#endif /* THROTTLE_H_ */
