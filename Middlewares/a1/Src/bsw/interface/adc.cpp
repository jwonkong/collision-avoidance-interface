/*
 * adc.hpp
 *
 *  Created on: April 5, 2022
 *      Author: Zafra Juanen
 */
#include "bsw/interface/adc.hpp"
#include "bsw/interface/io_function.hpp"

#include "main.h"

static const float ADC_max = 16384.0F; //  ADC max value because of 12bit + Oversampling
static const float ADC_ref = 3.3F;		// ADC Reference voltage

static const uint16_t ADC1_BufferSize = N_SAMPLE_PER_CH * (bsw::interface::TOTAL_N_CH - 1); // ADC1 Half buffer size
static uint32_t ADC1_Buffer[ADC1_BufferSize * 2]; // * 2 Since there is a double buffer for DMA

static volatile uint16_t adc[bsw::interface::TOTAL_N_CH-1]; // ADC1 value accumulator
static float voltage[bsw::interface::TOTAL_N_CH-1]; // ADC1 Voltage

static bool ADC_pending = false;

/*
 * @brief ADC Conversion function
 * @param pos 0, pos 1 means first and second half of the buffer
 * @description / usage
 * ADC buffer size is Channels * Buffer * 2 and it is divided in
 * low and high half.
 * HAL_ADC_ConvCpltCallback will trigger when the end of the high buffer is
 * reached.
 * HAL_ADC_ConvHalfCpltCallback will trigger when the first half of the buffer
 * is reached.
 */
void readValue(uint8_t pos) {
	uint16_t buffer_i = 0; // Buffer Index
	if(pos) buffer_i = ADC1_BufferSize;

	/*
	 * ADC Channels work as follows:
	 * As we configured Channel IN6 and IN8 from ADC1
	 * The resulting buffer will look like:
	 * [IN6_0, IN8_0, IN6_1, IN8_1, IN6_2, IN8_2 ... IN6_N, IN8_N]
	 * Where N is the n-sim sample
	 *
	 * In Dual Simultaneous mode as configured:
	 * ADC1 uses the buffer lower 2 bytes and ADC2 uses the upper 2 bytes as:
	 *
	 * [ADC2, ADC1]
	 */

	uint16_t i, j;
	uint32_t val;

	for(i = 0; i<N_SAMPLE_PER_CH; i++) {
		for(j = 0; j<(bsw::interface::TOTAL_N_CH - 1); j++) {
			val = *(ADC1_Buffer + buffer_i);
			if(i == 0) {
				*(adc + (2*j)) = (val & 0xffff);		// Initialize ADC1
				if(j==1) *(adc + (j)) = (val & 0xffff);
//				if(j == 0) *(adc + (2*j) + 1) = ((val >> 16) & 0xffff); // Initialize ADC2
			} else {
				*(adc + (2*j)) += (val & 0xffff);		// Elements from ADC1
				if(j==1) *(adc + (j)) += (val & 0xffff);
//				if(j == 0) *(adc + (2*j) + 1) += ((val >> 16) & 0xffff); // Elements from ADC2
			}
			buffer_i++;
		}
	}

	for(i = 0; i <bsw::interface::TOTAL_N_CH-1; i++){
		*(voltage+i) = (((float)((*(adc+i)) >> (ADC1_OVERSAMPLING+1)))/ADC_max * ADC_ref)*10;
	}

	ADC_pending = true;
}

void bsw::interface::AdcReader::setPending(bool state){
	ADC_pending = state;
}

bool bsw::interface::AdcReader::getPending(){
	return ADC_pending;
}

float* bsw::interface::AdcReader::getVoltage() {
	return voltage;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc) readValue(1);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc) readValue(0);
}

void bsw::interface::AdcReader::initAdc(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC1) {
		hadc1_ = hadc;
	} else if (hadc->Instance == ADC2) {
		hadc2_ = hadc;
	} else if (hadc->Instance == ADC3) {
		hadc3_ = hadc;
	}

	ADC_pending = false;
}

uint8_t bsw::interface::AdcReader::adcStart(void){
	uint8_t status = HAL_OK;
	uint8_t retval = 0;
	uint8_t s = 0;

	status = HAL_ADCEx_Calibration_Start(hadc1_, ADC_SINGLE_ENDED);		// calibrate ADC1
	if(status) retval |= (1 << s);
	s++;
//	status = HAL_ADCEx_Calibration_Start(hadc2_, ADC_SINGLE_ENDED);		// calibrate ADC2
//	if(status) retval |= (1 << s);
//	s++;

	status = HAL_ADC_Start_DMA(hadc1_, ADC1_Buffer, (uint32_t)(ADC1_BufferSize * 2));
//	status = HAL_ADC_Start(hadc2_);
//	status = HAL_ADCEx_MultiModeStart_DMA(hadc1_, ADC1_Buffer, (uint32_t)(ADC1_BufferSize * 2));

	if(status) retval |= (1 << s);
		s++;

	return(retval);
}

void bsw::interface::AdcReader::adcStop(void){
//	HAL_ADC_Stop(hadc2_);
//	HAL_ADCEx_MultiModeStop_DMA(hadc1_);
	HAL_ADC_Stop_DMA(hadc1_);
}

