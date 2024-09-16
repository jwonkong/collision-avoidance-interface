/*
 * adc.hpp
 *
 *  Created on: April 5, 2022
 *      Author: Zafra Juanen
 */

#ifndef INC_ADC_HPP__
#define INC_ADC_HPP__

#include "cmsis_os.h"
#include "stm32g4xx_hal.h"



namespace bsw{
namespace interface{

#define ADC1_OVERSAMPLING 2
// 0x Oversamling: ADC-REsolution = 12 bit (2^12 = 4096)
// 1x Oversampling: 12 bit --> 13 bit (2^13 = 8192) --> Buffer Size: 4^1 = 4
// 2x Oversampling: 12 bit --> 14 bit (2^14 = 16384) --> Buffer Size: 4^2 = 16
// 3x Oversampling: 12 bit --> 15 bit (2^15 = 32768) --> Buffer Size: 4^3 = 64
// 4x Oversampling: 12 bit --> 16 bit (2^16 = 65535) --> Buffer Size: 4^4 = 256
// Buffer size must match oversampling!!
#define N_SAMPLE_PER_CH 16	// ADC Buffer Size per channel

static const uint8_t TOTAL_N_CH = 3; // Total ADC channels used.

class AdcReader{

public:
	AdcReader() {};
	~AdcReader() {};

	/**
	 * interface functions
	 */
	void initAdc(ADC_HandleTypeDef* hadc);
	uint8_t adcStart(void);
	void adcStop(void);
	float* getVoltage();
	void setPending(bool status);
	bool getPending();

private:
	ADC_HandleTypeDef* hadc1_;
	ADC_HandleTypeDef* hadc2_;
	ADC_HandleTypeDef* hadc3_;

};
}
}


#endif /* INC_ADC_HPP_ */




