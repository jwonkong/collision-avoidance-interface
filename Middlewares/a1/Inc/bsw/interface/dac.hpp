/*
 * dac.hpp
 *
 *  Created on: April 10, 2022
 *      Author: Zafra Juanen
 */

#ifndef INC_DAC_HPP__
#define INC_DAC_HPP__

#include "cmsis_os.h"
#include "stm32g4xx_hal.h"



namespace bsw{
namespace interface{

//#define ADC1_OVERSAMPLING 2
//
//#define N_SAMPLE_PER_CH 16	// ADC Buffer Size per channel
//
//static const uint8_t TOTAL_N_CH = 3; // Total ADC channels used.

class DacWriter{

public:
	DacWriter() {};
	~DacWriter() {};

	/**
	 * interface functions
	 */
	void initDac(DAC_HandleTypeDef* hadc);
	uint8_t dacStart(void);
	void dacStop(void);
	void setVoltage(uint8_t port, float voltage); //V
	DAC_HandleTypeDef* getInterface();

private:
	DAC_HandleTypeDef* hdac1_;
	DAC_HandleTypeDef* hdac2_;
	DAC_HandleTypeDef* hdac3_;

};
}
}


#endif /* INC_ADC_HPP_ */




