/*
 * adc.hpp
 *
 *  Created on: April 10, 2022
 *      Author: Zafra Juanen
 */
#include "bsw/interface/dac.hpp"
#include "bsw/interface/io_function.hpp"

#include "main.h"

static const float DAC_gain = 4.386; //  Used to calculate the DAC output

void bsw::interface::DacWriter::setVoltage(uint8_t port, float voltage) {
	uint32_t channel = DAC_CHANNEL_1;
	DAC_HandleTypeDef* hdac = hdac1_;

	if (port == 1) {
		channel = DAC_CHANNEL_2;
	} else if (port == 2) {
		hdac = hdac2_;
	} else if(port == 3) {
		hdac = hdac3_;
	}

	float d_volt = (voltage *4095) / (3.3F * DAC_gain);
	HAL_DAC_SetValue(hdac, channel, DAC_ALIGN_12B_R, (uint32_t)(d_volt));
}

void bsw::interface::DacWriter::initDac(DAC_HandleTypeDef* hdac) {
	if(hdac->Instance == DAC1) {
		hdac1_ = hdac;
	} else if (hdac->Instance == DAC2) {
		hdac2_ = hdac;
	} else if (hdac->Instance == DAC3) {
		hdac3_ = hdac;
	}

}

uint8_t bsw::interface::DacWriter::dacStart(void){
	uint8_t status = HAL_OK;
	uint8_t retval = 0;
	uint8_t s = 0;

	status = HAL_DAC_Start(hdac1_, DAC_CHANNEL_1);
	if(status) retval |= (1 << s);
	s++;
	status = HAL_DAC_Start(hdac1_, DAC_CHANNEL_2);
	if(status) retval |= (1 << s);
	s++;
	status = HAL_DAC_Start(hdac2_, DAC_CHANNEL_1);
	if(status) retval |= (1 << s);
	s++;
	status = HAL_DAC_Start(hdac3_, DAC_CHANNEL_1);
	if(status) retval |= (1 << s);
	s++;

	return(retval);
}

void bsw::interface::DacWriter::dacStop(void){
	HAL_DAC_Stop(hdac1_, DAC_CHANNEL_1);
	HAL_DAC_Stop(hdac1_, DAC_CHANNEL_2);
	HAL_DAC_Stop(hdac2_, DAC_CHANNEL_1);
	HAL_DAC_Stop(hdac3_, DAC_CHANNEL_1);
}
