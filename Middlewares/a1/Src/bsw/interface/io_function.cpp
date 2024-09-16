/*
 * io_function.c
 *
 *  Created on: Oct 8, 2021
 *      Author: NB59
 */

#include "bsw/interface/io_function.hpp"
#include "main.h"

using namespace bsw;
using namespace interface;

void IoFunction::joinCanBus(){
	is_detached_ = false;
	HAL_GPIO_WritePin(RELAY5_GPIO_Port, RELAY5_Pin, GPIO_PinState::GPIO_PIN_RESET);
}

void IoFunction::detachCanBus(){
	is_detached_ = true;
	HAL_GPIO_WritePin(RELAY5_GPIO_Port, RELAY5_Pin, GPIO_PinState::GPIO_PIN_SET);
}

void IoFunction::setRelay(const uint8_t& relay, RelayState state){

	switch(relay){

	case 1:
		HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, static_cast<GPIO_PinState>(state));
		break;
	case 2:
		HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, static_cast<GPIO_PinState>(state));
		break;
	case 3:
		HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, static_cast<GPIO_PinState>(state));
		break;
	case 4:
		HAL_GPIO_WritePin(RELAY4_GPIO_Port, RELAY4_Pin, static_cast<GPIO_PinState>(state));
		break;
	case 5:
		HAL_GPIO_WritePin(RELAY5_GPIO_Port, RELAY5_Pin, static_cast<GPIO_PinState>(state));
		break;
	default:
		break;
	}
}

void IoFunction::setSSRelay(uint8_t SSRelay, SSRelayMode mode) {
	if (SSRelay == 4) {
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY12_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY11_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY10_Pin, GPIO_PIN_RESET);
		switch(mode){
			case BATT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY10_Pin, GPIO_PIN_SET);
				break;
			case DAC_OUT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY11_Pin, GPIO_PIN_SET);
				break;
			case GROUND:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY12_Pin, GPIO_PIN_SET);
				break;

			default:
				break;
			}
	} else if (SSRelay == 3) {
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY9_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY7_Pin, GPIO_PIN_RESET);
		switch(mode){
			case BATT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY7_Pin, GPIO_PIN_SET);
				break;
			case DAC_OUT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY8_Pin, GPIO_PIN_SET);
				break;
			case GROUND:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY9_Pin, GPIO_PIN_SET);
				break;

			default:
				break;
		}
	} else if (SSRelay == 2) {
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY4_Pin, GPIO_PIN_RESET);
		switch(mode){
			case BATT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY4_Pin, GPIO_PIN_SET);
				break;
			case DAC_OUT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY5_Pin, GPIO_PIN_SET);
				break;
			case GROUND:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY6_Pin, GPIO_PIN_SET);
				break;

			default:
				break;
		}
	} else if (SSRelay == 1) {
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY1_Pin, GPIO_PIN_RESET);
		switch(mode){
			case BATT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY1_Pin, GPIO_PIN_SET);
				break;
			case DAC_OUT:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY2_Pin, GPIO_PIN_SET);
				break;
			case GROUND:
				HAL_GPIO_WritePin(SSRELAY_GPIO_Port, SSRELAY3_Pin, GPIO_PIN_SET);
				break;

			default:
				break;
		}
	}
}
