/*
 * io_function.h
 *
 *  Created on: Oct 8, 2021
 *      Author: NB59
 */

#ifndef INC_IO_FUNCTION_HPP__
#define INC_IO_FUNCTION_HPP__

#include "cmsis_os.h"
#include "stm32g4xx_hal.h"

namespace bsw{
namespace interface{

typedef enum {
	BATT = 0U,
	DAC_OUT = 1U,
	GROUND = 2,
	NO = 3
} SSRelayMode;

typedef enum
{
	OFF = 0U,
	ON
}RelayState;

class IoFunction{

public:
	IoFunction() {};
	~IoFunction() {};

	/**
	 * interface functions
	 */
	void joinCanBus();
	void detachCanBus();
	void setRelay(const uint8_t& relay, RelayState state);
	void setSSRelay(uint8_t SSRelay, SSRelayMode mode);
	bool isDetachedCANBus(void) { return is_detached_; };

private:
	bool is_detached_ = false;
};
}
}


#endif /* INC_IO_FUNCTION_H_ */
