#ifndef RTE_HPP__
#define RTE_HPP__

#include <vector>
#include <unordered_map>

//* bsws
#include "bsw/interface/can_transfer.hpp"
#include "bsw/interface/io_function.hpp"
#include "bsw/interface/adc.hpp"
#include "bsw/interface/dac.hpp"

//* utilities
#include "rte/util/dbc_interpreter.hpp"
#include "rte/util/crc_calculator.hpp"

//* validation
#include "app/validation.h"

//* speed_filter
#include "app/speed_filter.hpp"

namespace rte {

#define QUEUE_SIZE 40
#define BOOT_WAIT_THRESHOLD 150
class Rte {

public:
	Rte(bsw::interface::CanTransfer* can_transfer,
		bsw::interface::IoFunction* io_function,
		bsw::interface::AdcReader* adc_reader,
		bsw::interface::DacWriter* dac_writer);

	Rte(bsw::interface::CanTransfer* can_transfer,
		bsw::interface::IoFunction* io_function);
	~Rte() {};

public:
	void initRte(void);
	bool isInit(void) { return init_flag_; };

	void task20ms(void);
	void task100ms(void);
	void task200ms(void);

	uint8_t checkValueInQueue(std::vector<uint8_t> queue);
	void pushQueue(std::vector<uint8_t>& queue, uint8_t value);

private:
	//* bsw
	bsw::interface::CanTransfer* can_transfer_;
	bsw::interface::IoFunction* io_function_;
	bsw::interface::AdcReader* adc_reader_;
	bsw::interface::DacWriter* dac_writer_;
	//* utilities
	util::CRCCalculator crc_calc_;
	//* rte
	bool init_flag_;
	//* app

	uint8_t boot_counter_;
};

}
#endif
