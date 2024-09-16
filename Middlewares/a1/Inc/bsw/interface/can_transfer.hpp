#ifndef CAN_TRANSFER_HPP__
#define CAN_TRANSFER_HPP__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "cmsis_os.h"
#include "stm32g4xx_hal.h"

#include "bsw/interface/frame_buffer.hpp"

void canRxTask(void const * argument);

namespace bsw{
namespace interface{

typedef struct {
	uint8_t can_ch;
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[64];
} rx_queue_msg_t;

typedef struct {
	uint8_t can_ch;
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[64];
} tx_queue_msg_t;

static const uint32_t LOG_MSG_LEN = 100;
static const uint32_t RESEND_TRIAL = 10;

const uint32_t can_data_length[65] = {
			FDCAN_DLC_BYTES_0,  FDCAN_DLC_BYTES_1,  FDCAN_DLC_BYTES_2,
			FDCAN_DLC_BYTES_3,  FDCAN_DLC_BYTES_4,  FDCAN_DLC_BYTES_5,
			FDCAN_DLC_BYTES_6,  FDCAN_DLC_BYTES_7,  FDCAN_DLC_BYTES_8,
			FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_12,
			FDCAN_DLC_BYTES_12, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16,
			FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_20,
			FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20, FDCAN_DLC_BYTES_20,
			FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_24,
			FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
			FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
			FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32, FDCAN_DLC_BYTES_32,
			FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
			FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
			FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
			FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
			FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_48,
			FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
			FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
			FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
			FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
			FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
			FDCAN_DLC_BYTES_64, FDCAN_DLC_BYTES_64,
	};

int can_transfer_send(uint8_t can_ch, uint32_t can_id, const uint8_t *data, size_t data_size, bool fd, bool brs);

class CanTransfer {
public:
	/**
	 * Constructor & destructor
	 */
	CanTransfer() {};
	~CanTransfer() {};

	/**
	 * Interface functions
	 */
	void initUserBuffer(UserBuffer* user_buffer);
	void init(FDCAN_HandleTypeDef* hfdcan1, FDCAN_HandleTypeDef* hfdcan2, FDCAN_HandleTypeDef* hfdcan3);

	void initMessageFilters(void);

	bool setIdFilter(uint8_t can_ch, std::vector<uint32_t> id_list);

	int turnOnBus(uint8_t can_ch, bool on);

	void setBypassMode(bool on);

	void setCaseFilter(const std::string& c, const std::vector<uint32_t> arr);
	bool isFiltered(const std::string& c, const uint32_t& id);
	void setCase(const std::string& c, const bool& flag);

	int send(uint8_t can_ch, const can_msg_t& msg);
	int sendRegisterStatus(uint8_t can_ch);
	int sendWoBuffering(uint8_t can_ch, const can_msg_t& msg);

	bool recv(uint8_t can_ch, can_msg_t& msg);
	can_msg_t getBuffer(uint8_t can_ch, uint32_t can_id);
	void setBuffer(uint8_t can_ch, can_msg_t msg);
	uint8_t getBufferSize(uint8_t can_ch);
	uint8_t getFilterSize(uint8_t can_ch);

	int configTiming(uint32_t can_ch, uint16_t prescaler,
	                               uint16_t sjw, uint16_t tseg1, uint16_t tseg2,
	                               uint16_t data_prescaler, uint16_t data_sjw,
	                               uint16_t data_tseg1, uint16_t data_tseg2,
	                               bool fd);

	int busState(uint32_t can_ch, bool *bus_off, bool *error_passive,
	                           uint8_t *error_code, uint8_t *data_error_code);

	int registHalfSignal(uint8_t can_ch,
	                                    osThreadId half_signal_thread,
	                                    int32_t half_signal_value);

	void incrementTimestemp(void);

	bool getBusErrorPassive(uint8_t can_ch);
	bool isBusOn(uint8_t can_ch);

	uint8_t getTxPointer(void);
	void setBus2DeviceBypassMode(const bool& flag);
	void setMsgFilteringMode(const bool& flag);
private:
	/**
	 * CAN interface variables
	 */
	FDCAN_HandleTypeDef* hfdcan1_;
	FDCAN_HandleTypeDef* hfdcan2_;
	FDCAN_HandleTypeDef* hfdcan3_;
};
}
}
#endif  //  CAN_TRANSFER_HPP__
