#ifndef A1_BSW_TYPE_HPP__
#define A1_BSW_TYPE_HPP__

#define CAN_CH1 0
#define CAN_CH2 1
#define CAN_CH3 2

#define FUC_LIVE_MSG_ID 0x667

#define EFF_FLAG 0x80000000

#define DIR_RX 0;
#define DIR_TX 1;

#define FDCAN_PSR_LEC_NO_CHANGE 0x7

namespace bsw{
namespace interface{

static const uint8_t CAN_CH_LEN = 3;

typedef struct {
			uint32_t can_id;
			uint16_t data_size;
			uint8_t data[64];
			bool fd;
			bool brs;
			uint32_t timestamp;
			uint8_t dir;
			uint8_t counter;
			bool is_completed;
		} can_msg_t;
}
}


#endif  //A1_BSW_TYPE_HPP__
