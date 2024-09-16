#ifndef FRAME_BUFFER_HPP__
#define FRAME_BUFFER_HPP__

#include <vector>
#include <unordered_map>

//* utilities
#include "bsw/interface/user_buffer.hpp"
#include "rte/util/dbc_interpreter.hpp"
#include "rte/util/mutex.hpp"

namespace bsw{
namespace interface{

class MsgBuffer{
public:
	MsgBuffer(UserBuffer* user_buffer) {
		initFrameBuffer(user_buffer);
		msg_ids_[CAN_CH1] = getBufferIds(CAN_CH1);
		msg_ids_[CAN_CH2] = getBufferIds(CAN_CH2);
		msg_ids_[CAN_CH3] = getBufferIds(CAN_CH3);
	};
	~MsgBuffer() {};

	bool isListed(const uint8_t& can_ch, const uint32_t& can_id);
	can_msg_t getBuffer(const uint8_t& can_ch, const uint32_t& can_id);
	std::vector<uint32_t> getBufferIds(const uint8_t& can_ch);
	void setBuffer(const uint8_t& can_ch, const can_msg_t& msg);
	void setUserBuffer(UserBuffer* user_buffer);

private:
	void initFrameBuffer(UserBuffer* user_buffer);

private:
	//* message buffers
	util::mutex mutex_[CAN_CH_LEN];
	bool is_init_ = false;
	//*  unordered_map<CAN_ID, CAN_FRAME>
	std::unordered_map<uint32_t, bsw::interface::can_msg_t> msg_buffer_[CAN_CH_LEN];
	std::unordered_map<uint32_t, uint8_t> msg_buffer_counter_[CAN_CH_LEN];
	std::vector<uint32_t> msg_ids_[CAN_CH_LEN];
};



}
}














#endif
