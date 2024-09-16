
#include "bsw/interface/frame_buffer.hpp"

using namespace bsw;
using namespace interface;

bool MsgBuffer::isListed(const uint8_t& can_ch, const uint32_t& can_id){
	for (auto& id : msg_ids_[can_ch]) {
		if(id == can_id){
			return true;
		}
	}
	return false;
}

/** @brief	Get the target CAN frame.
 * 	@param	Target CAN channel.
 * 	@param	Target CAN frame ID.
 * 	@retval	Target CAN frame
 */
can_msg_t MsgBuffer::getBuffer(const uint8_t& can_ch, const uint32_t& can_id){
	can_msg_t msg;
	if(!is_init_) return msg;
	msg = msg_buffer_[can_ch][can_id];

	return msg;
}

/** @brief	Get every CAN message buffer ID.
 * 	@param	Target CAN channel.
 * 	@retval list of buffer ID
 */
std::vector<uint32_t> MsgBuffer::getBufferIds(const uint8_t& can_ch){
	std::vector<uint32_t> ids_;
	for (auto& buffer : msg_buffer_[can_ch]) ids_.push_back(buffer.first);

	return ids_;
}

/**	@brief	Update CAN frame buffer
 * 	@detail	Save CAN frame from CAN channel on real-time task.
 * 			Use double buffer to prevent buffer overwrite.
 *	@param	Target CAN channel.
 *	@param	CAN frame.
 *	@retval	None
 */
void MsgBuffer::setBuffer(const uint8_t& can_ch, const can_msg_t& msg){
	can_msg_t _msg = msg;
	_msg.counter = msg_buffer_[can_ch][_msg.can_id].counter + 1;
	if(_msg.counter == 0) _msg.counter = 1;
	msg_buffer_[can_ch][msg.can_id] = _msg;
	msg_buffer_[can_ch][msg.can_id].is_completed = true;
}

/**	@brief	Specifies the CAN ID to save CAN messages.
 * 	@retval	None
 */
using namespace dbc;
void MsgBuffer::initFrameBuffer(UserBuffer* user_buffer){
	initBuffer(msg_buffer_[CAN_CH1], boot::UpdateRequest);
	initBuffer(msg_buffer_[CAN_CH1], boot::VersionRequest);

	for (int ch = 0; ch < CAN_CH_LEN; ch++) {
		user_buffer->init(msg_buffer_[ch], ch);
	}

	is_init_ = true;
}
