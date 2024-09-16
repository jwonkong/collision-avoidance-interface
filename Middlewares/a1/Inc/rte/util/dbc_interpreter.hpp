#ifndef DBC_INTERPRETER_HPP__
#define DBC_INTERPRETER_HPP__

#include <cstring>

#include "rte/db/v_can.hpp"
#include "rte/db/ad_can.hpp"
#include "rte/db/camera.hpp"
#include "rte/db/boot.hpp"

#define initBuffer(buffer, type) ({\
	type msg;\
	bsw::interface::can_msg_t frame;\
	frame.can_id = msg.id;\
	frame.data_size = msg.length; \
	frame.brs = msg.flags.brs; \
	frame.fd = msg.flags.fd; \
	frame.is_completed = false;\
	frame.counter = 0;\
	memset(frame.data, 0, 64);\
	buffer[msg.id] = frame; })
#define getBufferedMessage2(ch, msg_type, msg_name) msg_type msg_name; ({\
	auto f = can_transfer_->getBuffer(ch, msg_name.id);\
	msg_name.counter = f.counter;\
	memcpy(&msg_name.data[0], &f.data[0], msg_name.length);})
#define getBufferedMessage(ch, msg_type) ({\
	msg_type msg;\
	auto f = can_transfer_->getBuffer(ch, msg.id);\
	msg.counter = f.counter;\
	memcpy(&msg.data[0], &f.data[0], msg.length);\
	msg;})
#define sendMessage(ch, msg) ({\
	bsw::interface::can_msg_t frame;\
	frame.can_id = msg.id;\
	frame.data_size = msg.length; \
	frame.brs = msg.flags.brs; \
	frame.fd = msg.flags.fd; \
	memcpy(&frame.data[0], &msg.data[0], msg.length);\
	can_transfer_->send(ch, frame);\
	})
#define setMessage2Buffer(ch, msg) ({\
	bsw::interface::can_msg_t frame;\
	frame.can_id = msg.id;\
	frame.data_size = msg.length; \
	frame.brs = msg.flags.brs; \
	frame.fd = msg.flags.fd; \
	memcpy(&frame.data[0], &msg.data[0], msg.length);\
	can_transfer_->setBuffer(ch, frame);\
	})
#define bypassFrame(from, id, to) ({\
	auto f = can_transfer_->getBuffer(from, (uint32_t)id);\
	can_transfer_->send(to, f);\
})
#define sendBufferedMsg(ch, id) ({\
	bypassFrame(ch, id, ch);\
})

#endif
