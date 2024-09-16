/*
 * user_buffer.hpp
 *
 *  Created on: Apr 25, 2023
 *      Author: minchul
 */

#ifndef A1_INC_BSW_INTERFACE_USER_BUFFER_HPP_
#define A1_INC_BSW_INTERFACE_USER_BUFFER_HPP_

#include <stdint.h>
#include <unordered_map>
#include "bsw/interface/type.hpp"

namespace bsw{
namespace interface{

class UserBuffer{

public:
	UserBuffer() {};
	~UserBuffer() {};

	virtual void init(std::unordered_map<uint32_t, bsw::interface::can_msg_t>& buffer, const int& channel) = 0;
};

}
}
#endif /* A1_INC_BSW_INTERFACE_USER_BUFFER_HPP_ */
