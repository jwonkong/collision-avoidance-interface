/*
 * buffer.hpp
 *
 *  Created on: Apr 25, 2023
 *      Author: minchul
 */

#ifndef INC_BSW_BUFFER_HPP_
#define INC_BSW_BUFFER_HPP_

#include "bsw/interface/user_buffer.hpp"
#include "rte/util/dbc_interpreter.hpp"

namespace bsw{
namespace interface{

class Buffer : public UserBuffer {

public:
	Buffer() {};
	~Buffer() {};

	void init(std::unordered_map<uint32_t, bsw::interface::can_msg_t>& buffer, const int& channel);
};


}
}



#endif /* INC_BSW_BUFFER_HPP_ */
