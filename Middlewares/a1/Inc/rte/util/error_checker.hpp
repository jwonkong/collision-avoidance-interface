#ifndef ERROR_CHECKER_HPP_
#define ERROR_CHECKER_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdarg>

namespace util {
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

class ErrorChecker {
  typedef enum {
	kErrorInvalidTargetCommand =	0b00000001 ,
	kErrorCRCmatch = 				0b00000010,
	kErrorTimeout = 				0b00000100,
	kErrorCustom01 = 				0b00001000,
	kErrorCustom02 = 				0b00010000,
	kErrorCustom03 = 				0b00100000,
	kErrorCustom04 = 				0b01000000,
	kErrorCustom05 = 				0b10000000,
  } ErrorStatus;

public:
  ErrorChecker(){
	  len_error_code_ = 0;
  };
  ~ErrorChecker(){};

public:
  void setErrorCodeLength(const uint8_t& len) { len_error_code_ = len; }

  //*	@brief 	return controller error code
  uint8_t getStatus(const bool valid_cmd, const bool valid_crc, const bool alive_cmd, ...){
    uint8_t error_status = 0;
    va_list ap;

    if (valid_cmd == false) error_status += kErrorInvalidTargetCommand;
    if (valid_crc == false) error_status += kErrorCRCmatch;
    if (alive_cmd == false) error_status += kErrorTimeout;

    va_start(ap, alive_cmd);
    for(int i = 3; i < len_error_code_; i++) {
      uint8_t args = va_arg(ap, int);
      if (args == 0) {
    	error_status += (1 << i);
      }
    }
    va_end(ap);

	return error_status;
  }
private:
//  std::unordered_map<std::string, uint8_t> error_code_bit_;
  uint8_t len_error_code_;

};
} // namespace util

#endif
