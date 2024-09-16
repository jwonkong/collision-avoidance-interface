#ifndef CRC_CHECKER_HPP_
#define CRC_CHECKER_HPP_


#include <string>
#include <vector>
#include <unordered_map>
#include <cstdarg>

namespace util {
typedef enum {
	kNa = 0,
	kWarning,
	kError
}StatusCrc;

typedef enum {
	kWarningCriteria = 20,
	kErrorCriteria = 40
}StatusCriteria;

class CrcChecker {


public:
	CrcChecker()
	: counter_crc_invalid_(0),
		crc_status_(StatusCrc::kNa)
	{}
	~CrcChecker() {}

public:
	/** @brief 	return CRC validation status
	 * 	@param	CRC comparison(command frame CRC == calculated CRC) results
	 * 	@ret		CRC status
	 */
	uint8_t getStatusCrc(const bool match_crc) {
		if (match_crc) {
			if (counter_crc_invalid_ > 0) counter_crc_invalid_--;
		}
		else {
			if (counter_crc_invalid_ < 40) counter_crc_invalid_ += 4;
		}

		if (counter_crc_invalid_ >= StatusCriteria::kErrorCriteria) {
			crc_status_ = StatusCrc::kError;
		}
		else if (counter_crc_invalid_ >= StatusCriteria::kWarningCriteria) {
			crc_status_ = StatusCrc::kWarning;
		}
		else {
			crc_status_ = StatusCrc::kNa;
		}

		return crc_status_;
	}
	uint8_t getStatusValue(void) {
		return (uint8_t)crc_status_;
	}

	uint8_t getInvalidCount(void) {
		return counter_crc_invalid_;
	}

private:
	uint8_t counter_crc_invalid_;
	StatusCrc crc_status_;
};
}




#endif /* CRC_CHECKER_HPP_ */
