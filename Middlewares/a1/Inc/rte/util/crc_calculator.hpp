#ifndef CRC_CALCULATOR_HPP_
#define CRC_CALCULATOR_HPP_


namespace util {
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

class CRCCalculator {
public:
  CRCCalculator(){};
  ~CRCCalculator(){};

public:
  void initCrc8(uint8_t GP);
  void initCrc16(uint16_t GP);
  uint8_t getCrc8(uint8_t const message[], int nBytes, uint8_t DI);
  uint8_t getCrc8_E2E(uint8_t const message[], int nBytes, uint16_t id);
  uint16_t getCrc16_E2E(uint8_t const message[],
                        int nBytes,
                        uint16_t DI,
                        uint16_t fXor,
                        uint16_t id);

private:
  uint8_t crc8_table_[256];
  uint16_t crc16_table_[256];
};
}  // namespace util

#endif
