#include <rte/util/crc_calculator.hpp>

using namespace util;

#define WIDTH  (8 * sizeof(unsigned char))
#define TOPBIT (1 << (WIDTH - 1))

void CRCCalculator::initCrc8(uint8_t GP) {
  uint8_t remainder;

  for (int dividend = 0; dividend < 256; ++dividend) {
    remainder = dividend << (WIDTH - 8);

    for (uint8_t bit = 8; bit > 0; --bit) {
      if (remainder & TOPBIT)
        remainder = (remainder << 1) ^ GP;
      else
        remainder = (remainder << 1);
    }

    crc8_table_[dividend] = remainder;
  }
}

void CRCCalculator::initCrc16(uint16_t GP) {
  uint16_t remainder;

  for (uint16_t dividend = 0; dividend < 256; ++dividend) {
    remainder = dividend << ((16 * sizeof(unsigned char)) - 8);

    for (uint16_t bit = 8; bit > 0; --bit) {
      if (remainder & (1 << ((16 * sizeof(unsigned char)) - 1)))
        remainder = (remainder << 1) ^ GP;
      else
        remainder = (remainder << 1);
    }

    crc16_table_[dividend] = remainder;
  }
}

uint8_t CRCCalculator::getCrc8(uint8_t const message[],
                                 int nBytes,
                                 uint8_t DI) {
  uint8_t data;
  uint8_t remainder = DI;

  for (int byte = 0; byte < nBytes; ++byte) {
    data = message[byte] ^ (remainder >> (WIDTH - 8));
    remainder = crc8_table_[data] ^ (remainder << 8);
  }
  return (remainder);
}

uint8_t CRCCalculator::getCrc8_E2E(uint8_t const message[],
                                     int nBytes,
                                     uint16_t id) {
  uint8_t data;
  uint8_t remainder = 0x00;
  uint8_t dataid_l = (id)&0x00ff;
  uint8_t dataid_h = ((id)&0xff00) >> 8;

  remainder = crc8_table_[remainder ^ dataid_l];
  remainder = crc8_table_[remainder ^ dataid_h];
  for (int byte = 0; byte < nBytes; ++byte) {
    data = message[byte] ^ remainder;
    remainder = crc8_table_[data];
  }

  return remainder ^ 0xff;
}

uint16_t CRCCalculator::getCrc16_E2E(uint8_t const message[],
                                       int nBytes,
                                       uint16_t DI,
                                       uint16_t fXor,
                                       uint16_t id) {
  uint16_t crc = DI;
  uint16_t data;

  uint16_t data_id_l = (id)&0x00ff;
  uint16_t data_id_h = (id & 0xff00) >> 8;

  for (int byte = 0; byte < nBytes; ++byte) {
    data = (0x00ff & (unsigned short)message[byte]) ^ (crc >> 8);
    crc = (crc << 8) ^ crc16_table_[data];
  }
  crc = (crc << 8) ^ crc16_table_[((crc >> 8) ^ data_id_l) & 0x00ff];
  crc = (crc << 8) ^ crc16_table_[((crc >> 8) ^ data_id_h) & 0x00ff];

  return crc ^ fXor;
}
