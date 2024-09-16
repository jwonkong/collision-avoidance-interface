#ifndef BOOT_DBC_HPP__
#define BOOT_DBC_HPP__

#include <stdint.h>
#include <cstring>
#include <string>
#include <cmath>

namespace dbc {

#ifndef SIGNAL_INTERFACE_GENERAL
#define SIGNAL_INTERFACE_GENERAL
const uint8_t BYTE_LENGTH = 8;
enum class ByteOrder { BigEndian = 0, LittleEndian };
enum class ValueType { Unsigned = 0, Signed, Float, Double };
typedef struct signal_config {
  ValueType sign;
  double factor;
  double offset;
  uint8_t start;
  uint8_t length;
  ByteOrder order;
} SignalConfig;
typedef struct msg_flags {
  bool fd;
  bool brs;
} MsgFlags;
class MessageBase {
public:
  MessageBase(uint8_t* data,
              const uint32_t& _id = 0,
              const uint8_t& _length = 0,
              const bool& _fd = false,
              const bool& _brs = false)
      : _data(data), id(_id), length(_length), flags({_fd, _brs}){};
  uint32_t id;
  uint8_t length;
  MsgFlags flags;
  uint8_t counter = 0;
  uint8_t* _data;
  virtual void checkMemoryPointer(void) = 0;
  inline double get(const SignalConfig& config) {
    checkMemoryPointer();
    return getSignal(_data, length, config);
  };
  inline void set(const double& value, const SignalConfig& config) {
    checkMemoryPointer();
    setSignal(static_cast<double>(value), _data, length, config);
  };
private:
  double getSignal(uint8_t* data,
                   const uint8_t& data_len,
                   const SignalConfig& config) {
    uint8_t mask_offset = 0;
    uint64_t raw = 0;
    if(config.order == ByteOrder::BigEndian){
      uint8_t start_byte = config.start / BYTE_LENGTH;
      uint8_t remainder = BYTE_LENGTH - (config.start % BYTE_LENGTH);
      uint8_t blength = (config.length - remainder) / BYTE_LENGTH;
      if (config.length > remainder)
        if ((config.length % BYTE_LENGTH) != 0) blength++;
      blength++;

      uint8_t len_rem = config.length;
      for (uint8_t i = 0; i < blength; i++) {
        uint8_t mask = ((1 << remainder) - 1) << mask_offset;
        if (len_rem < remainder) {
          mask &= ~(((1 << (remainder - len_rem)) - 1) << mask_offset);
          uint8_t tmp_data = *(data + start_byte + i);
          tmp_data &= mask;
          raw += tmp_data >> (remainder - len_rem);
        } else {
          uint8_t tmp_data = *(data + start_byte + i);
          tmp_data &= mask;
          len_rem -= remainder;
          if (config.sign != ValueType::Double)
            raw += ((uint64_t)tmp_data << len_rem) >> mask_offset;
          else
            raw += ((uint64_t)tmp_data << len_rem) >> mask_offset;
        }

        if ((len_rem / BYTE_LENGTH) != 0) {
          remainder = 8;
        } else {
          mask_offset = BYTE_LENGTH - len_rem;
          remainder = len_rem;
        }
      }
    }
    else{
      uint8_t end = config.start + config.length - 1;
      uint8_t start_byte = config.start / BYTE_LENGTH;
      uint8_t remainder = (end + 1) % BYTE_LENGTH;
      if(remainder == 0) remainder = BYTE_LENGTH;
      uint8_t blength = (config.length - remainder) / BYTE_LENGTH;
      if (config.length > remainder)
        if (((config.length - remainder) % BYTE_LENGTH) != 0) blength++;
      blength++;

      uint8_t len_rem = config.length;
      for (uint8_t i = 0; i < blength; i++) {
        uint8_t mask = ((1 << remainder) - 1) << mask_offset;
        if (len_rem < remainder) {
          mask &= ~(((1 << (remainder - len_rem)) - 1) << mask_offset);
          uint8_t tmp_data = *(data + (blength - 1) + start_byte - i);
          tmp_data &= mask;
          raw += tmp_data >> (remainder - len_rem);
        } else {
          uint8_t tmp_data = *(data + (blength - 1) + start_byte - i);
          tmp_data &= mask;
          len_rem -= remainder;
          if (config.sign != ValueType::Double)
            raw += ((uint64_t)tmp_data << len_rem) >> mask_offset;
          else
            raw += ((uint64_t)tmp_data << len_rem) >> mask_offset;
        }

        if ((len_rem / BYTE_LENGTH) != 0) {
          remainder = 8;
        } else {
          mask_offset = BYTE_LENGTH - len_rem;
          remainder = len_rem;
        }
      }
    }
    if (config.sign != ValueType::Double)
      raw &= (((uint64_t)1 << config.length) - 1);

    if (config.sign == ValueType::Signed) {
      uint64_t msb = 1 << (config.length - 1);
      if (msb & raw) {
        raw &= ~msb;
        raw = msb - raw;
        return -1. * config.factor * static_cast<double>(raw) + config.offset;
      }
    } else if (config.sign == ValueType::Float) {
      uint32_t raw_32 = static_cast<uint32_t>(raw);
      float* raw_f = (float*)(&raw_32);
      return config.factor * static_cast<double>(*raw_f) + config.offset;
    } else if (config.sign == ValueType::Double) {
      double* raw_d = (double*)(&raw);
      return config.factor * (*raw_d) + config.offset;
    }
    return config.factor * static_cast<double>(raw) + config.offset;
  }
  void setSignal(const double& value,
                 uint8_t* data,
                 const uint8_t& data_len,
                 const SignalConfig& config) {
    uint64_t raw = 0;
    if (config.sign == ValueType::Float) {
      float v = static_cast<float>(value);
      float* p = &v;
      uint32_t raw_pre = *((uint32_t*)p);
      raw = static_cast<uint64_t>(raw_pre);
    } else if (config.sign == ValueType::Double) {
      double v = value;
      double* p = &v;
      raw = *((uint64_t*)p);
    } else if (config.sign == ValueType::Signed){
      int64_t int_raw = static_cast<int64_t>(round((value - config.offset) / config.factor));
      raw = static_cast<uint64_t>(int_raw);
    } else {
      raw = static_cast<uint64_t>(round((value - config.offset) / config.factor));
    }
    if (config.sign != ValueType::Double)
      raw &= (((uint64_t)1 << config.length) - 1);

    if(config.order == ByteOrder::BigEndian){
      uint8_t start_byte = config.start / BYTE_LENGTH;
      uint8_t remainder = BYTE_LENGTH - (config.start % BYTE_LENGTH);
      uint8_t blength = (config.length - remainder) / BYTE_LENGTH;
      if (config.length > remainder)
        if (((config.length - remainder) % BYTE_LENGTH) != 0) blength++;
      blength++;

      uint8_t len_rem = config.length;
      uint8_t mask_offset = 0;
      for (uint8_t i = 0; i < blength; i++) {
        uint8_t mask = ((1 << remainder) - 1) << mask_offset;
        uint8_t tmp_value = 0;

        if (len_rem < remainder) {
          mask &= ~(((1 << (remainder - len_rem)) - 1) << mask_offset);
          tmp_value = static_cast<uint8_t>(raw) << (remainder - len_rem);
        } else {
          len_rem -= remainder;
          tmp_value = static_cast<uint8_t>(raw >> len_rem) << mask_offset;
        }
        raw -= static_cast<uint64_t>(tmp_value) << len_rem;

        uint8_t tmp_data = *(data + start_byte + i);
        tmp_data &= ~mask;
        tmp_data |= tmp_value;
        *(data + start_byte + i) = tmp_data;

        if ((len_rem / BYTE_LENGTH) != 0) {
          remainder = 8;
        } else {
          mask_offset = BYTE_LENGTH - len_rem;
          remainder = len_rem;
        }
      }
    }
    else{
      uint8_t end = config.start + config.length - 1;
      uint8_t start_byte = config.start / BYTE_LENGTH;
      uint8_t remainder = (end + 1) % BYTE_LENGTH;
      if(remainder == 0) remainder = BYTE_LENGTH;
      uint8_t blength = (config.length - remainder) / BYTE_LENGTH;
      if (config.length > remainder)
        if (((config.length - remainder) % BYTE_LENGTH) != 0) blength++;
      blength++;

      uint8_t len_rem = config.length;
      uint8_t mask_offset = 0;
      for (uint8_t i = 0; i < blength; i++) {
        uint8_t mask = ((1 << remainder) - 1) << mask_offset;
        uint8_t tmp_value = 0;

        if (len_rem < remainder) {
          mask &= ~(((1 << (remainder - len_rem)) - 1) << mask_offset);
          tmp_value = static_cast<uint8_t>(raw) << (remainder - len_rem);
        } else {
          len_rem -= remainder;
          tmp_value = static_cast<uint8_t>(raw >> len_rem) << mask_offset;
        }
        raw -= static_cast<uint64_t>(tmp_value) << len_rem;

        uint8_t tmp_data = *(data + (blength - 1) + start_byte - i);
        tmp_data &= ~mask;
        tmp_data |= tmp_value;
        *(data + (blength - 1) + start_byte - i) = tmp_data;

        if ((len_rem / BYTE_LENGTH) != 0) {
          remainder = 8;
        } else {
          mask_offset = BYTE_LENGTH - len_rem;
          remainder = len_rem;
        }
      }
    }
  }
};
#endif

namespace boot {
enum class Id { 
  UpdateResponse = 2017,
  UpdateRequest = 1639,
  VersionResponse = 1537,
  VersionRequest = 1536,
};

#define boot_UpdateResponse_xcp_res {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateResponse_xcp_res_enable {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateResponse_xcp_byte_order {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateResponse_xcp_cto_packet_len {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateResponse_xcp_dto_packet_len {ValueType::Unsigned, 1, 0, 32, 16, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateResponse_xcp_version_protocol_layer {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateResponse_xcp_version_transport_layer {ValueType::Unsigned, 1, 0, 56, 8, ByteOrder::LittleEndian} // unit: []
class UpdateResponse : public MessageBase {
public:
  UpdateResponse(uint8_t* arr = nullptr): MessageBase(&data[0], 2017, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t xcp_res() { return static_cast<uint8_t>(get(boot_UpdateResponse_xcp_res)); };
  inline uint8_t xcp_res_enable() { return static_cast<uint8_t>(get(boot_UpdateResponse_xcp_res_enable)); };
  inline uint8_t xcp_byte_order() { return static_cast<uint8_t>(get(boot_UpdateResponse_xcp_byte_order)); };
  inline uint8_t xcp_cto_packet_len() { return static_cast<uint8_t>(get(boot_UpdateResponse_xcp_cto_packet_len)); };
  inline uint16_t xcp_dto_packet_len() { return static_cast<uint16_t>(get(boot_UpdateResponse_xcp_dto_packet_len)); };
  inline uint8_t xcp_version_protocol_layer() { return static_cast<uint8_t>(get(boot_UpdateResponse_xcp_version_protocol_layer)); };
  inline uint8_t xcp_version_transport_layer() { return static_cast<uint8_t>(get(boot_UpdateResponse_xcp_version_transport_layer)); };
  //* set interface
  inline void xcp_res(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_res); };
  inline void xcp_res_enable(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_res_enable); };
  inline void xcp_byte_order(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_byte_order); };
  inline void xcp_cto_packet_len(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_cto_packet_len); };
  inline void xcp_dto_packet_len(const uint16_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_dto_packet_len); };
  inline void xcp_version_protocol_layer(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_version_protocol_layer); };
  inline void xcp_version_transport_layer(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateResponse_xcp_version_transport_layer); };
};
#define boot_UpdateRequest_xcp_req {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateRequest_boot_id {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define boot_UpdateRequest_signature {ValueType::Unsigned, 1, 0, 16, 48, ByteOrder::LittleEndian} // unit: []
class UpdateRequest : public MessageBase {
public:
  UpdateRequest(uint8_t* arr = nullptr): MessageBase(&data[0], 1639, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t xcp_req() { return static_cast<uint8_t>(get(boot_UpdateRequest_xcp_req)); };
  inline uint8_t boot_id() { return static_cast<uint8_t>(get(boot_UpdateRequest_boot_id)); };
  inline uint64_t signature() { return static_cast<uint64_t>(get(boot_UpdateRequest_signature)); };
  //* set interface
  inline void xcp_req(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateRequest_xcp_req); };
  inline void boot_id(const uint8_t& value) { set(static_cast<double>(value), boot_UpdateRequest_boot_id); };
  inline void signature(const uint64_t& value) { set(static_cast<double>(value), boot_UpdateRequest_signature); };
};
#define boot_VersionResponse_response_boot_id {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define boot_VersionResponse_version_patch {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define boot_VersionResponse_version_minor {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define boot_VersionResponse_version_major {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class VersionResponse : public MessageBase {
public:
  VersionResponse(uint8_t* arr = nullptr): MessageBase(&data[0], 1537, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  //* get interface
  inline uint8_t response_boot_id() { return static_cast<uint8_t>(get(boot_VersionResponse_response_boot_id)); };
  inline uint8_t version_patch() { return static_cast<uint8_t>(get(boot_VersionResponse_version_patch)); };
  inline uint8_t version_minor() { return static_cast<uint8_t>(get(boot_VersionResponse_version_minor)); };
  inline uint8_t version_major() { return static_cast<uint8_t>(get(boot_VersionResponse_version_major)); };
  //* set interface
  inline void response_boot_id(const uint8_t& value) { set(static_cast<double>(value), boot_VersionResponse_response_boot_id); };
  inline void version_patch(const uint8_t& value) { set(static_cast<double>(value), boot_VersionResponse_version_patch); };
  inline void version_minor(const uint8_t& value) { set(static_cast<double>(value), boot_VersionResponse_version_minor); };
  inline void version_major(const uint8_t& value) { set(static_cast<double>(value), boot_VersionResponse_version_major); };
};
#define boot_VersionRequest_target_boot_id {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
class VersionRequest : public MessageBase {
public:
  VersionRequest(uint8_t* arr = nullptr): MessageBase(&data[0], 1536, 1, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[1];
  //* value table
  //* get interface
  inline uint8_t target_boot_id() { return static_cast<uint8_t>(get(boot_VersionRequest_target_boot_id)); };
  //* set interface
  inline void target_boot_id(const uint8_t& value) { set(static_cast<double>(value), boot_VersionRequest_target_boot_id); };
};

} // namespace boot
} // namespace dbc
#endif // BOOT_DBC_HPP__
