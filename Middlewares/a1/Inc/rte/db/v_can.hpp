#ifndef V_CAN_DBC_HPP__
#define V_CAN_DBC_HPP__

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

namespace v_can {
enum class Id { 
  FUCResponse = 2017,
  FUCRequest = 1639,
  VersionResponse = 1537,
  VersionRequest = 1536,
  DebugRequest = 1104,
  DebugMsg = 1024,
  drv_alive = 848,
  DrivingLoggerInfo1 = 809,
  DrivingLoggerInfo0 = 790,
  UserConfiguration = 256,
  DoorInfo = 125,
  ChargingInfo = 119,
  IgnitionInfo = 118,
  GearInfo = 117,
  TurnSignalInfo = 116,
  LongitudinalInfo = 115,
  SteeringInfo = 114,
  WheelInfo = 113,
  DynamicInfo = 112,
  DoorControl = 87,
  ChargingDoorControl = 86,
  IgnitionControl = 85,
  TurnSignalControl = 84,
  LateralControl = 83,
  GearControl = 82,
  LongitudinalControl = 81,
  DoorState = 24,
  IgnitionState = 23,
  ChargingState = 22,
  GearState = 21,
  TurnSignalState = 20,
  LongitudinalState = 19,
  LateralState = 18,
  MCP251XFDState = 555,
  MCPFIFOState = 556,
  LateralDebug = 528,
  ZED_F9R_Frame = 529,
};

#define v_can_FUCResponse_xcp_res {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCResponse_xcp_res_enable {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCResponse_xcp_byte_order {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCResponse_xcp_cto_packet_len {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCResponse_xcp_dto_packet_len {ValueType::Unsigned, 1, 0, 32, 16, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCResponse_xcp_version_protocol_layer {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCResponse_xcp_version_transport_layer {ValueType::Unsigned, 1, 0, 56, 8, ByteOrder::LittleEndian} // unit: []
class FUCResponse : public MessageBase {
public:
  FUCResponse(uint8_t* arr = nullptr): MessageBase(&data[0], 2017, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t xcp_res() { return static_cast<uint8_t>(get(v_can_FUCResponse_xcp_res)); };
  inline uint8_t xcp_res_enable() { return static_cast<uint8_t>(get(v_can_FUCResponse_xcp_res_enable)); };
  inline uint8_t xcp_byte_order() { return static_cast<uint8_t>(get(v_can_FUCResponse_xcp_byte_order)); };
  inline uint8_t xcp_cto_packet_len() { return static_cast<uint8_t>(get(v_can_FUCResponse_xcp_cto_packet_len)); };
  inline uint16_t xcp_dto_packet_len() { return static_cast<uint16_t>(get(v_can_FUCResponse_xcp_dto_packet_len)); };
  inline uint8_t xcp_version_protocol_layer() { return static_cast<uint8_t>(get(v_can_FUCResponse_xcp_version_protocol_layer)); };
  inline uint8_t xcp_version_transport_layer() { return static_cast<uint8_t>(get(v_can_FUCResponse_xcp_version_transport_layer)); };
  //* set interface
  inline void xcp_res(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_res); };
  inline void xcp_res_enable(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_res_enable); };
  inline void xcp_byte_order(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_byte_order); };
  inline void xcp_cto_packet_len(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_cto_packet_len); };
  inline void xcp_dto_packet_len(const uint16_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_dto_packet_len); };
  inline void xcp_version_protocol_layer(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_version_protocol_layer); };
  inline void xcp_version_transport_layer(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCResponse_xcp_version_transport_layer); };
};
#define v_can_FUCRequest_xcp_req {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCRequest_boot_id {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_FUCRequest_signature {ValueType::Unsigned, 1, 0, 16, 48, ByteOrder::LittleEndian} // unit: []
class FUCRequest : public MessageBase {
public:
  FUCRequest(uint8_t* arr = nullptr): MessageBase(&data[0], 1639, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t xcp_req() { return static_cast<uint8_t>(get(v_can_FUCRequest_xcp_req)); };
  inline uint8_t boot_id() { return static_cast<uint8_t>(get(v_can_FUCRequest_boot_id)); };
  inline uint64_t signature() { return static_cast<uint64_t>(get(v_can_FUCRequest_signature)); };
  //* set interface
  inline void xcp_req(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCRequest_xcp_req); };
  inline void boot_id(const uint8_t& value) { set(static_cast<double>(value), v_can_FUCRequest_boot_id); };
  inline void signature(const uint64_t& value) { set(static_cast<double>(value), v_can_FUCRequest_signature); };
};
#define v_can_VersionResponse_response_boot_id {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_VersionResponse_version_patch {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_VersionResponse_version_minor {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_VersionResponse_version_major {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class VersionResponse : public MessageBase {
public:
  VersionResponse(uint8_t* arr = nullptr): MessageBase(&data[0], 1537, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  //* get interface
  inline uint8_t response_boot_id() { return static_cast<uint8_t>(get(v_can_VersionResponse_response_boot_id)); };
  inline uint8_t version_patch() { return static_cast<uint8_t>(get(v_can_VersionResponse_version_patch)); };
  inline uint8_t version_minor() { return static_cast<uint8_t>(get(v_can_VersionResponse_version_minor)); };
  inline uint8_t version_major() { return static_cast<uint8_t>(get(v_can_VersionResponse_version_major)); };
  //* set interface
  inline void response_boot_id(const uint8_t& value) { set(static_cast<double>(value), v_can_VersionResponse_response_boot_id); };
  inline void version_patch(const uint8_t& value) { set(static_cast<double>(value), v_can_VersionResponse_version_patch); };
  inline void version_minor(const uint8_t& value) { set(static_cast<double>(value), v_can_VersionResponse_version_minor); };
  inline void version_major(const uint8_t& value) { set(static_cast<double>(value), v_can_VersionResponse_version_major); };
};
#define v_can_VersionRequest_target_boot_id {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
class VersionRequest : public MessageBase {
public:
  VersionRequest(uint8_t* arr = nullptr): MessageBase(&data[0], 1536, 1, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[1];
  //* value table
  //* get interface
  inline uint8_t target_boot_id() { return static_cast<uint8_t>(get(v_can_VersionRequest_target_boot_id)); };
  //* set interface
  inline void target_boot_id(const uint8_t& value) { set(static_cast<double>(value), v_can_VersionRequest_target_boot_id); };
};
#define v_can_DebugRequest_dbg_toggle {ValueType::Unsigned, 1, 0, 0, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DebugRequest_ecu_id {ValueType::Unsigned, 1, 0, 2, 6, ByteOrder::LittleEndian} // unit: []
#define v_can_DebugRequest_dbg_frame_start_id {ValueType::Unsigned, 1, 0, 8, 32, ByteOrder::LittleEndian} // unit: []
#define v_can_DebugRequest_dbg_frame_id_scope {ValueType::Unsigned, 1, 0, 40, 16, ByteOrder::LittleEndian} // unit: []
class DebugRequest : public MessageBase {
public:
  DebugRequest(uint8_t* arr = nullptr): MessageBase(&data[0], 1104, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  //* get interface
  inline uint8_t dbg_toggle() { return static_cast<uint8_t>(get(v_can_DebugRequest_dbg_toggle)); };
  inline uint8_t ecu_id() { return static_cast<uint8_t>(get(v_can_DebugRequest_ecu_id)); };
  inline uint32_t dbg_frame_start_id() { return static_cast<uint32_t>(get(v_can_DebugRequest_dbg_frame_start_id)); };
  inline uint16_t dbg_frame_id_scope() { return static_cast<uint16_t>(get(v_can_DebugRequest_dbg_frame_id_scope)); };
  //* set interface
  inline void dbg_toggle(const uint8_t& value) { set(static_cast<double>(value), v_can_DebugRequest_dbg_toggle); };
  inline void ecu_id(const uint8_t& value) { set(static_cast<double>(value), v_can_DebugRequest_ecu_id); };
  inline void dbg_frame_start_id(const uint32_t& value) { set(static_cast<double>(value), v_can_DebugRequest_dbg_frame_start_id); };
  inline void dbg_frame_id_scope(const uint16_t& value) { set(static_cast<double>(value), v_can_DebugRequest_dbg_frame_id_scope); };
};

class DebugMsg : public MessageBase {
public:
  DebugMsg(uint8_t* arr = nullptr): MessageBase(&data[0], 1024, 8, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  //* set interface
};
#define v_can_drv_alive_is_aps10 {ValueType::Unsigned, 1, 0, 0, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_is_cmd30 {ValueType::Unsigned, 1, 0, 2, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_is_cmd31 {ValueType::Unsigned, 1, 0, 4, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_is_cmd32 {ValueType::Unsigned, 1, 0, 6, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_is_cmd21 {ValueType::Unsigned, 1, 0, 8, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_is_drv_dummy {ValueType::Unsigned, 1, 0, 10, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_is_drv01 {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_cnt_aps10 {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_cnt_cmd30 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_cnt_cmd31 {ValueType::Unsigned, 1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_cnt_cmd32 {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_cnt_cmd21 {ValueType::Unsigned, 1, 0, 56, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_drv_alive_cnt_aps {ValueType::Unsigned, 1, 0, 64, 8, ByteOrder::LittleEndian} // unit: []
class drv_alive : public MessageBase {
public:
  drv_alive(uint8_t* arr = nullptr): MessageBase(&data[0], 848, 12, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[12];
  //* value table
  //* get interface
  inline uint8_t is_aps10() { return static_cast<uint8_t>(get(v_can_drv_alive_is_aps10)); };
  inline uint8_t is_cmd30() { return static_cast<uint8_t>(get(v_can_drv_alive_is_cmd30)); };
  inline uint8_t is_cmd31() { return static_cast<uint8_t>(get(v_can_drv_alive_is_cmd31)); };
  inline uint8_t is_cmd32() { return static_cast<uint8_t>(get(v_can_drv_alive_is_cmd32)); };
  inline uint8_t is_cmd21() { return static_cast<uint8_t>(get(v_can_drv_alive_is_cmd21)); };
  inline uint8_t is_drv_dummy() { return static_cast<uint8_t>(get(v_can_drv_alive_is_drv_dummy)); };
  inline uint8_t is_drv01() { return static_cast<uint8_t>(get(v_can_drv_alive_is_drv01)); };
  inline uint8_t cnt_aps10() { return static_cast<uint8_t>(get(v_can_drv_alive_cnt_aps10)); };
  inline uint8_t cnt_cmd30() { return static_cast<uint8_t>(get(v_can_drv_alive_cnt_cmd30)); };
  inline uint8_t cnt_cmd31() { return static_cast<uint8_t>(get(v_can_drv_alive_cnt_cmd31)); };
  inline uint8_t cnt_cmd32() { return static_cast<uint8_t>(get(v_can_drv_alive_cnt_cmd32)); };
  inline uint8_t cnt_cmd21() { return static_cast<uint8_t>(get(v_can_drv_alive_cnt_cmd21)); };
  inline uint8_t cnt_aps() { return static_cast<uint8_t>(get(v_can_drv_alive_cnt_aps)); };
  //* set interface
  inline void is_aps10(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_aps10); };
  inline void is_cmd30(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_cmd30); };
  inline void is_cmd31(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_cmd31); };
  inline void is_cmd32(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_cmd32); };
  inline void is_cmd21(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_cmd21); };
  inline void is_drv_dummy(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_drv_dummy); };
  inline void is_drv01(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_is_drv01); };
  inline void cnt_aps10(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_cnt_aps10); };
  inline void cnt_cmd30(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_cnt_cmd30); };
  inline void cnt_cmd31(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_cnt_cmd31); };
  inline void cnt_cmd32(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_cnt_cmd32); };
  inline void cnt_cmd21(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_cnt_cmd21); };
  inline void cnt_aps(const uint8_t& value) { set(static_cast<double>(value), v_can_drv_alive_cnt_aps); };
};
#define v_can_DrivingLoggerInfo1_brake_signal {ValueType::Unsigned, 1, 0, 33, 1, ByteOrder::LittleEndian} // unit: []
class DrivingLoggerInfo1 : public MessageBase {
public:
  DrivingLoggerInfo1(uint8_t* arr = nullptr): MessageBase(&data[0], 809, 5, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[5];
  //* value table
  //* get interface
  inline uint8_t brake_signal() { return static_cast<uint8_t>(get(v_can_DrivingLoggerInfo1_brake_signal)); };
  //* set interface
  inline void brake_signal(const uint8_t& value) { set(static_cast<double>(value), v_can_DrivingLoggerInfo1_brake_signal); };
};
#define v_can_DrivingLoggerInfo0_engine_rpm {ValueType::Unsigned, 0.25, 0, 16, 16, ByteOrder::LittleEndian} // unit: []
#define v_can_DrivingLoggerInfo0_vehicle_speed {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
class DrivingLoggerInfo0 : public MessageBase {
public:
  DrivingLoggerInfo0(uint8_t* arr = nullptr): MessageBase(&data[0], 790, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  //* get interface
  inline double engine_rpm() { return static_cast<double>(get(v_can_DrivingLoggerInfo0_engine_rpm)); };
  inline uint8_t vehicle_speed() { return static_cast<uint8_t>(get(v_can_DrivingLoggerInfo0_vehicle_speed)); };
  //* set interface
  inline void engine_rpm(const double& value) { set(static_cast<double>(value), v_can_DrivingLoggerInfo0_engine_rpm); };
  inline void vehicle_speed(const uint8_t& value) { set(static_cast<double>(value), v_can_DrivingLoggerInfo0_vehicle_speed); };
};
#define v_can_UserConfiguration_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_UserConfiguration_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_UserConfiguration_max_speed {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_UserConfiguration_max_steering_torque {ValueType::Unsigned, 0.1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_UserConfiguration_max_acceleration {ValueType::Unsigned, 0.1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_UserConfiguration_max_deceleration {ValueType::Unsigned, -0.1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_UserConfiguration_max_jerk {ValueType::Unsigned, 0.01, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
class UserConfiguration : public MessageBase {
public:
  UserConfiguration(uint8_t* arr = nullptr): MessageBase(&data[0], 256, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_UserConfiguration_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_UserConfiguration_life_count)); };
  inline uint8_t max_speed() { return static_cast<uint8_t>(get(v_can_UserConfiguration_max_speed)); };
  inline double max_steering_torque() { return static_cast<double>(get(v_can_UserConfiguration_max_steering_torque)); };
  inline double max_acceleration() { return static_cast<double>(get(v_can_UserConfiguration_max_acceleration)); };
  inline double max_deceleration() { return static_cast<double>(get(v_can_UserConfiguration_max_deceleration)); };
  inline double max_jerk() { return static_cast<double>(get(v_can_UserConfiguration_max_jerk)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_UserConfiguration_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_UserConfiguration_life_count); };
  inline void max_speed(const uint8_t& value) { set(static_cast<double>(value), v_can_UserConfiguration_max_speed); };
  inline void max_steering_torque(const double& value) { set(static_cast<double>(value), v_can_UserConfiguration_max_steering_torque); };
  inline void max_acceleration(const double& value) { set(static_cast<double>(value), v_can_UserConfiguration_max_acceleration); };
  inline void max_deceleration(const double& value) { set(static_cast<double>(value), v_can_UserConfiguration_max_deceleration); };
  inline void max_jerk(const double& value) { set(static_cast<double>(value), v_can_UserConfiguration_max_jerk); };
};
#define v_can_DoorInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorInfo_door_lock_status {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorInfo_power_door_left_status {ValueType::Unsigned, 1, 0, 18, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorInfo_power_door_right_status {ValueType::Unsigned, 1, 0, 20, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorInfo_trunk_door_status {ValueType::Unsigned, 1, 0, 22, 2, ByteOrder::LittleEndian} // unit: []
class DoorInfo : public MessageBase {
public:
  DoorInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 125, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class DoorLockStatus { kInvalid = 0, kClosed = 1, kOpened = 2 };
  enum class PowerDoorLeftStatus { kInvalid = 0, kClosed = 1, kOpened = 2 };
  enum class PowerDoorRightStatus { kInvalid = 0, kClosed = 1, kOpened = 2 };
  enum class TrunkDoorStatus { kInvalid = 0, kClosed = 1, kOpened = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_DoorInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_DoorInfo_life_count)); };
  inline DoorLockStatus door_lock_status() { return static_cast<DoorLockStatus>(get(v_can_DoorInfo_door_lock_status)); };
  inline PowerDoorLeftStatus power_door_left_status() { return static_cast<PowerDoorLeftStatus>(get(v_can_DoorInfo_power_door_left_status)); };
  inline PowerDoorRightStatus power_door_right_status() { return static_cast<PowerDoorRightStatus>(get(v_can_DoorInfo_power_door_right_status)); };
  inline TrunkDoorStatus trunk_door_status() { return static_cast<TrunkDoorStatus>(get(v_can_DoorInfo_trunk_door_status)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorInfo_life_count); };
  inline void door_lock_status(const DoorLockStatus& value) { set(static_cast<double>(value), v_can_DoorInfo_door_lock_status); };
  inline void power_door_left_status(const PowerDoorLeftStatus& value) { set(static_cast<double>(value), v_can_DoorInfo_power_door_left_status); };
  inline void power_door_right_status(const PowerDoorRightStatus& value) { set(static_cast<double>(value), v_can_DoorInfo_power_door_right_status); };
  inline void trunk_door_status(const TrunkDoorStatus& value) { set(static_cast<double>(value), v_can_DoorInfo_trunk_door_status); };
};
#define v_can_ChargingInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingInfo_charge_state {ValueType::Unsigned, 0.01, 0, 16, 16, ByteOrder::LittleEndian} // unit: [%]
#define v_can_ChargingInfo_charge_door_status {ValueType::Unsigned, 1, 0, 32, 4, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingInfo_connected_status {ValueType::Unsigned, 1, 0, 36, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingInfo_charging_status {ValueType::Unsigned, 1, 0, 38, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingInfo_estimate_time {ValueType::Unsigned, 1, 0, 40, 16, ByteOrder::LittleEndian} // unit: [min]
class ChargingInfo : public MessageBase {
public:
  ChargingInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 119, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  enum class ChargeDoorStatus { kHold = 0, kClose = 1, kOpen = 2 };
  enum class ConnectedStatus { kInvaildOrNa = 0, kDisconnected = 1, kConnected = 2 };
  enum class ChargingStatus { kInvaildOrNa = 0, kUncharging = 1, kCharging = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_ChargingInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_ChargingInfo_life_count)); };
  // unit: [%]
  inline double charge_state() { return static_cast<double>(get(v_can_ChargingInfo_charge_state)); };
  inline ChargeDoorStatus charge_door_status() { return static_cast<ChargeDoorStatus>(get(v_can_ChargingInfo_charge_door_status)); };
  inline ConnectedStatus connected_status() { return static_cast<ConnectedStatus>(get(v_can_ChargingInfo_connected_status)); };
  inline ChargingStatus charging_status() { return static_cast<ChargingStatus>(get(v_can_ChargingInfo_charging_status)); };
  // unit: [min]
  inline uint16_t estimate_time() { return static_cast<uint16_t>(get(v_can_ChargingInfo_estimate_time)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingInfo_life_count); };
  // unit: [%]
  inline void charge_state(const double& value) { set(static_cast<double>(value), v_can_ChargingInfo_charge_state); };
  inline void charge_door_status(const ChargeDoorStatus& value) { set(static_cast<double>(value), v_can_ChargingInfo_charge_door_status); };
  inline void connected_status(const ConnectedStatus& value) { set(static_cast<double>(value), v_can_ChargingInfo_connected_status); };
  inline void charging_status(const ChargingStatus& value) { set(static_cast<double>(value), v_can_ChargingInfo_charging_status); };
  // unit: [min]
  inline void estimate_time(const uint16_t& value) { set(static_cast<double>(value), v_can_ChargingInfo_estimate_time); };
};
#define v_can_IgnitionInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionInfo_vehicle_status {ValueType::Unsigned, 1, 0, 16, 4, ByteOrder::LittleEndian} // unit: []
class IgnitionInfo : public MessageBase {
public:
  IgnitionInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 118, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class VehicleStatus { kDrive = 4, kStart = 3, kAccessory = 2, kOff = 1 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_IgnitionInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_IgnitionInfo_life_count)); };
  inline VehicleStatus vehicle_status() { return static_cast<VehicleStatus>(get(v_can_IgnitionInfo_vehicle_status)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionInfo_life_count); };
  inline void vehicle_status(const VehicleStatus& value) { set(static_cast<double>(value), v_can_IgnitionInfo_vehicle_status); };
};
#define v_can_GearInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_GearInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_GearInfo_gear_status {ValueType::Unsigned, 1, 0, 16, 4, ByteOrder::LittleEndian} // unit: []
class GearInfo : public MessageBase {
public:
  GearInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 117, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class GearStatus { kD = 4, kN = 3, kR = 2, kP = 1, kInvalidOrNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_GearInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_GearInfo_life_count)); };
  inline GearStatus gear_status() { return static_cast<GearStatus>(get(v_can_GearInfo_gear_status)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_GearInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_GearInfo_life_count); };
  inline void gear_status(const GearStatus& value) { set(static_cast<double>(value), v_can_GearInfo_gear_status); };
};
#define v_can_TurnSignalInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalInfo_turn_signal_status {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
class TurnSignalInfo : public MessageBase {
public:
  TurnSignalInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 116, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class TurnSignalStatus { kNa = 0, kLeftBlinking = 1, kRightBlinking = 2, kBothBlinking = 3 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_TurnSignalInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_TurnSignalInfo_life_count)); };
  inline TurnSignalStatus turn_signal_status() { return static_cast<TurnSignalStatus>(get(v_can_TurnSignalInfo_turn_signal_status)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_TurnSignalInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_TurnSignalInfo_life_count); };
  inline void turn_signal_status(const TurnSignalStatus& value) { set(static_cast<double>(value), v_can_TurnSignalInfo_turn_signal_status); };
};
#define v_can_LongitudinalInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalInfo_motor_rpm {ValueType::Unsigned, 0.25, 0, 16, 16, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalInfo_acceleration_pedal_pos {ValueType::Unsigned, 0.5, 0, 32, 8, ByteOrder::LittleEndian} // unit: [%]
#define v_can_LongitudinalInfo_brake_pedal_pos {ValueType::Unsigned, 0.5, 0, 40, 8, ByteOrder::LittleEndian} // unit: [%]
class LongitudinalInfo : public MessageBase {
public:
  LongitudinalInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 115, 6, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[6];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_LongitudinalInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_LongitudinalInfo_life_count)); };
  inline double motor_rpm() { return static_cast<double>(get(v_can_LongitudinalInfo_motor_rpm)); };
  // unit: [%]
  inline double acceleration_pedal_pos() { return static_cast<double>(get(v_can_LongitudinalInfo_acceleration_pedal_pos)); };
  // unit: [%]
  inline double brake_pedal_pos() { return static_cast<double>(get(v_can_LongitudinalInfo_brake_pedal_pos)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalInfo_life_count); };
  inline void motor_rpm(const double& value) { set(static_cast<double>(value), v_can_LongitudinalInfo_motor_rpm); };
  // unit: [%]
  inline void acceleration_pedal_pos(const double& value) { set(static_cast<double>(value), v_can_LongitudinalInfo_acceleration_pedal_pos); };
  // unit: [%]
  inline void brake_pedal_pos(const double& value) { set(static_cast<double>(value), v_can_LongitudinalInfo_brake_pedal_pos); };
};
#define v_can_SteeringInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_SteeringInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_SteeringInfo_steering_angle {ValueType::Signed, 0.1, 0, 16, 14, ByteOrder::LittleEndian} // unit: [deg]
#define v_can_SteeringInfo_steering_torque {ValueType::Signed, 0.01, 0, 30, 10, ByteOrder::LittleEndian} // unit: [Nm]
#define v_can_SteeringInfo_steering_angle_rate {ValueType::Unsigned, 4, 0, 40, 12, ByteOrder::LittleEndian} // unit: [deg/s]
class SteeringInfo : public MessageBase {
public:
  SteeringInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 114, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_SteeringInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_SteeringInfo_life_count)); };
  // unit: [deg]
  inline double steering_angle() { return static_cast<double>(get(v_can_SteeringInfo_steering_angle)); };
  // unit: [Nm]
  inline double steering_torque() { return static_cast<double>(get(v_can_SteeringInfo_steering_torque)); };
  // unit: [deg/s]
  inline uint16_t steering_angle_rate() { return static_cast<uint16_t>(get(v_can_SteeringInfo_steering_angle_rate)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_SteeringInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_SteeringInfo_life_count); };
  // unit: [deg]
  inline void steering_angle(const double& value) { set(static_cast<double>(value), v_can_SteeringInfo_steering_angle); };
  // unit: [Nm]
  inline void steering_torque(const double& value) { set(static_cast<double>(value), v_can_SteeringInfo_steering_torque); };
  // unit: [deg/s]
  inline void steering_angle_rate(const uint16_t& value) { set(static_cast<double>(value), v_can_SteeringInfo_steering_angle_rate); };
};
#define v_can_WheelInfo_crc16 {ValueType::Unsigned, 1, 0, 0, 16, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_life_count {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_dir_fl {ValueType::Unsigned, 1, 0, 24, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_dir_fr {ValueType::Unsigned, 1, 0, 26, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_dir_rl {ValueType::Unsigned, 1, 0, 28, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_dir_rr {ValueType::Unsigned, 1, 0, 30, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_pulse_fl {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_pulse_fr {ValueType::Unsigned, 1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_pulse_rl {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_pulse_rr {ValueType::Unsigned, 1, 0, 56, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_WheelInfo_wheel_speed_fl {ValueType::Unsigned, 0.01, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m/s]
#define v_can_WheelInfo_wheel_speed_fr {ValueType::Unsigned, 0.01, 0, 77, 13, ByteOrder::LittleEndian} // unit: [m/s]
#define v_can_WheelInfo_wheel_speed_rl {ValueType::Unsigned, 0.01, 0, 90, 13, ByteOrder::LittleEndian} // unit: [m/s]
#define v_can_WheelInfo_wheel_speed_rr {ValueType::Unsigned, 0.01, 0, 103, 13, ByteOrder::LittleEndian} // unit: [m/s]
class WheelInfo : public MessageBase {
public:
  WheelInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 113, 16, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[16];
  //* value table
  //* get interface
  inline uint16_t crc16() { return static_cast<uint16_t>(get(v_can_WheelInfo_crc16)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_WheelInfo_life_count)); };
  inline uint8_t wheel_dir_fl() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_dir_fl)); };
  inline uint8_t wheel_dir_fr() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_dir_fr)); };
  inline uint8_t wheel_dir_rl() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_dir_rl)); };
  inline uint8_t wheel_dir_rr() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_dir_rr)); };
  inline uint8_t wheel_pulse_fl() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_pulse_fl)); };
  inline uint8_t wheel_pulse_fr() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_pulse_fr)); };
  inline uint8_t wheel_pulse_rl() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_pulse_rl)); };
  inline uint8_t wheel_pulse_rr() { return static_cast<uint8_t>(get(v_can_WheelInfo_wheel_pulse_rr)); };
  // unit: [m/s]
  inline double wheel_speed_fl() { return static_cast<double>(get(v_can_WheelInfo_wheel_speed_fl)); };
  // unit: [m/s]
  inline double wheel_speed_fr() { return static_cast<double>(get(v_can_WheelInfo_wheel_speed_fr)); };
  // unit: [m/s]
  inline double wheel_speed_rl() { return static_cast<double>(get(v_can_WheelInfo_wheel_speed_rl)); };
  // unit: [m/s]
  inline double wheel_speed_rr() { return static_cast<double>(get(v_can_WheelInfo_wheel_speed_rr)); };
  //* set interface
  inline void crc16(const uint16_t& value) { set(static_cast<double>(value), v_can_WheelInfo_crc16); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_life_count); };
  inline void wheel_dir_fl(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_dir_fl); };
  inline void wheel_dir_fr(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_dir_fr); };
  inline void wheel_dir_rl(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_dir_rl); };
  inline void wheel_dir_rr(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_dir_rr); };
  inline void wheel_pulse_fl(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_pulse_fl); };
  inline void wheel_pulse_fr(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_pulse_fr); };
  inline void wheel_pulse_rl(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_pulse_rl); };
  inline void wheel_pulse_rr(const uint8_t& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_pulse_rr); };
  // unit: [m/s]
  inline void wheel_speed_fl(const double& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_speed_fl); };
  // unit: [m/s]
  inline void wheel_speed_fr(const double& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_speed_fr); };
  // unit: [m/s]
  inline void wheel_speed_rl(const double& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_speed_rl); };
  // unit: [m/s]
  inline void wheel_speed_rr(const double& value) { set(static_cast<double>(value), v_can_WheelInfo_wheel_speed_rr); };
};
#define v_can_DynamicInfo_crc16 {ValueType::Unsigned, 1, 0, 0, 16, ByteOrder::LittleEndian} // unit: []
#define v_can_DynamicInfo_life_count {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DynamicInfo_long_acceleration {ValueType::Signed, 0.01, 0, 24, 12, ByteOrder::LittleEndian} // unit: [m/s^2]
#define v_can_DynamicInfo_lat_acceleration {ValueType::Signed, 0.01, 0, 38, 10, ByteOrder::LittleEndian} // unit: [m/s^2]
#define v_can_DynamicInfo_roll_rate {ValueType::Signed, 0.1, 0, 48, 10, ByteOrder::LittleEndian} // unit: [deg/s]
#define v_can_DynamicInfo_pitch_rate {ValueType::Signed, 0.1, 0, 58, 10, ByteOrder::LittleEndian} // unit: [deg/s]
#define v_can_DynamicInfo_yaw_rate {ValueType::Signed, 0.1, 0, 68, 12, ByteOrder::LittleEndian} // unit: [deg/s]
class DynamicInfo : public MessageBase {
public:
  DynamicInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 112, 12, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[12];
  //* value table
  //* get interface
  inline uint16_t crc16() { return static_cast<uint16_t>(get(v_can_DynamicInfo_crc16)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_DynamicInfo_life_count)); };
  // unit: [m/s^2]
  inline double long_acceleration() { return static_cast<double>(get(v_can_DynamicInfo_long_acceleration)); };
  // unit: [m/s^2]
  inline double lat_acceleration() { return static_cast<double>(get(v_can_DynamicInfo_lat_acceleration)); };
  // unit: [deg/s]
  inline double roll_rate() { return static_cast<double>(get(v_can_DynamicInfo_roll_rate)); };
  // unit: [deg/s]
  inline double pitch_rate() { return static_cast<double>(get(v_can_DynamicInfo_pitch_rate)); };
  // unit: [deg/s]
  inline double yaw_rate() { return static_cast<double>(get(v_can_DynamicInfo_yaw_rate)); };
  //* set interface
  inline void crc16(const uint16_t& value) { set(static_cast<double>(value), v_can_DynamicInfo_crc16); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_DynamicInfo_life_count); };
  // unit: [m/s^2]
  inline void long_acceleration(const double& value) { set(static_cast<double>(value), v_can_DynamicInfo_long_acceleration); };
  // unit: [m/s^2]
  inline void lat_acceleration(const double& value) { set(static_cast<double>(value), v_can_DynamicInfo_lat_acceleration); };
  // unit: [deg/s]
  inline void roll_rate(const double& value) { set(static_cast<double>(value), v_can_DynamicInfo_roll_rate); };
  // unit: [deg/s]
  inline void pitch_rate(const double& value) { set(static_cast<double>(value), v_can_DynamicInfo_pitch_rate); };
  // unit: [deg/s]
  inline void yaw_rate(const double& value) { set(static_cast<double>(value), v_can_DynamicInfo_yaw_rate); };
};
#define v_can_DoorControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorControl_door_control_mode {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorControl_door_lock_cmd {ValueType::Unsigned, 1, 0, 20, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorControl_power_door_left_cmd {ValueType::Unsigned, 1, 0, 22, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorControl_power_door_right_cmd {ValueType::Unsigned, 1, 0, 24, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorControl_trunk_door_cmd {ValueType::Unsigned, 1, 0, 26, 2, ByteOrder::LittleEndian} // unit: []
class DoorControl : public MessageBase {
public:
  DoorControl(uint8_t* arr = nullptr): MessageBase(&data[0], 87, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class DoorControlMode { kNa = 0, kAd = 1 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_DoorControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_DoorControl_life_count)); };
  inline DoorControlMode door_control_mode() { return static_cast<DoorControlMode>(get(v_can_DoorControl_door_control_mode)); };
  inline uint8_t door_lock_cmd() { return static_cast<uint8_t>(get(v_can_DoorControl_door_lock_cmd)); };
  inline uint8_t power_door_left_cmd() { return static_cast<uint8_t>(get(v_can_DoorControl_power_door_left_cmd)); };
  inline uint8_t power_door_right_cmd() { return static_cast<uint8_t>(get(v_can_DoorControl_power_door_right_cmd)); };
  inline uint8_t trunk_door_cmd() { return static_cast<uint8_t>(get(v_can_DoorControl_trunk_door_cmd)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorControl_life_count); };
  inline void door_control_mode(const DoorControlMode& value) { set(static_cast<double>(value), v_can_DoorControl_door_control_mode); };
  inline void door_lock_cmd(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorControl_door_lock_cmd); };
  inline void power_door_left_cmd(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorControl_power_door_left_cmd); };
  inline void power_door_right_cmd(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorControl_power_door_right_cmd); };
  inline void trunk_door_cmd(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorControl_trunk_door_cmd); };
};
#define v_can_ChargingDoorControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingDoorControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingDoorControl_charge_door_ctrl_mode {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingDoorControl_charge_door_cmd {ValueType::Unsigned, 1, 0, 17, 2, ByteOrder::LittleEndian} // unit: []
class ChargingDoorControl : public MessageBase {
public:
  ChargingDoorControl(uint8_t* arr = nullptr): MessageBase(&data[0], 86, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class ChargeDoorCtrlMode { kInactive = 0, kAd = 1 };
  enum class ChargeDoorCmd { kHold = 0, kClose = 1, kOpen = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_ChargingDoorControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_ChargingDoorControl_life_count)); };
  inline ChargeDoorCtrlMode charge_door_ctrl_mode() { return static_cast<ChargeDoorCtrlMode>(get(v_can_ChargingDoorControl_charge_door_ctrl_mode)); };
  inline ChargeDoorCmd charge_door_cmd() { return static_cast<ChargeDoorCmd>(get(v_can_ChargingDoorControl_charge_door_cmd)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingDoorControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingDoorControl_life_count); };
  inline void charge_door_ctrl_mode(const ChargeDoorCtrlMode& value) { set(static_cast<double>(value), v_can_ChargingDoorControl_charge_door_ctrl_mode); };
  inline void charge_door_cmd(const ChargeDoorCmd& value) { set(static_cast<double>(value), v_can_ChargingDoorControl_charge_door_cmd); };
};
#define v_can_IgnitionControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionControl_ign_target {ValueType::Unsigned, 1, 0, 16, 4, ByteOrder::LittleEndian} // unit: []
class IgnitionControl : public MessageBase {
public:
  IgnitionControl(uint8_t* arr = nullptr): MessageBase(&data[0], 85, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class IgnTarget { kDrive = 4, kStart = 3, kAccessory = 2, kOff = 1 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_IgnitionControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_IgnitionControl_life_count)); };
  inline IgnTarget ign_target() { return static_cast<IgnTarget>(get(v_can_IgnitionControl_ign_target)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionControl_life_count); };
  inline void ign_target(const IgnTarget& value) { set(static_cast<double>(value), v_can_IgnitionControl_ign_target); };
};
#define v_can_TurnSignalControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalControl_turn_signal_control_mode {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalControl_target_turn_signal {ValueType::Unsigned, 1, 0, 18, 2, ByteOrder::LittleEndian} // unit: []
class TurnSignalControl : public MessageBase {
public:
  TurnSignalControl(uint8_t* arr = nullptr): MessageBase(&data[0], 84, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class TurnSignalControlMode { kNa = 0, kAd = 1 };
  enum class TargetTurnSignal { kNa = 0, kLeftBlinking = 1, kRightBlinking = 2, kBothBlinking = 3 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_TurnSignalControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_TurnSignalControl_life_count)); };
  inline TurnSignalControlMode turn_signal_control_mode() { return static_cast<TurnSignalControlMode>(get(v_can_TurnSignalControl_turn_signal_control_mode)); };
  inline TargetTurnSignal target_turn_signal() { return static_cast<TargetTurnSignal>(get(v_can_TurnSignalControl_target_turn_signal)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_TurnSignalControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_TurnSignalControl_life_count); };
  inline void turn_signal_control_mode(const TurnSignalControlMode& value) { set(static_cast<double>(value), v_can_TurnSignalControl_turn_signal_control_mode); };
  inline void target_turn_signal(const TargetTurnSignal& value) { set(static_cast<double>(value), v_can_TurnSignalControl_target_turn_signal); };
};
#define v_can_LateralControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralControl_steering_control_mode {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralControl_target_steering_torque {ValueType::Signed, 0.01, 0, 24, 10, ByteOrder::LittleEndian} // unit: [Nm]
#define v_can_LateralControl_target_steering_angle {ValueType::Signed, 0.1, 0, 34, 14, ByteOrder::LittleEndian} // unit: [deg]
#define v_can_LateralControl_steering_torque_limit {ValueType::Unsigned, 0.1, 0, 48, 7, ByteOrder::LittleEndian} // unit: [Nm]
class LateralControl : public MessageBase {
public:
  LateralControl(uint8_t* arr = nullptr): MessageBase(&data[0], 83, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  enum class SteeringControlMode { kAngleControl = 2, kTorqueControl = 1, kInactive = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_LateralControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_LateralControl_life_count)); };
  inline SteeringControlMode steering_control_mode() { return static_cast<SteeringControlMode>(get(v_can_LateralControl_steering_control_mode)); };
  // unit: [Nm]
  inline double target_steering_torque() { return static_cast<double>(get(v_can_LateralControl_target_steering_torque)); };
  // unit: [deg]
  inline double target_steering_angle() { return static_cast<double>(get(v_can_LateralControl_target_steering_angle)); };
  // unit: [Nm]
  inline double steering_torque_limit() { return static_cast<double>(get(v_can_LateralControl_steering_torque_limit)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralControl_life_count); };
  inline void steering_control_mode(const SteeringControlMode& value) { set(static_cast<double>(value), v_can_LateralControl_steering_control_mode); };
  // unit: [Nm]
  inline void target_steering_torque(const double& value) { set(static_cast<double>(value), v_can_LateralControl_target_steering_torque); };
  // unit: [deg]
  inline void target_steering_angle(const double& value) { set(static_cast<double>(value), v_can_LateralControl_target_steering_angle); };
  // unit: [Nm]
  inline void steering_torque_limit(const double& value) { set(static_cast<double>(value), v_can_LateralControl_steering_torque_limit); };
};
#define v_can_GearControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_GearControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_GearControl_gear_control_mode {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_GearControl_target_gear {ValueType::Unsigned, 1, 0, 20, 4, ByteOrder::LittleEndian} // unit: []
class GearControl : public MessageBase {
public:
  GearControl(uint8_t* arr = nullptr): MessageBase(&data[0], 82, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class GearControlMode { kNa = 0, kAd = 1 };
  enum class TargetGear { kD = 4, kN = 3, kR = 2, kP = 1, kInvalidOrNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_GearControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_GearControl_life_count)); };
  inline GearControlMode gear_control_mode() { return static_cast<GearControlMode>(get(v_can_GearControl_gear_control_mode)); };
  inline TargetGear target_gear() { return static_cast<TargetGear>(get(v_can_GearControl_target_gear)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_GearControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_GearControl_life_count); };
  inline void gear_control_mode(const GearControlMode& value) { set(static_cast<double>(value), v_can_GearControl_gear_control_mode); };
  inline void target_gear(const TargetGear& value) { set(static_cast<double>(value), v_can_GearControl_target_gear); };
};
#define v_can_LongitudinalControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalControl_longitudinal_control_mode {ValueType::Unsigned, 1, 0, 16, 3, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalControl_target_acceleration {ValueType::Signed, 0.01, 0, 19, 11, ByteOrder::LittleEndian} // unit: [m/s2]
#define v_can_LongitudinalControl_target_acc_pedal_position {ValueType::Unsigned, 1, 0, 30, 7, ByteOrder::LittleEndian} // unit: [%]
#define v_can_LongitudinalControl_target_brk_pedal_position {ValueType::Unsigned, 1, 0, 37, 7, ByteOrder::LittleEndian} // unit: [%]
#define v_can_LongitudinalControl_target_acc_pedal_torque {ValueType::Unsigned, 1, 0, 44, 7, ByteOrder::LittleEndian} // unit: [%]
#define v_can_LongitudinalControl_target_brk_pedal_torque {ValueType::Unsigned, 1, 0, 51, 7, ByteOrder::LittleEndian} // unit: [%]
class LongitudinalControl : public MessageBase {
public:
  LongitudinalControl(uint8_t* arr = nullptr): MessageBase(&data[0], 81, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  enum class LongitudinalControlMode { kInvalidOrInactive = 0, kAccelerationControl = 1, kPedalPositionControl = 2, kPedalTorqueControl = 3, kAccel_posbrake_torque = 4, kAccel_torquebrake_pos = 5 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_LongitudinalControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_LongitudinalControl_life_count)); };
  inline LongitudinalControlMode longitudinal_control_mode() { return static_cast<LongitudinalControlMode>(get(v_can_LongitudinalControl_longitudinal_control_mode)); };
  // unit: [m/s2]
  inline double target_acceleration() { return static_cast<double>(get(v_can_LongitudinalControl_target_acceleration)); };
  // unit: [%]
  inline uint8_t target_acc_pedal_position() { return static_cast<uint8_t>(get(v_can_LongitudinalControl_target_acc_pedal_position)); };
  // unit: [%]
  inline uint8_t target_brk_pedal_position() { return static_cast<uint8_t>(get(v_can_LongitudinalControl_target_brk_pedal_position)); };
  // unit: [%]
  inline uint8_t target_acc_pedal_torque() { return static_cast<uint8_t>(get(v_can_LongitudinalControl_target_acc_pedal_torque)); };
  // unit: [%]
  inline uint8_t target_brk_pedal_torque() { return static_cast<uint8_t>(get(v_can_LongitudinalControl_target_brk_pedal_torque)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalControl_life_count); };
  inline void longitudinal_control_mode(const LongitudinalControlMode& value) { set(static_cast<double>(value), v_can_LongitudinalControl_longitudinal_control_mode); };
  // unit: [m/s2]
  inline void target_acceleration(const double& value) { set(static_cast<double>(value), v_can_LongitudinalControl_target_acceleration); };
  // unit: [%]
  inline void target_acc_pedal_position(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalControl_target_acc_pedal_position); };
  // unit: [%]
  inline void target_brk_pedal_position(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalControl_target_brk_pedal_position); };
  // unit: [%]
  inline void target_acc_pedal_torque(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalControl_target_acc_pedal_torque); };
  // unit: [%]
  inline void target_brk_pedal_torque(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalControl_target_brk_pedal_torque); };
};
#define v_can_DoorState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorState_door_ctrl_state {ValueType::Unsigned, 1, 0, 16, 4, ByteOrder::LittleEndian} // unit: []
#define v_can_DoorState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class DoorState : public MessageBase {
public:
  DoorState(uint8_t* arr = nullptr): MessageBase(&data[0], 24, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_DoorState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_DoorState_life_count)); };
  inline uint8_t door_ctrl_state() { return static_cast<uint8_t>(get(v_can_DoorState_door_ctrl_state)); };
  inline uint8_t error_code() { return static_cast<uint8_t>(get(v_can_DoorState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorState_life_count); };
  inline void door_ctrl_state(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorState_door_ctrl_state); };
  inline void error_code(const uint8_t& value) { set(static_cast<double>(value), v_can_DoorState_error_code); };
};
#define v_can_IgnitionState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionState_in_progress {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionState_button_push {ValueType::Unsigned, 1, 0, 17, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionState_driver_ctrl {ValueType::Unsigned, 1, 0, 18, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionState_timeout {ValueType::Unsigned, 1, 0, 19, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_IgnitionState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class IgnitionState : public MessageBase {
public:
  IgnitionState(uint8_t* arr = nullptr): MessageBase(&data[0], 23, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class ErrorCode { kInvalidTargetIgnition = 8, kControlCommandTimeout = 4, kInvalidCrc = 2, kInvalidControlMode = 1, kNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_IgnitionState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_IgnitionState_life_count)); };
  inline uint8_t in_progress() { return static_cast<uint8_t>(get(v_can_IgnitionState_in_progress)); };
  inline uint8_t button_push() { return static_cast<uint8_t>(get(v_can_IgnitionState_button_push)); };
  inline uint8_t driver_ctrl() { return static_cast<uint8_t>(get(v_can_IgnitionState_driver_ctrl)); };
  inline uint8_t timeout() { return static_cast<uint8_t>(get(v_can_IgnitionState_timeout)); };
  inline ErrorCode error_code() { return static_cast<ErrorCode>(get(v_can_IgnitionState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionState_life_count); };
  inline void in_progress(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionState_in_progress); };
  inline void button_push(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionState_button_push); };
  inline void driver_ctrl(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionState_driver_ctrl); };
  inline void timeout(const uint8_t& value) { set(static_cast<double>(value), v_can_IgnitionState_timeout); };
  inline void error_code(const ErrorCode& value) { set(static_cast<double>(value), v_can_IgnitionState_error_code); };
};
#define v_can_ChargingState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingState_charge_door_ctrl_state {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingState_charge_door_override {ValueType::Unsigned, 1, 0, 17, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_ChargingState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class ChargingState : public MessageBase {
public:
  ChargingState(uint8_t* arr = nullptr): MessageBase(&data[0], 22, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class ErrorCode { kControlCommandTimeout = 4, kInvalidCrc = 2, kInvalidControlMode = 1, kNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_ChargingState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_ChargingState_life_count)); };
  inline uint8_t charge_door_ctrl_state() { return static_cast<uint8_t>(get(v_can_ChargingState_charge_door_ctrl_state)); };
  inline uint8_t charge_door_override() { return static_cast<uint8_t>(get(v_can_ChargingState_charge_door_override)); };
  inline ErrorCode error_code() { return static_cast<ErrorCode>(get(v_can_ChargingState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingState_life_count); };
  inline void charge_door_ctrl_state(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingState_charge_door_ctrl_state); };
  inline void charge_door_override(const uint8_t& value) { set(static_cast<double>(value), v_can_ChargingState_charge_door_override); };
  inline void error_code(const ErrorCode& value) { set(static_cast<double>(value), v_can_ChargingState_error_code); };
};
#define v_can_GearState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_GearState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_GearState_gear_ctrl_state {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_GearState_gear_override {ValueType::Unsigned, 1, 0, 17, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_GearState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class GearState : public MessageBase {
public:
  GearState(uint8_t* arr = nullptr): MessageBase(&data[0], 21, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class GearCtrlState { kAdControlActivate = 1, kInactive = 0 };
  enum class GearOverride { kInvaildOrNa = 0, kOverried = 1 };
  enum class ErrorCode { kInvalidTargetGear = 8, kControlCommandTimeout = 4, kInvalidCrc = 2, kInvalidControlMode = 1, kNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_GearState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_GearState_life_count)); };
  inline GearCtrlState gear_ctrl_state() { return static_cast<GearCtrlState>(get(v_can_GearState_gear_ctrl_state)); };
  inline GearOverride gear_override() { return static_cast<GearOverride>(get(v_can_GearState_gear_override)); };
  inline ErrorCode error_code() { return static_cast<ErrorCode>(get(v_can_GearState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_GearState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_GearState_life_count); };
  inline void gear_ctrl_state(const GearCtrlState& value) { set(static_cast<double>(value), v_can_GearState_gear_ctrl_state); };
  inline void gear_override(const GearOverride& value) { set(static_cast<double>(value), v_can_GearState_gear_override); };
  inline void error_code(const ErrorCode& value) { set(static_cast<double>(value), v_can_GearState_error_code); };
};
#define v_can_TurnSignalState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalState_turnsignal_ctrl_status {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalState_turnsignal_override {ValueType::Unsigned, 1, 0, 22, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_TurnSignalState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class TurnSignalState : public MessageBase {
public:
  TurnSignalState(uint8_t* arr = nullptr): MessageBase(&data[0], 20, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class TurnsignalCtrlStatus { kNa = 0, kAd = 1 };
  enum class TurnsignalOverride { kInvaildOrNa = 0, kOverrided = 1 };
  enum class ErrorCode { kInvalidTargetTurn_signal = 8, kControlCommandTimeout = 4, kInvalidCrc = 2, kInvalidControlMode = 1, kNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_TurnSignalState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_TurnSignalState_life_count)); };
  inline TurnsignalCtrlStatus turnsignal_ctrl_status() { return static_cast<TurnsignalCtrlStatus>(get(v_can_TurnSignalState_turnsignal_ctrl_status)); };
  inline TurnsignalOverride turnsignal_override() { return static_cast<TurnsignalOverride>(get(v_can_TurnSignalState_turnsignal_override)); };
  inline ErrorCode error_code() { return static_cast<ErrorCode>(get(v_can_TurnSignalState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_TurnSignalState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_TurnSignalState_life_count); };
  inline void turnsignal_ctrl_status(const TurnsignalCtrlStatus& value) { set(static_cast<double>(value), v_can_TurnSignalState_turnsignal_ctrl_status); };
  inline void turnsignal_override(const TurnsignalOverride& value) { set(static_cast<double>(value), v_can_TurnSignalState_turnsignal_override); };
  inline void error_code(const ErrorCode& value) { set(static_cast<double>(value), v_can_TurnSignalState_error_code); };
};
#define v_can_LongitudinalState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalState_longitudinal_ctrl_state {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalState_longitudinal_override {ValueType::Unsigned, 1, 0, 17, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalState_actuator_status {ValueType::Unsigned, 1, 0, 20, 4, ByteOrder::LittleEndian} // unit: []
#define v_can_LongitudinalState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class LongitudinalState : public MessageBase {
public:
  LongitudinalState(uint8_t* arr = nullptr): MessageBase(&data[0], 19, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class LongitudinalCtrlState { kAdControlActivate = 1, kInactive = 0 };
  enum class LongitudinalOverride { kInvalidOrNa = 0, kAccelPedalOverried = 1, kBrakePedalOverride = 2, kBothOverried = 3 };
  enum class ActuatorStatus { kLongitudinalControlActivate = 1, kNa = 0 };
  enum class ErrorCode { kInvalidTargetAngle = 16, kInvalidTargetTorque = 8, kControlCommandTimeout = 4, kInvalidCrc = 2, kInvalidControlMode = 1, kNa = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_LongitudinalState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_LongitudinalState_life_count)); };
  inline LongitudinalCtrlState longitudinal_ctrl_state() { return static_cast<LongitudinalCtrlState>(get(v_can_LongitudinalState_longitudinal_ctrl_state)); };
  inline LongitudinalOverride longitudinal_override() { return static_cast<LongitudinalOverride>(get(v_can_LongitudinalState_longitudinal_override)); };
  inline ActuatorStatus actuator_status() { return static_cast<ActuatorStatus>(get(v_can_LongitudinalState_actuator_status)); };
  inline ErrorCode error_code() { return static_cast<ErrorCode>(get(v_can_LongitudinalState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LongitudinalState_life_count); };
  inline void longitudinal_ctrl_state(const LongitudinalCtrlState& value) { set(static_cast<double>(value), v_can_LongitudinalState_longitudinal_ctrl_state); };
  inline void longitudinal_override(const LongitudinalOverride& value) { set(static_cast<double>(value), v_can_LongitudinalState_longitudinal_override); };
  inline void actuator_status(const ActuatorStatus& value) { set(static_cast<double>(value), v_can_LongitudinalState_actuator_status); };
  inline void error_code(const ErrorCode& value) { set(static_cast<double>(value), v_can_LongitudinalState_error_code); };
};
#define v_can_LateralState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralState_lateral_ctrl_state {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralState_lateral_override {ValueType::Unsigned, 1, 0, 17, 1, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralState_actuator_status {ValueType::Unsigned, 1, 0, 20, 4, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class LateralState : public MessageBase {
public:
  LateralState(uint8_t* arr = nullptr): MessageBase(&data[0], 18, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class LateralCtrlState { kAdControlActivate = 1, kInactive = 0 };
  enum class LateralOverride { kInvaildOrNa = 0, kOverried = 1 };
  enum class ActuatorStatus { kErrorMdps = 13, kControlMdps = 12, kWaitMdps = 11, kReadyMdps = 10, kFreeMdps = 9, kUnknownMdps = 8, kActuatorWarning = 6, kActuatorPositionControl = 5, kActuatorWaitReadOn = 4, kActuatorFreeWheeling = 3, kActuatorReadServoReady = 2, kActuatorWaitServoReady = 1, kInactive = 0 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(v_can_LateralState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_LateralState_life_count)); };
  inline LateralCtrlState lateral_ctrl_state() { return static_cast<LateralCtrlState>(get(v_can_LateralState_lateral_ctrl_state)); };
  inline LateralOverride lateral_override() { return static_cast<LateralOverride>(get(v_can_LateralState_lateral_override)); };
  inline ActuatorStatus actuator_status() { return static_cast<ActuatorStatus>(get(v_can_LateralState_actuator_status)); };
  inline uint8_t error_code() { return static_cast<uint8_t>(get(v_can_LateralState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralState_life_count); };
  inline void lateral_ctrl_state(const LateralCtrlState& value) { set(static_cast<double>(value), v_can_LateralState_lateral_ctrl_state); };
  inline void lateral_override(const LateralOverride& value) { set(static_cast<double>(value), v_can_LateralState_lateral_override); };
  inline void actuator_status(const ActuatorStatus& value) { set(static_cast<double>(value), v_can_LateralState_actuator_status); };
  inline void error_code(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralState_error_code); };
};
#define v_can_MCP251XFDState_interrupt_flag {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCP251XFDState_fifo_name {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCP251XFDState_fifo_status {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
class MCP251XFDState : public MessageBase {
public:
  MCP251XFDState(uint8_t* arr = nullptr): MessageBase(&data[0], 555, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  enum class FifoName { kMcp251xfdNoFifo = 32, kFifo1 = 1 };
  enum class FifoStatus { kMcp251xfdRxFifoNotEmpty = 1, kMcp251xfdRxFifoHalfFull = 2, kMcp251xfdRxFifoFull = 4, kMcp251xfdRxFifoOverflow = 8 };
  //* get interface
  inline uint8_t interrupt_flag() { return static_cast<uint8_t>(get(v_can_MCP251XFDState_interrupt_flag)); };
  inline FifoName fifo_name() { return static_cast<FifoName>(get(v_can_MCP251XFDState_fifo_name)); };
  inline FifoStatus fifo_status() { return static_cast<FifoStatus>(get(v_can_MCP251XFDState_fifo_status)); };
  //* set interface
  inline void interrupt_flag(const uint8_t& value) { set(static_cast<double>(value), v_can_MCP251XFDState_interrupt_flag); };
  inline void fifo_name(const FifoName& value) { set(static_cast<double>(value), v_can_MCP251XFDState_fifo_name); };
  inline void fifo_status(const FifoStatus& value) { set(static_cast<double>(value), v_can_MCP251XFDState_fifo_status); };
};
#define v_can_MCPFIFOState_fifo1_status {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCPFIFOState_fifo2_status {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCPFIFOState_fifo3_status {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCPFIFOState_fifo4_status {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCPFIFOState_fifo5_status {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_MCPFIFOState_fifo6_status {ValueType::Unsigned, 1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
class MCPFIFOState : public MessageBase {
public:
  MCPFIFOState(uint8_t* arr = nullptr): MessageBase(&data[0], 556, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t fifo1_status() { return static_cast<uint8_t>(get(v_can_MCPFIFOState_fifo1_status)); };
  inline uint8_t fifo2_status() { return static_cast<uint8_t>(get(v_can_MCPFIFOState_fifo2_status)); };
  inline uint8_t fifo3_status() { return static_cast<uint8_t>(get(v_can_MCPFIFOState_fifo3_status)); };
  inline uint8_t fifo4_status() { return static_cast<uint8_t>(get(v_can_MCPFIFOState_fifo4_status)); };
  inline uint8_t fifo5_status() { return static_cast<uint8_t>(get(v_can_MCPFIFOState_fifo5_status)); };
  inline uint8_t fifo6_status() { return static_cast<uint8_t>(get(v_can_MCPFIFOState_fifo6_status)); };
  //* set interface
  inline void fifo1_status(const uint8_t& value) { set(static_cast<double>(value), v_can_MCPFIFOState_fifo1_status); };
  inline void fifo2_status(const uint8_t& value) { set(static_cast<double>(value), v_can_MCPFIFOState_fifo2_status); };
  inline void fifo3_status(const uint8_t& value) { set(static_cast<double>(value), v_can_MCPFIFOState_fifo3_status); };
  inline void fifo4_status(const uint8_t& value) { set(static_cast<double>(value), v_can_MCPFIFOState_fifo4_status); };
  inline void fifo5_status(const uint8_t& value) { set(static_cast<double>(value), v_can_MCPFIFOState_fifo5_status); };
  inline void fifo6_status(const uint8_t& value) { set(static_cast<double>(value), v_can_MCPFIFOState_fifo6_status); };
};
#define v_can_LateralDebug_calc_crc {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralDebug_recv_crc {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralDebug_crc_checker_count {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralDebug_crc_status {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_LateralDebug_life_count {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
class LateralDebug : public MessageBase {
public:
  LateralDebug(uint8_t* arr = nullptr): MessageBase(&data[0], 528, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t calc_crc() { return static_cast<uint8_t>(get(v_can_LateralDebug_calc_crc)); };
  inline uint8_t recv_crc() { return static_cast<uint8_t>(get(v_can_LateralDebug_recv_crc)); };
  inline uint8_t crc_checker_count() { return static_cast<uint8_t>(get(v_can_LateralDebug_crc_checker_count)); };
  inline uint8_t crc_status() { return static_cast<uint8_t>(get(v_can_LateralDebug_crc_status)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_LateralDebug_life_count)); };
  //* set interface
  inline void calc_crc(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralDebug_calc_crc); };
  inline void recv_crc(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralDebug_recv_crc); };
  inline void crc_checker_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralDebug_crc_checker_count); };
  inline void crc_status(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralDebug_crc_status); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_LateralDebug_life_count); };
};
#define v_can_ZED_F9R_Frame_crc16 {ValueType::Unsigned, 1, 0, 0, 16, ByteOrder::LittleEndian} // unit: []
#define v_can_ZED_F9R_Frame_life_count {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define v_can_ZED_F9R_Frame_data_valid {ValueType::Unsigned, 1, 0, 24, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_ZED_F9R_Frame_NS {ValueType::Unsigned, 1, 0, 26, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_ZED_F9R_Frame_WE {ValueType::Unsigned, 1, 0, 28, 2, ByteOrder::LittleEndian} // unit: []
#define v_can_ZED_F9R_Frame_time {ValueType::Unsigned, 1, 0, 32, 16, ByteOrder::LittleEndian} // unit: [hhmmss]
#define v_can_ZED_F9R_Frame_date {ValueType::Unsigned, 1, 0, 48, 16, ByteOrder::LittleEndian} // unit: [yymmdd]
#define v_can_ZED_F9R_Frame_latitude {ValueType::Float, 1, 0, 64, 32, ByteOrder::LittleEndian} // unit: [deg]
#define v_can_ZED_F9R_Frame_longitude {ValueType::Float, 1, 0, 96, 32, ByteOrder::LittleEndian} // unit: [deg]
class ZED_F9R_Frame : public MessageBase {
public:
  ZED_F9R_Frame(uint8_t* arr = nullptr): MessageBase(&data[0], 529, 16, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[16];
  //* value table
  //* get interface
  inline uint16_t crc16() { return static_cast<uint16_t>(get(v_can_ZED_F9R_Frame_crc16)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(v_can_ZED_F9R_Frame_life_count)); };
  inline uint8_t data_valid() { return static_cast<uint8_t>(get(v_can_ZED_F9R_Frame_data_valid)); };
  inline uint8_t NS() { return static_cast<uint8_t>(get(v_can_ZED_F9R_Frame_NS)); };
  inline uint8_t WE() { return static_cast<uint8_t>(get(v_can_ZED_F9R_Frame_WE)); };
  // unit: [hhmmss]
  inline uint16_t time() { return static_cast<uint16_t>(get(v_can_ZED_F9R_Frame_time)); };
  // unit: [yymmdd]
  inline uint16_t date() { return static_cast<uint16_t>(get(v_can_ZED_F9R_Frame_date)); };
  // unit: [deg]
  inline float latitude() { return static_cast<float>(get(v_can_ZED_F9R_Frame_latitude)); };
  // unit: [deg]
  inline float longitude() { return static_cast<float>(get(v_can_ZED_F9R_Frame_longitude)); };
  //* set interface
  inline void crc16(const uint16_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_crc16); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_life_count); };
  inline void data_valid(const uint8_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_data_valid); };
  inline void NS(const uint8_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_NS); };
  inline void WE(const uint8_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_WE); };
  // unit: [hhmmss]
  inline void time(const uint16_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_time); };
  // unit: [yymmdd]
  inline void date(const uint16_t& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_date); };
  // unit: [deg]
  inline void latitude(const float& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_latitude); };
  // unit: [deg]
  inline void longitude(const float& value) { set(static_cast<double>(value), v_can_ZED_F9R_Frame_longitude); };
};

} // namespace v_can
} // namespace dbc
#endif // V_CAN_DBC_HPP__
