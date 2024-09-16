#ifndef AD_CAN_DBC_HPP__
#define AD_CAN_DBC_HPP__

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

namespace ad_can {
enum class Id { 
  AutonomousState = 16,
  OperationControl = 17,
  BrainState = 33,
  RemoteControlStatus = 34,
  WaypointList = 35,
  WaypointPosition = 36,
  WaypointExtended01 = 37,
  WaypointExtended02 = 38,
  WaypointExtended03 = 39,
  LEDIndicator = 41,
  MissionInfo = 40,
  LongitudinalControl = 81,
  GearControl = 82,
  LateralControl = 83,
  TurnSignalControl = 84,
  GPSLocation = 120,
  GPSLocationExt = 121,
  TrafficLightInfo = 122,
  TrafficSignInfo = 123,
  SystemInfo = 124,
  LoggerInfo = 125,
  ObjectList = 144,
  ObjectInfo = 145,
  ObjectExt = 146,
  UserConfiguration = 256,
  DrivingLoggerInfo0 = 790,
  DrivingLoggerInfo1 = 809,
  DoorControl = 87,
};

#define ad_can_AutonomousState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_AutonomousState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_AutonomousState_operation_mode {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define ad_can_AutonomousState_autonomous_mode {ValueType::Unsigned, 1, 0, 17, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_AutonomousState_error_code {ValueType::Unsigned, 1, 0, 24, 16, ByteOrder::LittleEndian} // unit: []
#define ad_can_AutonomousState_warning_code {ValueType::Unsigned, 1, 0, 40, 16, ByteOrder::LittleEndian} // unit: []
class AutonomousState : public MessageBase {
public:
  AutonomousState(uint8_t* arr = nullptr): MessageBase(&data[0], 16, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  enum class OperationMode { kNa = 0, kSystemRun = 1 };
  enum class AutonomousMode { kNa = 0, kAutonomousMode = 1, kError = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_AutonomousState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_AutonomousState_life_count)); };
  inline OperationMode operation_mode() { return static_cast<OperationMode>(get(ad_can_AutonomousState_operation_mode)); };
  inline AutonomousMode autonomous_mode() { return static_cast<AutonomousMode>(get(ad_can_AutonomousState_autonomous_mode)); };
  inline uint16_t error_code() { return static_cast<uint16_t>(get(ad_can_AutonomousState_error_code)); };
  inline uint16_t warning_code() { return static_cast<uint16_t>(get(ad_can_AutonomousState_warning_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_AutonomousState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_AutonomousState_life_count); };
  inline void operation_mode(const OperationMode& value) { set(static_cast<double>(value), ad_can_AutonomousState_operation_mode); };
  inline void autonomous_mode(const AutonomousMode& value) { set(static_cast<double>(value), ad_can_AutonomousState_autonomous_mode); };
  inline void error_code(const uint16_t& value) { set(static_cast<double>(value), ad_can_AutonomousState_error_code); };
  inline void warning_code(const uint16_t& value) { set(static_cast<double>(value), ad_can_AutonomousState_warning_code); };
};
#define ad_can_OperationControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_OperationControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_OperationControl_operation_sw {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define ad_can_OperationControl_autonomous_sw {ValueType::Unsigned, 1, 0, 17, 1, ByteOrder::LittleEndian} // unit: []
#define ad_can_OperationControl_emergency_sw {ValueType::Unsigned, 1, 0, 18, 1, ByteOrder::LittleEndian} // unit: []
class OperationControl : public MessageBase {
public:
  OperationControl(uint8_t* arr = nullptr): MessageBase(&data[0], 17, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_OperationControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_OperationControl_life_count)); };
  inline uint8_t operation_sw() { return static_cast<uint8_t>(get(ad_can_OperationControl_operation_sw)); };
  inline uint8_t autonomous_sw() { return static_cast<uint8_t>(get(ad_can_OperationControl_autonomous_sw)); };
  inline uint8_t emergency_sw() { return static_cast<uint8_t>(get(ad_can_OperationControl_emergency_sw)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_OperationControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_OperationControl_life_count); };
  inline void operation_sw(const uint8_t& value) { set(static_cast<double>(value), ad_can_OperationControl_operation_sw); };
  inline void autonomous_sw(const uint8_t& value) { set(static_cast<double>(value), ad_can_OperationControl_autonomous_sw); };
  inline void emergency_sw(const uint8_t& value) { set(static_cast<double>(value), ad_can_OperationControl_emergency_sw); };
};
#define ad_can_BrainState_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_BrainState_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_BrainState_brain_status {ValueType::Unsigned, 1, 0, 16, 3, ByteOrder::LittleEndian} // unit: []
#define ad_can_BrainState_debug_mode {ValueType::Unsigned, 1, 0, 20, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_BrainState_error_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
class BrainState : public MessageBase {
public:
  BrainState(uint8_t* arr = nullptr): MessageBase(&data[0], 33, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class BrainStatus { kNa = 0, kOk = 1, kWarning = 2, kError = 3 };
  enum class DebugMode { kNa = 0, kLateralOnly = 1, kLongitudinalOnly = 2, kAccesorryOnly = 3 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_BrainState_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_BrainState_life_count)); };
  inline BrainStatus brain_status() { return static_cast<BrainStatus>(get(ad_can_BrainState_brain_status)); };
  inline DebugMode debug_mode() { return static_cast<DebugMode>(get(ad_can_BrainState_debug_mode)); };
  inline uint8_t error_code() { return static_cast<uint8_t>(get(ad_can_BrainState_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_BrainState_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_BrainState_life_count); };
  inline void brain_status(const BrainStatus& value) { set(static_cast<double>(value), ad_can_BrainState_brain_status); };
  inline void debug_mode(const DebugMode& value) { set(static_cast<double>(value), ad_can_BrainState_debug_mode); };
  inline void error_code(const uint8_t& value) { set(static_cast<double>(value), ad_can_BrainState_error_code); };
};
#define ad_can_RemoteControlStatus_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_RemoteControlStatus_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_RemoteControlStatus_remote_status {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_RemoteControlStatus_remote_type {ValueType::Unsigned, 1, 0, 18, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_RemoteControlStatus_connection_error {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_RemoteControlStatus_camera_error {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_RemoteControlStatus_error_code {ValueType::Unsigned, 1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
class RemoteControlStatus : public MessageBase {
public:
  RemoteControlStatus(uint8_t* arr = nullptr): MessageBase(&data[0], 34, 6, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[6];
  //* value table
  enum class RemoteStatus { kInvalid = 0, kA1 = 1, kRemote = 2 };
  enum class RemoteType { kInvalid = 0, kDirect = 1, kIndirect1 = 2, kIndirect2 = 3, kIndirect3 = 4, kRoutine = 5 };
  enum class ConnectionError { kNa = 0, kFrontConnection = 1, kLeftConnection = 2, kRightConnection = 4, kMotorConnection = 8, kRrt = 16 };
  enum class CameraError { kNa = 0, kFrontFrame = 1, kLeftFrame = 2, kRightFrame = 4 };
  enum class ErrorCode { kNa = 0, kAvsRequest = 1, kRemoteResponse = 2, kRemoteRequest = 4, kAvsResponse = 8, kPlanningOnoff = 16, kControllerOnoff = 32 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_RemoteControlStatus_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_RemoteControlStatus_life_count)); };
  inline RemoteStatus remote_status() { return static_cast<RemoteStatus>(get(ad_can_RemoteControlStatus_remote_status)); };
  inline RemoteType remote_type() { return static_cast<RemoteType>(get(ad_can_RemoteControlStatus_remote_type)); };
  inline ConnectionError connection_error() { return static_cast<ConnectionError>(get(ad_can_RemoteControlStatus_connection_error)); };
  inline CameraError camera_error() { return static_cast<CameraError>(get(ad_can_RemoteControlStatus_camera_error)); };
  inline ErrorCode error_code() { return static_cast<ErrorCode>(get(ad_can_RemoteControlStatus_error_code)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_life_count); };
  inline void remote_status(const RemoteStatus& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_remote_status); };
  inline void remote_type(const RemoteType& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_remote_type); };
  inline void connection_error(const ConnectionError& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_connection_error); };
  inline void camera_error(const CameraError& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_camera_error); };
  inline void error_code(const ErrorCode& value) { set(static_cast<double>(value), ad_can_RemoteControlStatus_error_code); };
};
#define ad_can_WaypointList_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointList_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointList_frame_id {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointList_number_of_waypoints {ValueType::Unsigned, 1, 0, 18, 6, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointList_type {ValueType::Unsigned, 1, 0, 24, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointList_sender {ValueType::Unsigned, 1, 0, 26, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointList_dt {ValueType::Unsigned, 0.005, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
class WaypointList : public MessageBase {
public:
  WaypointList(uint8_t* arr = nullptr): MessageBase(&data[0], 35, 5, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[5];
  //* value table
  enum class Type { kUnknown = 0, kWaypoint = 1, kTragectory = 2 };
  enum class Sender { kUnknown = 0, kA1 = 1, kExternal = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_WaypointList_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_WaypointList_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_WaypointList_frame_id)); };
  inline uint8_t number_of_waypoints() { return static_cast<uint8_t>(get(ad_can_WaypointList_number_of_waypoints)); };
  inline Type type() { return static_cast<Type>(get(ad_can_WaypointList_type)); };
  inline Sender sender() { return static_cast<Sender>(get(ad_can_WaypointList_sender)); };
  inline double dt() { return static_cast<double>(get(ad_can_WaypointList_dt)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointList_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointList_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointList_frame_id); };
  inline void number_of_waypoints(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointList_number_of_waypoints); };
  inline void type(const Type& value) { set(static_cast<double>(value), ad_can_WaypointList_type); };
  inline void sender(const Sender& value) { set(static_cast<double>(value), ad_can_WaypointList_sender); };
  inline void dt(const double& value) { set(static_cast<double>(value), ad_can_WaypointList_dt); };
};
#define ad_can_WaypointPosition_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointPosition_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointPosition_frame_id {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointPosition_index {ValueType::Unsigned, 1, 0, 14, 6, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointPosition_latitude_int {ValueType::Unsigned, 1, 0, 20, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointPosition_longitude_int {ValueType::Unsigned, 1, 0, 28, 10, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointPosition_heading {ValueType::Unsigned, 0.1, -180, 38, 12, ByteOrder::LittleEndian} // unit: [deg]
class WaypointPosition : public MessageBase {
public:
  WaypointPosition(uint8_t* arr = nullptr): MessageBase(&data[0], 36, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_WaypointPosition_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_WaypointPosition_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_WaypointPosition_frame_id)); };
  inline uint8_t index() { return static_cast<uint8_t>(get(ad_can_WaypointPosition_index)); };
  inline uint8_t latitude_int() { return static_cast<uint8_t>(get(ad_can_WaypointPosition_latitude_int)); };
  inline uint16_t longitude_int() { return static_cast<uint16_t>(get(ad_can_WaypointPosition_longitude_int)); };
  // unit: [deg]
  inline double heading() { return static_cast<double>(get(ad_can_WaypointPosition_heading)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointPosition_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointPosition_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointPosition_frame_id); };
  inline void index(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointPosition_index); };
  inline void latitude_int(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointPosition_latitude_int); };
  inline void longitude_int(const uint16_t& value) { set(static_cast<double>(value), ad_can_WaypointPosition_longitude_int); };
  // unit: [deg]
  inline void heading(const double& value) { set(static_cast<double>(value), ad_can_WaypointPosition_heading); };
};
#define ad_can_WaypointExtended01_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended01_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended01_frame_id {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended01_index {ValueType::Unsigned, 1, 0, 14, 6, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended01_latitude_decimal {ValueType::Unsigned, 1e-06, 0, 20, 20, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended01_longitude_decimal {ValueType::Unsigned, 1e-06, 0, 40, 20, ByteOrder::LittleEndian} // unit: []
class WaypointExtended01 : public MessageBase {
public:
  WaypointExtended01(uint8_t* arr = nullptr): MessageBase(&data[0], 37, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_WaypointExtended01_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_WaypointExtended01_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_WaypointExtended01_frame_id)); };
  inline uint8_t index() { return static_cast<uint8_t>(get(ad_can_WaypointExtended01_index)); };
  inline double latitude_decimal() { return static_cast<double>(get(ad_can_WaypointExtended01_latitude_decimal)); };
  inline double longitude_decimal() { return static_cast<double>(get(ad_can_WaypointExtended01_longitude_decimal)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended01_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended01_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended01_frame_id); };
  inline void index(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended01_index); };
  inline void latitude_decimal(const double& value) { set(static_cast<double>(value), ad_can_WaypointExtended01_latitude_decimal); };
  inline void longitude_decimal(const double& value) { set(static_cast<double>(value), ad_can_WaypointExtended01_longitude_decimal); };
};
#define ad_can_WaypointExtended02_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended02_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended02_frame_id {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended02_index {ValueType::Unsigned, 1, 0, 16, 6, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended02_height {ValueType::Float, 1, 0, 24, 32, ByteOrder::LittleEndian} // unit: []
class WaypointExtended02 : public MessageBase {
public:
  WaypointExtended02(uint8_t* arr = nullptr): MessageBase(&data[0], 38, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_WaypointExtended02_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_WaypointExtended02_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_WaypointExtended02_frame_id)); };
  inline uint8_t index() { return static_cast<uint8_t>(get(ad_can_WaypointExtended02_index)); };
  inline float height() { return static_cast<float>(get(ad_can_WaypointExtended02_height)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended02_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended02_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended02_frame_id); };
  inline void index(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended02_index); };
  inline void height(const float& value) { set(static_cast<double>(value), ad_can_WaypointExtended02_height); };
};
#define ad_can_WaypointExtended03_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended03_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended03_frame_id {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended03_index {ValueType::Unsigned, 1, 0, 14, 6, ByteOrder::LittleEndian} // unit: []
#define ad_can_WaypointExtended03_speed {ValueType::Unsigned, 0.1, 0, 20, 12, ByteOrder::LittleEndian} // unit: [m/s]
class WaypointExtended03 : public MessageBase {
public:
  WaypointExtended03(uint8_t* arr = nullptr): MessageBase(&data[0], 39, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_WaypointExtended03_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_WaypointExtended03_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_WaypointExtended03_frame_id)); };
  inline uint8_t index() { return static_cast<uint8_t>(get(ad_can_WaypointExtended03_index)); };
  // unit: [m/s]
  inline double speed() { return static_cast<double>(get(ad_can_WaypointExtended03_speed)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended03_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended03_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended03_frame_id); };
  inline void index(const uint8_t& value) { set(static_cast<double>(value), ad_can_WaypointExtended03_index); };
  // unit: [m/s]
  inline void speed(const double& value) { set(static_cast<double>(value), ad_can_WaypointExtended03_speed); };
};
#define ad_can_LEDIndicator_LED0 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED1 {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED2 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED3 {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED4 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED5 {ValueType::Unsigned, 1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED6 {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LEDIndicator_LED7 {ValueType::Unsigned, 1, 0, 56, 8, ByteOrder::LittleEndian} // unit: []
class LEDIndicator : public MessageBase {
public:
  LEDIndicator(uint8_t* arr = nullptr): MessageBase(&data[0], 41, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t LED0() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED0)); };
  inline uint8_t LED1() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED1)); };
  inline uint8_t LED2() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED2)); };
  inline uint8_t LED3() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED3)); };
  inline uint8_t LED4() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED4)); };
  inline uint8_t LED5() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED5)); };
  inline uint8_t LED6() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED6)); };
  inline uint8_t LED7() { return static_cast<uint8_t>(get(ad_can_LEDIndicator_LED7)); };
  //* set interface
  inline void LED0(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED0); };
  inline void LED1(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED1); };
  inline void LED2(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED2); };
  inline void LED3(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED3); };
  inline void LED4(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED4); };
  inline void LED5(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED5); };
  inline void LED6(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED6); };
  inline void LED7(const uint8_t& value) { set(static_cast<double>(value), ad_can_LEDIndicator_LED7); };
};
#define ad_can_MissionInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_MissionInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_MissionInfo_mission {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
class MissionInfo : public MessageBase {
public:
  MissionInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 40, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class Mission { kUnknown = 0, kStart = 1, kWaitrequest = 2, kDrive = 3, kAborted = 4, kComplete = 5, kLanekeeping = 6, kLanechange = 7 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_MissionInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_MissionInfo_life_count)); };
  inline Mission mission() { return static_cast<Mission>(get(ad_can_MissionInfo_mission)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_MissionInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_MissionInfo_life_count); };
  inline void mission(const Mission& value) { set(static_cast<double>(value), ad_can_MissionInfo_mission); };
};
#define ad_can_LongitudinalControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LongitudinalControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LongitudinalControl_longitudinal_control_mode {ValueType::Unsigned, 1, 0, 16, 3, ByteOrder::LittleEndian} // unit: []
#define ad_can_LongitudinalControl_target_acceleration {ValueType::Signed, 0.01, 0, 19, 11, ByteOrder::LittleEndian} // unit: [m/s^2]
#define ad_can_LongitudinalControl_target_acc_pedal_position {ValueType::Unsigned, 1, 0, 30, 7, ByteOrder::LittleEndian} // unit: [%]
#define ad_can_LongitudinalControl_target_brk_pedal_position {ValueType::Unsigned, 1, 0, 37, 7, ByteOrder::LittleEndian} // unit: [%]
#define ad_can_LongitudinalControl_target_acc_pedal_torque {ValueType::Unsigned, 1, 0, 44, 7, ByteOrder::LittleEndian} // unit: []
#define ad_can_LongitudinalControl_target_brk_pedal_torque {ValueType::Unsigned, 1, 0, 51, 7, ByteOrder::LittleEndian} // unit: []
class LongitudinalControl : public MessageBase {
public:
  LongitudinalControl(uint8_t* arr = nullptr): MessageBase(&data[0], 81, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  enum class LongitudinalControlMode { kInactive = 0, kAccelerationControl = 1, kPedalPosControl = 2, kPedalTorqueControl = 3, kAccelPosbrakeTorque = 4, kAccelTorquebrakePos = 5 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_LongitudinalControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_LongitudinalControl_life_count)); };
  inline LongitudinalControlMode longitudinal_control_mode() { return static_cast<LongitudinalControlMode>(get(ad_can_LongitudinalControl_longitudinal_control_mode)); };
  // unit: [m/s^2]
  inline double target_acceleration() { return static_cast<double>(get(ad_can_LongitudinalControl_target_acceleration)); };
  // unit: [%]
  inline uint8_t target_acc_pedal_position() { return static_cast<uint8_t>(get(ad_can_LongitudinalControl_target_acc_pedal_position)); };
  // unit: [%]
  inline uint8_t target_brk_pedal_position() { return static_cast<uint8_t>(get(ad_can_LongitudinalControl_target_brk_pedal_position)); };
  inline uint8_t target_acc_pedal_torque() { return static_cast<uint8_t>(get(ad_can_LongitudinalControl_target_acc_pedal_torque)); };
  inline uint8_t target_brk_pedal_torque() { return static_cast<uint8_t>(get(ad_can_LongitudinalControl_target_brk_pedal_torque)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_life_count); };
  inline void longitudinal_control_mode(const LongitudinalControlMode& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_longitudinal_control_mode); };
  // unit: [m/s^2]
  inline void target_acceleration(const double& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_target_acceleration); };
  // unit: [%]
  inline void target_acc_pedal_position(const uint8_t& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_target_acc_pedal_position); };
  // unit: [%]
  inline void target_brk_pedal_position(const uint8_t& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_target_brk_pedal_position); };
  inline void target_acc_pedal_torque(const uint8_t& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_target_acc_pedal_torque); };
  inline void target_brk_pedal_torque(const uint8_t& value) { set(static_cast<double>(value), ad_can_LongitudinalControl_target_brk_pedal_torque); };
};
#define ad_can_GearControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_GearControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_GearControl_gear_control_mode {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_GearControl_target_gear {ValueType::Unsigned, 1, 0, 20, 4, ByteOrder::LittleEndian} // unit: []
class GearControl : public MessageBase {
public:
  GearControl(uint8_t* arr = nullptr): MessageBase(&data[0], 82, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class GearControlMode { kInactive = 0, kAdControl = 1 };
  enum class TargetGear { kP = 1, kR = 2, kN = 3, kD = 4 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_GearControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_GearControl_life_count)); };
  inline GearControlMode gear_control_mode() { return static_cast<GearControlMode>(get(ad_can_GearControl_gear_control_mode)); };
  inline TargetGear target_gear() { return static_cast<TargetGear>(get(ad_can_GearControl_target_gear)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_GearControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_GearControl_life_count); };
  inline void gear_control_mode(const GearControlMode& value) { set(static_cast<double>(value), ad_can_GearControl_gear_control_mode); };
  inline void target_gear(const TargetGear& value) { set(static_cast<double>(value), ad_can_GearControl_target_gear); };
};
#define ad_can_LateralControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LateralControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LateralControl_steering_control_mode {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_LateralControl_target_steering_torque {ValueType::Signed, 0.01, 0, 24, 10, ByteOrder::LittleEndian} // unit: [Nm]
#define ad_can_LateralControl_target_steering_angle {ValueType::Signed, 0.1, 0, 34, 14, ByteOrder::LittleEndian} // unit: [deg]
#define ad_can_LateralControl_steering_torque_limit {ValueType::Unsigned, 0.1, 0, 48, 7, ByteOrder::LittleEndian} // unit: [Nm]
class LateralControl : public MessageBase {
public:
  LateralControl(uint8_t* arr = nullptr): MessageBase(&data[0], 83, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  enum class SteeringControlMode { kInactive = 0, kTorqueControl = 1, kAngleControl = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_LateralControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_LateralControl_life_count)); };
  inline SteeringControlMode steering_control_mode() { return static_cast<SteeringControlMode>(get(ad_can_LateralControl_steering_control_mode)); };
  // unit: [Nm]
  inline double target_steering_torque() { return static_cast<double>(get(ad_can_LateralControl_target_steering_torque)); };
  // unit: [deg]
  inline double target_steering_angle() { return static_cast<double>(get(ad_can_LateralControl_target_steering_angle)); };
  // unit: [Nm]
  inline double steering_torque_limit() { return static_cast<double>(get(ad_can_LateralControl_steering_torque_limit)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_LateralControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_LateralControl_life_count); };
  inline void steering_control_mode(const SteeringControlMode& value) { set(static_cast<double>(value), ad_can_LateralControl_steering_control_mode); };
  // unit: [Nm]
  inline void target_steering_torque(const double& value) { set(static_cast<double>(value), ad_can_LateralControl_target_steering_torque); };
  // unit: [deg]
  inline void target_steering_angle(const double& value) { set(static_cast<double>(value), ad_can_LateralControl_target_steering_angle); };
  // unit: [Nm]
  inline void steering_torque_limit(const double& value) { set(static_cast<double>(value), ad_can_LateralControl_steering_torque_limit); };
};
#define ad_can_TurnSignalControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_TurnSignalControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_TurnSignalControl_turn_signal_control_mode {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_TurnSignalControl_target_turn_signal {ValueType::Unsigned, 1, 0, 18, 2, ByteOrder::LittleEndian} // unit: []
class TurnSignalControl : public MessageBase {
public:
  TurnSignalControl(uint8_t* arr = nullptr): MessageBase(&data[0], 84, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class TurnSignalControlMode { kInactive = 0, kAdControl = 1 };
  enum class TargetTurnSignal { kOff = 0, kLeftBlinking = 1, kRightBlinking = 2, kBothBlinking = 3 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_TurnSignalControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_TurnSignalControl_life_count)); };
  inline TurnSignalControlMode turn_signal_control_mode() { return static_cast<TurnSignalControlMode>(get(ad_can_TurnSignalControl_turn_signal_control_mode)); };
  inline TargetTurnSignal target_turn_signal() { return static_cast<TargetTurnSignal>(get(ad_can_TurnSignalControl_target_turn_signal)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_TurnSignalControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_TurnSignalControl_life_count); };
  inline void turn_signal_control_mode(const TurnSignalControlMode& value) { set(static_cast<double>(value), ad_can_TurnSignalControl_turn_signal_control_mode); };
  inline void target_turn_signal(const TargetTurnSignal& value) { set(static_cast<double>(value), ad_can_TurnSignalControl_target_turn_signal); };
};
#define ad_can_GPSLocation_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_GPSLocation_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_GPSLocation_latitude {ValueType::Unsigned, 5.36442e-06, -90, 12, 25, ByteOrder::LittleEndian} // unit: [deg]
#define ad_can_GPSLocation_longitude {ValueType::Unsigned, 2.68221e-06, -180, 37, 27, ByteOrder::LittleEndian} // unit: [deg]
class GPSLocation : public MessageBase {
public:
  GPSLocation(uint8_t* arr = nullptr): MessageBase(&data[0], 120, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_GPSLocation_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_GPSLocation_life_count)); };
  // unit: [deg]
  inline double latitude() { return static_cast<double>(get(ad_can_GPSLocation_latitude)); };
  // unit: [deg]
  inline double longitude() { return static_cast<double>(get(ad_can_GPSLocation_longitude)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_GPSLocation_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_GPSLocation_life_count); };
  // unit: [deg]
  inline void latitude(const double& value) { set(static_cast<double>(value), ad_can_GPSLocation_latitude); };
  // unit: [deg]
  inline void longitude(const double& value) { set(static_cast<double>(value), ad_can_GPSLocation_longitude); };
};
#define ad_can_GPSLocationExt_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_GPSLocationExt_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_GPSLocationExt_heading {ValueType::Unsigned, 0.1, -180, 16, 12, ByteOrder::LittleEndian} // unit: [deg]
#define ad_can_GPSLocationExt_height {ValueType::Float, 1, 0, 32, 32, ByteOrder::LittleEndian} // unit: []
class GPSLocationExt : public MessageBase {
public:
  GPSLocationExt(uint8_t* arr = nullptr): MessageBase(&data[0], 121, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_GPSLocationExt_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_GPSLocationExt_life_count)); };
  // unit: [deg]
  inline double heading() { return static_cast<double>(get(ad_can_GPSLocationExt_heading)); };
  inline float height() { return static_cast<float>(get(ad_can_GPSLocationExt_height)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_GPSLocationExt_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_GPSLocationExt_life_count); };
  // unit: [deg]
  inline void heading(const double& value) { set(static_cast<double>(value), ad_can_GPSLocationExt_heading); };
  inline void height(const float& value) { set(static_cast<double>(value), ad_can_GPSLocationExt_height); };
};
#define ad_can_TrafficLightInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficLightInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficLightInfo_traffic_light {ValueType::Unsigned, 1, 0, 16, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficLightInfo_probability {ValueType::Unsigned, 0.5, 0, 24, 8, ByteOrder::LittleEndian} // unit: [%]
class TrafficLightInfo : public MessageBase {
public:
  TrafficLightInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 122, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class TrafficLight { kNa = 0, kRed = 1, kYellow = 2, kGreen = 3, kLeft = 4, kUturn = 5, kRight = 6, kGreenAll = 7 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_TrafficLightInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_TrafficLightInfo_life_count)); };
  inline TrafficLight traffic_light() { return static_cast<TrafficLight>(get(ad_can_TrafficLightInfo_traffic_light)); };
  // unit: [%]
  inline double probability() { return static_cast<double>(get(ad_can_TrafficLightInfo_probability)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficLightInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficLightInfo_life_count); };
  inline void traffic_light(const TrafficLight& value) { set(static_cast<double>(value), ad_can_TrafficLightInfo_traffic_light); };
  // unit: [%]
  inline void probability(const double& value) { set(static_cast<double>(value), ad_can_TrafficLightInfo_probability); };
};
#define ad_can_TrafficSignInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficSignInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficSignInfo_speed_limit {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: [km/h]
#define ad_can_TrafficSignInfo_school_zone {ValueType::Unsigned, 1, 0, 24, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficSignInfo_tollgate_lane {ValueType::Unsigned, 1, 0, 26, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_TrafficSignInfo_conn_detection {ValueType::Unsigned, 1, 0, 28, 2, ByteOrder::LittleEndian} // unit: []
class TrafficSignInfo : public MessageBase {
public:
  TrafficSignInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 123, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_TrafficSignInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_TrafficSignInfo_life_count)); };
  // unit: [km/h]
  inline uint8_t speed_limit() { return static_cast<uint8_t>(get(ad_can_TrafficSignInfo_speed_limit)); };
  inline uint8_t school_zone() { return static_cast<uint8_t>(get(ad_can_TrafficSignInfo_school_zone)); };
  inline uint8_t tollgate_lane() { return static_cast<uint8_t>(get(ad_can_TrafficSignInfo_tollgate_lane)); };
  inline uint8_t conn_detection() { return static_cast<uint8_t>(get(ad_can_TrafficSignInfo_conn_detection)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficSignInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficSignInfo_life_count); };
  // unit: [km/h]
  inline void speed_limit(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficSignInfo_speed_limit); };
  inline void school_zone(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficSignInfo_school_zone); };
  inline void tollgate_lane(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficSignInfo_tollgate_lane); };
  inline void conn_detection(const uint8_t& value) { set(static_cast<double>(value), ad_can_TrafficSignInfo_conn_detection); };
};
#define ad_can_SystemInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_time_sync_status {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_v_can_status {ValueType::Unsigned, 1, 0, 18, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_avs_status {ValueType::Unsigned, 1, 0, 20, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_network_available {ValueType::Unsigned, 1, 0, 22, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_device_alive_code {ValueType::Unsigned, 1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_connection_nodes {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_SystemInfo_voltage_level {ValueType::Unsigned, 0.1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
class SystemInfo : public MessageBase {
public:
  SystemInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 124, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  enum class TimeSyncStatus { kNa = 0, kNotSynchronized = 1, kOk = 3 };
  enum class VCanStatus { kNa = 0, kNotConnected = 1, kErrors = 2, kOk = 3 };
  enum class AvsStatus { kNa = 0, kNotConnected = 1, kConnected = 2 };
  enum class NetworkAvailable { kNa = 0, kNoNetwork = 1, kSlow = 2, kOk = 4 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_SystemInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_SystemInfo_life_count)); };
  inline TimeSyncStatus time_sync_status() { return static_cast<TimeSyncStatus>(get(ad_can_SystemInfo_time_sync_status)); };
  inline VCanStatus v_can_status() { return static_cast<VCanStatus>(get(ad_can_SystemInfo_v_can_status)); };
  inline AvsStatus avs_status() { return static_cast<AvsStatus>(get(ad_can_SystemInfo_avs_status)); };
  inline NetworkAvailable network_available() { return static_cast<NetworkAvailable>(get(ad_can_SystemInfo_network_available)); };
  inline uint8_t device_alive_code() { return static_cast<uint8_t>(get(ad_can_SystemInfo_device_alive_code)); };
  inline uint8_t connection_nodes() { return static_cast<uint8_t>(get(ad_can_SystemInfo_connection_nodes)); };
  inline double voltage_level() { return static_cast<double>(get(ad_can_SystemInfo_voltage_level)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_SystemInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_SystemInfo_life_count); };
  inline void time_sync_status(const TimeSyncStatus& value) { set(static_cast<double>(value), ad_can_SystemInfo_time_sync_status); };
  inline void v_can_status(const VCanStatus& value) { set(static_cast<double>(value), ad_can_SystemInfo_v_can_status); };
  inline void avs_status(const AvsStatus& value) { set(static_cast<double>(value), ad_can_SystemInfo_avs_status); };
  inline void network_available(const NetworkAvailable& value) { set(static_cast<double>(value), ad_can_SystemInfo_network_available); };
  inline void device_alive_code(const uint8_t& value) { set(static_cast<double>(value), ad_can_SystemInfo_device_alive_code); };
  inline void connection_nodes(const uint8_t& value) { set(static_cast<double>(value), ad_can_SystemInfo_connection_nodes); };
  inline void voltage_level(const double& value) { set(static_cast<double>(value), ad_can_SystemInfo_voltage_level); };
};
#define ad_can_LoggerInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LoggerInfo_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_LoggerInfo_event_logging {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_LoggerInfo_manual_logging {ValueType::Unsigned, 1, 0, 18, 2, ByteOrder::LittleEndian} // unit: []
class LoggerInfo : public MessageBase {
public:
  LoggerInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 125, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  enum class EventLogging { kNa = 0, kLogging = 1 };
  enum class ManualLogging { kNa = 0, kLogging = 1 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_LoggerInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_LoggerInfo_life_count)); };
  inline EventLogging event_logging() { return static_cast<EventLogging>(get(ad_can_LoggerInfo_event_logging)); };
  inline ManualLogging manual_logging() { return static_cast<ManualLogging>(get(ad_can_LoggerInfo_manual_logging)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_LoggerInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_LoggerInfo_life_count); };
  inline void event_logging(const EventLogging& value) { set(static_cast<double>(value), ad_can_LoggerInfo_event_logging); };
  inline void manual_logging(const ManualLogging& value) { set(static_cast<double>(value), ad_can_LoggerInfo_manual_logging); };
};
#define ad_can_ObjectList_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectList_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectList_frame_id {ValueType::Unsigned, 1, 0, 16, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectList_number_of_objects {ValueType::Unsigned, 1, 0, 18, 4, ByteOrder::LittleEndian} // unit: []
class ObjectList : public MessageBase {
public:
  ObjectList(uint8_t* arr = nullptr): MessageBase(&data[0], 144, 3, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[3];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_ObjectList_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_ObjectList_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_ObjectList_frame_id)); };
  inline uint8_t number_of_objects() { return static_cast<uint8_t>(get(ad_can_ObjectList_number_of_objects)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectList_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectList_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectList_frame_id); };
  inline void number_of_objects(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectList_number_of_objects); };
};
#define ad_can_ObjectInfo_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_frame_id {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_index {ValueType::Unsigned, 1, 0, 14, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_x {ValueType::Unsigned, 0.1, -51.2, 18, 10, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_y {ValueType::Unsigned, 0.1, -51.2, 28, 10, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_box_length {ValueType::Unsigned, 0.2, 0, 38, 7, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_box_width {ValueType::Unsigned, 0.2, 0, 45, 7, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectInfo_heading {ValueType::Unsigned, 0.1, -180, 52, 12, ByteOrder::LittleEndian} // unit: [ ]
class ObjectInfo : public MessageBase {
public:
  ObjectInfo(uint8_t* arr = nullptr): MessageBase(&data[0], 145, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_ObjectInfo_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_ObjectInfo_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_ObjectInfo_frame_id)); };
  inline uint8_t index() { return static_cast<uint8_t>(get(ad_can_ObjectInfo_index)); };
  inline double x() { return static_cast<double>(get(ad_can_ObjectInfo_x)); };
  inline double y() { return static_cast<double>(get(ad_can_ObjectInfo_y)); };
  inline double box_length() { return static_cast<double>(get(ad_can_ObjectInfo_box_length)); };
  inline double box_width() { return static_cast<double>(get(ad_can_ObjectInfo_box_width)); };
  // unit: [ ]
  inline double heading() { return static_cast<double>(get(ad_can_ObjectInfo_heading)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectInfo_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectInfo_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectInfo_frame_id); };
  inline void index(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectInfo_index); };
  inline void x(const double& value) { set(static_cast<double>(value), ad_can_ObjectInfo_x); };
  inline void y(const double& value) { set(static_cast<double>(value), ad_can_ObjectInfo_y); };
  inline void box_length(const double& value) { set(static_cast<double>(value), ad_can_ObjectInfo_box_length); };
  inline void box_width(const double& value) { set(static_cast<double>(value), ad_can_ObjectInfo_box_width); };
  // unit: [ ]
  inline void heading(const double& value) { set(static_cast<double>(value), ad_can_ObjectInfo_heading); };
};
#define ad_can_ObjectExt_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_life_count {ValueType::Unsigned, 1, 0, 8, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_frame_id {ValueType::Unsigned, 1, 0, 12, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_index {ValueType::Unsigned, 1, 0, 14, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_vx {ValueType::Unsigned, 0.1, -102.4, 18, 11, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_vy {ValueType::Unsigned, 0.1, -102.4, 29, 11, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_object_class {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define ad_can_ObjectExt_status {ValueType::Unsigned, 1, 0, 44, 4, ByteOrder::LittleEndian} // unit: []
class ObjectExt : public MessageBase {
public:
  ObjectExt(uint8_t* arr = nullptr): MessageBase(&data[0], 146, 6, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[6];
  //* value table
  enum class ObjectClass { kUnknown = 0, kPedestrian = 1, kBike = 2, kCar = 3, kTruck = 4, kMotorcycle = 5 };
  enum class Status { kUnknown = 0, kStop = 1, kStationary = 2, kMoving = 3 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_ObjectExt_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_ObjectExt_life_count)); };
  inline uint8_t frame_id() { return static_cast<uint8_t>(get(ad_can_ObjectExt_frame_id)); };
  inline uint8_t index() { return static_cast<uint8_t>(get(ad_can_ObjectExt_index)); };
  inline double vx() { return static_cast<double>(get(ad_can_ObjectExt_vx)); };
  inline double vy() { return static_cast<double>(get(ad_can_ObjectExt_vy)); };
  inline ObjectClass object_class() { return static_cast<ObjectClass>(get(ad_can_ObjectExt_object_class)); };
  inline Status status() { return static_cast<Status>(get(ad_can_ObjectExt_status)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectExt_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectExt_life_count); };
  inline void frame_id(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectExt_frame_id); };
  inline void index(const uint8_t& value) { set(static_cast<double>(value), ad_can_ObjectExt_index); };
  inline void vx(const double& value) { set(static_cast<double>(value), ad_can_ObjectExt_vx); };
  inline void vy(const double& value) { set(static_cast<double>(value), ad_can_ObjectExt_vy); };
  inline void object_class(const ObjectClass& value) { set(static_cast<double>(value), ad_can_ObjectExt_object_class); };
  inline void status(const Status& value) { set(static_cast<double>(value), ad_can_ObjectExt_status); };
};
#define ad_can_UserConfiguration_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_UserConfiguration_life_counter {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_UserConfiguration_max_speed {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_UserConfiguration_max_steering_torque {ValueType::Unsigned, 0.1, 0, 24, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_UserConfiguration_max_acceleration {ValueType::Unsigned, 0.1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_UserConfiguration_max_deceleration {ValueType::Unsigned, -0.1, 0, 40, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_UserConfiguration_max_jerk {ValueType::Unsigned, 0.01, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
class UserConfiguration : public MessageBase {
public:
  UserConfiguration(uint8_t* arr = nullptr): MessageBase(&data[0], 256, 8, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[8];
  //* value table
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_UserConfiguration_crc8)); };
  inline uint8_t life_counter() { return static_cast<uint8_t>(get(ad_can_UserConfiguration_life_counter)); };
  inline uint8_t max_speed() { return static_cast<uint8_t>(get(ad_can_UserConfiguration_max_speed)); };
  inline double max_steering_torque() { return static_cast<double>(get(ad_can_UserConfiguration_max_steering_torque)); };
  inline double max_acceleration() { return static_cast<double>(get(ad_can_UserConfiguration_max_acceleration)); };
  inline double max_deceleration() { return static_cast<double>(get(ad_can_UserConfiguration_max_deceleration)); };
  inline double max_jerk() { return static_cast<double>(get(ad_can_UserConfiguration_max_jerk)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_UserConfiguration_crc8); };
  inline void life_counter(const uint8_t& value) { set(static_cast<double>(value), ad_can_UserConfiguration_life_counter); };
  inline void max_speed(const uint8_t& value) { set(static_cast<double>(value), ad_can_UserConfiguration_max_speed); };
  inline void max_steering_torque(const double& value) { set(static_cast<double>(value), ad_can_UserConfiguration_max_steering_torque); };
  inline void max_acceleration(const double& value) { set(static_cast<double>(value), ad_can_UserConfiguration_max_acceleration); };
  inline void max_deceleration(const double& value) { set(static_cast<double>(value), ad_can_UserConfiguration_max_deceleration); };
  inline void max_jerk(const double& value) { set(static_cast<double>(value), ad_can_UserConfiguration_max_jerk); };
};
#define ad_can_DrivingLoggerInfo0_engine_rpm {ValueType::Unsigned, 0.25, 0, 16, 16, ByteOrder::LittleEndian} // unit: []
#define ad_can_DrivingLoggerInfo0_vehicle_speed {ValueType::Unsigned, 1, 0, 48, 8, ByteOrder::LittleEndian} // unit: []
class DrivingLoggerInfo0 : public MessageBase {
public:
  DrivingLoggerInfo0(uint8_t* arr = nullptr): MessageBase(&data[0], 790, 7, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[7];
  //* value table
  //* get interface
  inline double engine_rpm() { return static_cast<double>(get(ad_can_DrivingLoggerInfo0_engine_rpm)); };
  inline uint8_t vehicle_speed() { return static_cast<uint8_t>(get(ad_can_DrivingLoggerInfo0_vehicle_speed)); };
  //* set interface
  inline void engine_rpm(const double& value) { set(static_cast<double>(value), ad_can_DrivingLoggerInfo0_engine_rpm); };
  inline void vehicle_speed(const uint8_t& value) { set(static_cast<double>(value), ad_can_DrivingLoggerInfo0_vehicle_speed); };
};
#define ad_can_DrivingLoggerInfo1_brake_signal {ValueType::Unsigned, 1, 0, 33, 1, ByteOrder::LittleEndian} // unit: []
class DrivingLoggerInfo1 : public MessageBase {
public:
  DrivingLoggerInfo1(uint8_t* arr = nullptr): MessageBase(&data[0], 809, 5, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[5];
  //* value table
  //* get interface
  inline uint8_t brake_signal() { return static_cast<uint8_t>(get(ad_can_DrivingLoggerInfo1_brake_signal)); };
  //* set interface
  inline void brake_signal(const uint8_t& value) { set(static_cast<double>(value), ad_can_DrivingLoggerInfo1_brake_signal); };
};
#define ad_can_DoorControl_crc8 {ValueType::Unsigned, 1, 0, 0, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_DoorControl_life_count {ValueType::Unsigned, 1, 0, 8, 8, ByteOrder::LittleEndian} // unit: []
#define ad_can_DoorControl_door_control_mode {ValueType::Unsigned, 1, 0, 16, 1, ByteOrder::LittleEndian} // unit: []
#define ad_can_DoorControl_door_lock_cmd {ValueType::Unsigned, 1, 0, 20, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_DoorControl_power_door_left_cmd {ValueType::Unsigned, 1, 0, 22, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_DoorControl_power_door_right_cmd {ValueType::Unsigned, 1, 0, 24, 2, ByteOrder::LittleEndian} // unit: []
#define ad_can_DoorControl_trunk_door_cmd {ValueType::Unsigned, 1, 0, 26, 2, ByteOrder::LittleEndian} // unit: []
class DoorControl : public MessageBase {
public:
  DoorControl(uint8_t* arr = nullptr): MessageBase(&data[0], 87, 4, false, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[4];
  //* value table
  enum class DoorLockCmd { kNa = 0, kLock = 1, kUnlock = 2 };
  enum class PowerDoorLeftCmd { kNa = 0, kClose = 1, kOpen = 2 };
  enum class PowerDoorRightCmd { kNa = 0, kClose = 1, kOpen = 2 };
  enum class TrunkDoorCmd { kNa = 0, kClose = 1, kOpen = 2 };
  //* get interface
  inline uint8_t crc8() { return static_cast<uint8_t>(get(ad_can_DoorControl_crc8)); };
  inline uint8_t life_count() { return static_cast<uint8_t>(get(ad_can_DoorControl_life_count)); };
  inline uint8_t door_control_mode() { return static_cast<uint8_t>(get(ad_can_DoorControl_door_control_mode)); };
  inline DoorLockCmd door_lock_cmd() { return static_cast<DoorLockCmd>(get(ad_can_DoorControl_door_lock_cmd)); };
  inline PowerDoorLeftCmd power_door_left_cmd() { return static_cast<PowerDoorLeftCmd>(get(ad_can_DoorControl_power_door_left_cmd)); };
  inline PowerDoorRightCmd power_door_right_cmd() { return static_cast<PowerDoorRightCmd>(get(ad_can_DoorControl_power_door_right_cmd)); };
  inline TrunkDoorCmd trunk_door_cmd() { return static_cast<TrunkDoorCmd>(get(ad_can_DoorControl_trunk_door_cmd)); };
  //* set interface
  inline void crc8(const uint8_t& value) { set(static_cast<double>(value), ad_can_DoorControl_crc8); };
  inline void life_count(const uint8_t& value) { set(static_cast<double>(value), ad_can_DoorControl_life_count); };
  inline void door_control_mode(const uint8_t& value) { set(static_cast<double>(value), ad_can_DoorControl_door_control_mode); };
  inline void door_lock_cmd(const DoorLockCmd& value) { set(static_cast<double>(value), ad_can_DoorControl_door_lock_cmd); };
  inline void power_door_left_cmd(const PowerDoorLeftCmd& value) { set(static_cast<double>(value), ad_can_DoorControl_power_door_left_cmd); };
  inline void power_door_right_cmd(const PowerDoorRightCmd& value) { set(static_cast<double>(value), ad_can_DoorControl_power_door_right_cmd); };
  inline void trunk_door_cmd(const TrunkDoorCmd& value) { set(static_cast<double>(value), ad_can_DoorControl_trunk_door_cmd); };
};

} // namespace ad_can
} // namespace dbc
#endif // AD_CAN_DBC_HPP__
