#ifndef CAMERA_DBC_HPP__
#define CAMERA_DBC_HPP__

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

namespace camera {
enum class Id {
  FrontCam3DObject01 = 699,
  FrontCam3DObject02 = 700,
  FrontCam3DObject03 = 701,
  FrontCam3DObject04 = 702,
  FrontCamLaneEgo = 674,
  FrontCamLaneNext = 675,
  FrontCamLaneExt = 676,
  FrontCamObject01 = 384,
  FrontCamObject02 = 385,
  FrontCamObject03 = 386,
  FrontCamObject04 = 387,
  FrontCamObject05 = 388,
  FrontCamObject06 = 438,
  FrontCamObject07 = 439,
  FrontCamObject08 = 440,
  FrontCamObject09 = 441,
  FrontCamObject10 = 507,
};

#define camera_FrontCam3DObject01_life_count_3D_01 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCam3DObject01_heading_angle_01 {ValueType::Unsigned, 0.01, -3.14, 24, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_02 {ValueType::Unsigned, 0.01, -3.14, 34, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_03 {ValueType::Unsigned, 0.01, -3.14, 44, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_04 {ValueType::Unsigned, 0.01, -3.14, 54, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_05 {ValueType::Unsigned, 0.01, -3.14, 64, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_06 {ValueType::Unsigned, 0.01, -3.14, 74, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_07 {ValueType::Unsigned, 0.01, -3.14, 84, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_08 {ValueType::Unsigned, 0.01, -3.14, 94, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_09 {ValueType::Unsigned, 0.01, -3.14, 104, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject01_heading_angle_10 {ValueType::Unsigned, 0.01, -3.14, 114, 10, ByteOrder::LittleEndian} // unit: [rad]
class FrontCam3DObject01 : public MessageBase {
public:
  FrontCam3DObject01(uint8_t* arr = nullptr): MessageBase(&data[0], 699, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_3D_01() { return static_cast<uint8_t>(get(camera_FrontCam3DObject01_life_count_3D_01)); };
  // unit: [rad]
  inline double heading_angle_01() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_01)); };
  // unit: [rad]
  inline double heading_angle_02() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_02)); };
  // unit: [rad]
  inline double heading_angle_03() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_03)); };
  // unit: [rad]
  inline double heading_angle_04() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_04)); };
  // unit: [rad]
  inline double heading_angle_05() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_05)); };
  // unit: [rad]
  inline double heading_angle_06() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_06)); };
  // unit: [rad]
  inline double heading_angle_07() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_07)); };
  // unit: [rad]
  inline double heading_angle_08() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_08)); };
  // unit: [rad]
  inline double heading_angle_09() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_09)); };
  // unit: [rad]
  inline double heading_angle_10() { return static_cast<double>(get(camera_FrontCam3DObject01_heading_angle_10)); };
  //* set interface
  inline void life_count_3D_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_life_count_3D_01); };
  // unit: [rad]
  inline void heading_angle_01(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_01); };
  // unit: [rad]
  inline void heading_angle_02(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_02); };
  // unit: [rad]
  inline void heading_angle_03(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_03); };
  // unit: [rad]
  inline void heading_angle_04(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_04); };
  // unit: [rad]
  inline void heading_angle_05(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_05); };
  // unit: [rad]
  inline void heading_angle_06(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_06); };
  // unit: [rad]
  inline void heading_angle_07(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_07); };
  // unit: [rad]
  inline void heading_angle_08(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_08); };
  // unit: [rad]
  inline void heading_angle_09(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_09); };
  // unit: [rad]
  inline void heading_angle_10(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject01_heading_angle_10); };
};
#define camera_FrontCam3DObject02_life_count_3D_02 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCam3DObject02_length_object_01 {ValueType::Unsigned, 0.05, 0, 24, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_02 {ValueType::Unsigned, 0.05, 0, 32, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_03 {ValueType::Unsigned, 0.05, 0, 40, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_04 {ValueType::Unsigned, 0.05, 0, 48, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_05 {ValueType::Unsigned, 0.05, 0, 56, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_06 {ValueType::Unsigned, 0.05, 0, 64, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_07 {ValueType::Unsigned, 0.05, 0, 72, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_08 {ValueType::Unsigned, 0.05, 0, 80, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_09 {ValueType::Unsigned, 0.05, 0, 88, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject02_length_object_10 {ValueType::Unsigned, 0.05, 0, 96, 8, ByteOrder::LittleEndian} // unit: [m]
class FrontCam3DObject02 : public MessageBase {
public:
  FrontCam3DObject02(uint8_t* arr = nullptr): MessageBase(&data[0], 700, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_3D_02() { return static_cast<uint8_t>(get(camera_FrontCam3DObject02_life_count_3D_02)); };
  // unit: [m]
  inline double length_object_01() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_01)); };
  // unit: [m]
  inline double length_object_02() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_02)); };
  // unit: [m]
  inline double length_object_03() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_03)); };
  // unit: [m]
  inline double length_object_04() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_04)); };
  // unit: [m]
  inline double length_object_05() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_05)); };
  // unit: [m]
  inline double length_object_06() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_06)); };
  // unit: [m]
  inline double length_object_07() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_07)); };
  // unit: [m]
  inline double length_object_08() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_08)); };
  // unit: [m]
  inline double length_object_09() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_09)); };
  // unit: [m]
  inline double length_object_10() { return static_cast<double>(get(camera_FrontCam3DObject02_length_object_10)); };
  //* set interface
  inline void life_count_3D_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_life_count_3D_02); };
  // unit: [m]
  inline void length_object_01(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_01); };
  // unit: [m]
  inline void length_object_02(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_02); };
  // unit: [m]
  inline void length_object_03(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_03); };
  // unit: [m]
  inline void length_object_04(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_04); };
  // unit: [m]
  inline void length_object_05(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_05); };
  // unit: [m]
  inline void length_object_06(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_06); };
  // unit: [m]
  inline void length_object_07(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_07); };
  // unit: [m]
  inline void length_object_08(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_08); };
  // unit: [m]
  inline void length_object_09(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_09); };
  // unit: [m]
  inline void length_object_10(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject02_length_object_10); };
};
#define camera_FrontCam3DObject03_life_count_3D_03 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCam3DObject03_heading_angle_11 {ValueType::Unsigned, 0.01, -3.14, 24, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_12 {ValueType::Unsigned, 0.01, -3.14, 34, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_13 {ValueType::Unsigned, 0.01, -3.14, 44, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_14 {ValueType::Unsigned, 0.01, -3.14, 54, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_15 {ValueType::Unsigned, 0.01, -3.14, 64, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_16 {ValueType::Unsigned, 0.01, -3.14, 74, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_17 {ValueType::Unsigned, 0.01, -3.14, 84, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_18 {ValueType::Unsigned, 0.01, -3.14, 94, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_19 {ValueType::Unsigned, 0.01, -3.14, 104, 10, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCam3DObject03_heading_angle_20 {ValueType::Unsigned, 0.01, -3.14, 114, 10, ByteOrder::LittleEndian} // unit: [rad]
class FrontCam3DObject03 : public MessageBase {
public:
  FrontCam3DObject03(uint8_t* arr = nullptr): MessageBase(&data[0], 701, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_3D_03() { return static_cast<uint8_t>(get(camera_FrontCam3DObject03_life_count_3D_03)); };
  // unit: [rad]
  inline double heading_angle_11() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_11)); };
  // unit: [rad]
  inline double heading_angle_12() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_12)); };
  // unit: [rad]
  inline double heading_angle_13() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_13)); };
  // unit: [rad]
  inline double heading_angle_14() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_14)); };
  // unit: [rad]
  inline double heading_angle_15() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_15)); };
  // unit: [rad]
  inline double heading_angle_16() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_16)); };
  // unit: [rad]
  inline double heading_angle_17() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_17)); };
  // unit: [rad]
  inline double heading_angle_18() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_18)); };
  // unit: [rad]
  inline double heading_angle_19() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_19)); };
  // unit: [rad]
  inline double heading_angle_20() { return static_cast<double>(get(camera_FrontCam3DObject03_heading_angle_20)); };
  //* set interface
  inline void life_count_3D_03(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_life_count_3D_03); };
  // unit: [rad]
  inline void heading_angle_11(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_11); };
  // unit: [rad]
  inline void heading_angle_12(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_12); };
  // unit: [rad]
  inline void heading_angle_13(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_13); };
  // unit: [rad]
  inline void heading_angle_14(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_14); };
  // unit: [rad]
  inline void heading_angle_15(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_15); };
  // unit: [rad]
  inline void heading_angle_16(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_16); };
  // unit: [rad]
  inline void heading_angle_17(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_17); };
  // unit: [rad]
  inline void heading_angle_18(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_18); };
  // unit: [rad]
  inline void heading_angle_19(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_19); };
  // unit: [rad]
  inline void heading_angle_20(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject03_heading_angle_20); };
};
#define camera_FrontCam3DObject04_life_count_3D_04 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCam3DObject04_length_object_11 {ValueType::Unsigned, 0.05, 0, 24, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_12 {ValueType::Unsigned, 0.05, 0, 32, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_13 {ValueType::Unsigned, 0.05, 0, 40, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_14 {ValueType::Unsigned, 0.05, 0, 48, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_15 {ValueType::Unsigned, 0.05, 0, 56, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_16 {ValueType::Unsigned, 0.05, 0, 64, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_17 {ValueType::Unsigned, 0.05, 0, 72, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_18 {ValueType::Unsigned, 0.05, 0, 80, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_19 {ValueType::Unsigned, 0.05, 0, 88, 8, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCam3DObject04_length_object_20 {ValueType::Unsigned, 0.05, 0, 96, 8, ByteOrder::LittleEndian} // unit: [m]
class FrontCam3DObject04 : public MessageBase {
public:
  FrontCam3DObject04(uint8_t* arr = nullptr): MessageBase(&data[0], 702, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_3D_04() { return static_cast<uint8_t>(get(camera_FrontCam3DObject04_life_count_3D_04)); };
  // unit: [m]
  inline double length_object_11() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_11)); };
  // unit: [m]
  inline double length_object_12() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_12)); };
  // unit: [m]
  inline double length_object_13() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_13)); };
  // unit: [m]
  inline double length_object_14() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_14)); };
  // unit: [m]
  inline double length_object_15() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_15)); };
  // unit: [m]
  inline double length_object_16() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_16)); };
  // unit: [m]
  inline double length_object_17() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_17)); };
  // unit: [m]
  inline double length_object_18() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_18)); };
  // unit: [m]
  inline double length_object_19() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_19)); };
  // unit: [m]
  inline double length_object_20() { return static_cast<double>(get(camera_FrontCam3DObject04_length_object_20)); };
  //* set interface
  inline void life_count_3D_04(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_life_count_3D_04); };
  // unit: [m]
  inline void length_object_11(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_11); };
  // unit: [m]
  inline void length_object_12(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_12); };
  // unit: [m]
  inline void length_object_13(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_13); };
  // unit: [m]
  inline void length_object_14(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_14); };
  // unit: [m]
  inline void length_object_15(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_15); };
  // unit: [m]
  inline void length_object_16(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_16); };
  // unit: [m]
  inline void length_object_17(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_17); };
  // unit: [m]
  inline void length_object_18(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_18); };
  // unit: [m]
  inline void length_object_19(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_19); };
  // unit: [m]
  inline void length_object_20(const double& value) { set(static_cast<double>(value), camera_FrontCam3DObject04_length_object_20); };
};
#define camera_FrontCamLaneEgo_life_count_lane_ego {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneEgo_lane_param_d_left_ego {ValueType::Unsigned, 0.01, -10, 24, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneEgo_lane_param_d_right_ego {ValueType::Unsigned, 0.01, -10, 40, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneEgo_lane_param_c_left_ego {ValueType::Unsigned, 0.000977, -0.357, 64, 16, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCamLaneEgo_lane_param_c_right_ego {ValueType::Unsigned, 0.000977, -0.357, 96, 16, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCamLaneEgo_lane_param_b_left_ego {ValueType::Unsigned, 9.77e-07, -0.032, 128, 32, ByteOrder::LittleEndian} // unit: [1/m]
#define camera_FrontCamLaneEgo_lane_param_b_right_ego {ValueType::Unsigned, 9.77e-07, -0.032, 160, 32, ByteOrder::LittleEndian} // unit: [1/m]
#define camera_FrontCamLaneEgo_lane_param_a_left_ego {ValueType::Unsigned, 3.73e-09, -0.00012, 192, 32, ByteOrder::LittleEndian} // unit: [1/m^2]
#define camera_FrontCamLaneEgo_lane_param_a_right_ego {ValueType::Unsigned, 3.73e-09, -0.00012, 224, 32, ByteOrder::LittleEndian} // unit: [1/m^2]
class FrontCamLaneEgo : public MessageBase {
public:
  FrontCamLaneEgo(uint8_t* arr = nullptr): MessageBase(&data[0], 674, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_lane_ego() { return static_cast<uint8_t>(get(camera_FrontCamLaneEgo_life_count_lane_ego)); };
  // unit: [m]
  inline double lane_param_d_left_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_d_left_ego)); };
  // unit: [m]
  inline double lane_param_d_right_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_d_right_ego)); };
  // unit: [rad]
  inline double lane_param_c_left_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_c_left_ego)); };
  // unit: [rad]
  inline double lane_param_c_right_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_c_right_ego)); };
  // unit: [1/m]
  inline double lane_param_b_left_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_b_left_ego)); };
  // unit: [1/m]
  inline double lane_param_b_right_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_b_right_ego)); };
  // unit: [1/m^2]
  inline double lane_param_a_left_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_a_left_ego)); };
  // unit: [1/m^2]
  inline double lane_param_a_right_ego() { return static_cast<double>(get(camera_FrontCamLaneEgo_lane_param_a_right_ego)); };
  //* set interface
  inline void life_count_lane_ego(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_life_count_lane_ego); };
  // unit: [m]
  inline void lane_param_d_left_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_d_left_ego); };
  // unit: [m]
  inline void lane_param_d_right_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_d_right_ego); };
  // unit: [rad]
  inline void lane_param_c_left_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_c_left_ego); };
  // unit: [rad]
  inline void lane_param_c_right_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_c_right_ego); };
  // unit: [1/m]
  inline void lane_param_b_left_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_b_left_ego); };
  // unit: [1/m]
  inline void lane_param_b_right_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_b_right_ego); };
  // unit: [1/m^2]
  inline void lane_param_a_left_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_a_left_ego); };
  // unit: [1/m^2]
  inline void lane_param_a_right_ego(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneEgo_lane_param_a_right_ego); };
};
#define camera_FrontCamLaneNext_life_count_lane_next {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneNext_lane_param_d_left_next {ValueType::Unsigned, 0.01, -10, 24, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneNext_lane_param_d_right_next {ValueType::Unsigned, 0.01, -10, 40, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneNext_lane_param_c_left_next {ValueType::Unsigned, 0.000977, -0.357, 64, 16, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCamLaneNext_lane_param_c_right_next {ValueType::Unsigned, 0.000977, -0.357, 96, 16, ByteOrder::LittleEndian} // unit: [rad]
#define camera_FrontCamLaneNext_lane_param_b_left_next {ValueType::Unsigned, 9.77e-07, -0.032, 128, 32, ByteOrder::LittleEndian} // unit: [1/m]
#define camera_FrontCamLaneNext_lane_param_b_right_next {ValueType::Unsigned, 9.77e-07, -0.032, 160, 32, ByteOrder::LittleEndian} // unit: [1/m]
#define camera_FrontCamLaneNext_lane_param_a_left_next {ValueType::Unsigned, 3.73e-09, -0.00012, 192, 32, ByteOrder::LittleEndian} // unit: [1/m^2]
#define camera_FrontCamLaneNext_lane_param_a_right_next {ValueType::Unsigned, 3.73e-09, -0.00012, 224, 32, ByteOrder::LittleEndian} // unit: [1/m^2]
class FrontCamLaneNext : public MessageBase {
public:
  FrontCamLaneNext(uint8_t* arr = nullptr): MessageBase(&data[0], 675, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_lane_next() { return static_cast<uint8_t>(get(camera_FrontCamLaneNext_life_count_lane_next)); };
  // unit: [m]
  inline double lane_param_d_left_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_d_left_next)); };
  // unit: [m]
  inline double lane_param_d_right_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_d_right_next)); };
  // unit: [rad]
  inline double lane_param_c_left_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_c_left_next)); };
  // unit: [rad]
  inline double lane_param_c_right_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_c_right_next)); };
  // unit: [1/m]
  inline double lane_param_b_left_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_b_left_next)); };
  // unit: [1/m]
  inline double lane_param_b_right_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_b_right_next)); };
  // unit: [1/m^2]
  inline double lane_param_a_left_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_a_left_next)); };
  // unit: [1/m^2]
  inline double lane_param_a_right_next() { return static_cast<double>(get(camera_FrontCamLaneNext_lane_param_a_right_next)); };
  //* set interface
  inline void life_count_lane_next(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_life_count_lane_next); };
  // unit: [m]
  inline void lane_param_d_left_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_d_left_next); };
  // unit: [m]
  inline void lane_param_d_right_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_d_right_next); };
  // unit: [rad]
  inline void lane_param_c_left_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_c_left_next); };
  // unit: [rad]
  inline void lane_param_c_right_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_c_right_next); };
  // unit: [1/m]
  inline void lane_param_b_left_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_b_left_next); };
  // unit: [1/m]
  inline void lane_param_b_right_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_b_right_next); };
  // unit: [1/m^2]
  inline void lane_param_a_left_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_a_left_next); };
  // unit: [1/m^2]
  inline void lane_param_a_right_next(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneNext_lane_param_a_right_next); };
};
#define camera_FrontCamLaneExt_life_count_lane_ext {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneExt_quality_level_lane_left_01 {ValueType::Unsigned, 1, 0, 56, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneExt_quality_level_lane_right_01 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneExt_quality_level_lane_left_02 {ValueType::Unsigned, 1, 0, 72, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneExt_quality_level_lane_right_02 {ValueType::Unsigned, 1, 0, 76, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamLaneExt_view_range_left {ValueType::Unsigned, 0.1, 0, 80, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneExt_view_range_right {ValueType::Unsigned, 0.1, 0, 96, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneExt_view_range_start_left {ValueType::Unsigned, 0.1, 0, 112, 16, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamLaneExt_view_range_start_right {ValueType::Unsigned, 0.1, 0, 128, 16, ByteOrder::LittleEndian} // unit: [m]
class FrontCamLaneExt : public MessageBase {
public:
  FrontCamLaneExt(uint8_t* arr = nullptr): MessageBase(&data[0], 676, 24, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[24];
  //* value table
  //* get interface
  inline uint8_t life_count_lane_ext() { return static_cast<uint8_t>(get(camera_FrontCamLaneExt_life_count_lane_ext)); };
  inline uint8_t quality_level_lane_left_01() { return static_cast<uint8_t>(get(camera_FrontCamLaneExt_quality_level_lane_left_01)); };
  inline uint8_t quality_level_lane_right_01() { return static_cast<uint8_t>(get(camera_FrontCamLaneExt_quality_level_lane_right_01)); };
  inline uint8_t quality_level_lane_left_02() { return static_cast<uint8_t>(get(camera_FrontCamLaneExt_quality_level_lane_left_02)); };
  inline uint8_t quality_level_lane_right_02() { return static_cast<uint8_t>(get(camera_FrontCamLaneExt_quality_level_lane_right_02)); };
  // unit: [m]
  inline double view_range_left() { return static_cast<double>(get(camera_FrontCamLaneExt_view_range_left)); };
  // unit: [m]
  inline double view_range_right() { return static_cast<double>(get(camera_FrontCamLaneExt_view_range_right)); };
  // unit: [m]
  inline double view_range_start_left() { return static_cast<double>(get(camera_FrontCamLaneExt_view_range_start_left)); };
  // unit: [m]
  inline double view_range_start_right() { return static_cast<double>(get(camera_FrontCamLaneExt_view_range_start_right)); };
  //* set interface
  inline void life_count_lane_ext(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_life_count_lane_ext); };
  inline void quality_level_lane_left_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_quality_level_lane_left_01); };
  inline void quality_level_lane_right_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_quality_level_lane_right_01); };
  inline void quality_level_lane_left_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_quality_level_lane_left_02); };
  inline void quality_level_lane_right_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_quality_level_lane_right_02); };
  // unit: [m]
  inline void view_range_left(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_view_range_left); };
  // unit: [m]
  inline void view_range_right(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_view_range_right); };
  // unit: [m]
  inline void view_range_start_left(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_view_range_start_left); };
  // unit: [m]
  inline void view_range_start_right(const double& value) { set(static_cast<double>(value), camera_FrontCamLaneExt_view_range_start_right); };
};
#define camera_FrontCamObject01_life_count_message_01 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_quality_level_object_01 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject01_life_count_object_01 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_moving_flag_01 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_width_object_01 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject01_classification_object_01 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_relative_X_position_01 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject01_relative_Y_position_01 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject01_relative_X_velocity_01 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject01_relative_Y_velocity_01 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject01_relative_X_acceleration_01 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject01_quality_level_object_02 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject01_life_count_object_02 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_moving_flag_02 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_width_object_02 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject01_classification_object_02 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject01_relative_X_position_02 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject01_relative_Y_position_02 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject01_relative_X_velocity_02 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject01_relative_Y_velocity_02 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject01_relative_X_acceleration_02 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject01 : public MessageBase {
public:
  FrontCamObject01(uint8_t* arr = nullptr): MessageBase(&data[0], 384, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_01() { return static_cast<uint8_t>(get(camera_FrontCamObject01_life_count_message_01)); };
  // unit: [%]
  inline uint8_t quality_level_object_01() { return static_cast<uint8_t>(get(camera_FrontCamObject01_quality_level_object_01)); };
  inline uint8_t life_count_object_01() { return static_cast<uint8_t>(get(camera_FrontCamObject01_life_count_object_01)); };
  inline uint8_t moving_flag_01() { return static_cast<uint8_t>(get(camera_FrontCamObject01_moving_flag_01)); };
  // unit: [m]
  inline double width_object_01() { return static_cast<double>(get(camera_FrontCamObject01_width_object_01)); };
  inline uint8_t classification_object_01() { return static_cast<uint8_t>(get(camera_FrontCamObject01_classification_object_01)); };
  // unit: [m]
  inline double relative_X_position_01() { return static_cast<double>(get(camera_FrontCamObject01_relative_X_position_01)); };
  // unit: [m]
  inline double relative_Y_position_01() { return static_cast<double>(get(camera_FrontCamObject01_relative_Y_position_01)); };
  // unit: [m/s]
  inline double relative_X_velocity_01() { return static_cast<double>(get(camera_FrontCamObject01_relative_X_velocity_01)); };
  // unit: [m/s]
  inline double relative_Y_velocity_01() { return static_cast<double>(get(camera_FrontCamObject01_relative_Y_velocity_01)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_01() { return static_cast<double>(get(camera_FrontCamObject01_relative_X_acceleration_01)); };
  // unit: [%]
  inline uint8_t quality_level_object_02() { return static_cast<uint8_t>(get(camera_FrontCamObject01_quality_level_object_02)); };
  inline uint8_t life_count_object_02() { return static_cast<uint8_t>(get(camera_FrontCamObject01_life_count_object_02)); };
  inline uint8_t moving_flag_02() { return static_cast<uint8_t>(get(camera_FrontCamObject01_moving_flag_02)); };
  // unit: [m]
  inline double width_object_02() { return static_cast<double>(get(camera_FrontCamObject01_width_object_02)); };
  inline uint8_t classification_object_02() { return static_cast<uint8_t>(get(camera_FrontCamObject01_classification_object_02)); };
  // unit: [m]
  inline double relative_X_position_02() { return static_cast<double>(get(camera_FrontCamObject01_relative_X_position_02)); };
  // unit: [m]
  inline double relative_Y_position_02() { return static_cast<double>(get(camera_FrontCamObject01_relative_Y_position_02)); };
  // unit: [m/s]
  inline double relative_X_velocity_02() { return static_cast<double>(get(camera_FrontCamObject01_relative_X_velocity_02)); };
  // unit: [m/s]
  inline double relative_Y_velocity_02() { return static_cast<double>(get(camera_FrontCamObject01_relative_Y_velocity_02)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_02() { return static_cast<double>(get(camera_FrontCamObject01_relative_X_acceleration_02)); };
  //* set interface
  inline void life_count_message_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_life_count_message_01); };
  // unit: [%]
  inline void quality_level_object_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_quality_level_object_01); };
  inline void life_count_object_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_life_count_object_01); };
  inline void moving_flag_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_moving_flag_01); };
  // unit: [m]
  inline void width_object_01(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_width_object_01); };
  inline void classification_object_01(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_classification_object_01); };
  // unit: [m]
  inline void relative_X_position_01(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_X_position_01); };
  // unit: [m]
  inline void relative_Y_position_01(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_Y_position_01); };
  // unit: [m/s]
  inline void relative_X_velocity_01(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_X_velocity_01); };
  // unit: [m/s]
  inline void relative_Y_velocity_01(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_Y_velocity_01); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_01(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_X_acceleration_01); };
  // unit: [%]
  inline void quality_level_object_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_quality_level_object_02); };
  inline void life_count_object_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_life_count_object_02); };
  inline void moving_flag_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_moving_flag_02); };
  // unit: [m]
  inline void width_object_02(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_width_object_02); };
  inline void classification_object_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject01_classification_object_02); };
  // unit: [m]
  inline void relative_X_position_02(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_X_position_02); };
  // unit: [m]
  inline void relative_Y_position_02(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_Y_position_02); };
  // unit: [m/s]
  inline void relative_X_velocity_02(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_X_velocity_02); };
  // unit: [m/s]
  inline void relative_Y_velocity_02(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_Y_velocity_02); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_02(const double& value) { set(static_cast<double>(value), camera_FrontCamObject01_relative_X_acceleration_02); };
};
#define camera_FrontCamObject02_life_count_message_02 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_quality_level_object_03 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject02_life_count_object_03 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_moving_flag_03 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_width_object_03 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject02_classification_object_03 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_relative_X_position_03 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject02_relative_Y_position_03 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject02_relative_X_velocity_03 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject02_relative_Y_velocity_03 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject02_relative_X_acceleration_03 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject02_quality_level_object_04 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject02_life_count_object_04 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_moving_flag_04 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_width_object_04 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject02_classification_object_04 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject02_relative_X_position_04 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject02_relative_Y_position_04 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject02_relative_X_velocity_04 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject02_relative_Y_velocity_04 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject02_relative_X_acceleration_04 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject02 : public MessageBase {
public:
  FrontCamObject02(uint8_t* arr = nullptr): MessageBase(&data[0], 385, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_02() { return static_cast<uint8_t>(get(camera_FrontCamObject02_life_count_message_02)); };
  // unit: [%]
  inline uint8_t quality_level_object_03() { return static_cast<uint8_t>(get(camera_FrontCamObject02_quality_level_object_03)); };
  inline uint8_t life_count_object_03() { return static_cast<uint8_t>(get(camera_FrontCamObject02_life_count_object_03)); };
  inline uint8_t moving_flag_03() { return static_cast<uint8_t>(get(camera_FrontCamObject02_moving_flag_03)); };
  // unit: [m]
  inline double width_object_03() { return static_cast<double>(get(camera_FrontCamObject02_width_object_03)); };
  inline uint8_t classification_object_03() { return static_cast<uint8_t>(get(camera_FrontCamObject02_classification_object_03)); };
  // unit: [m]
  inline double relative_X_position_03() { return static_cast<double>(get(camera_FrontCamObject02_relative_X_position_03)); };
  // unit: [m]
  inline double relative_Y_position_03() { return static_cast<double>(get(camera_FrontCamObject02_relative_Y_position_03)); };
  // unit: [m/s]
  inline double relative_X_velocity_03() { return static_cast<double>(get(camera_FrontCamObject02_relative_X_velocity_03)); };
  // unit: [m/s]
  inline double relative_Y_velocity_03() { return static_cast<double>(get(camera_FrontCamObject02_relative_Y_velocity_03)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_03() { return static_cast<double>(get(camera_FrontCamObject02_relative_X_acceleration_03)); };
  // unit: [%]
  inline uint8_t quality_level_object_04() { return static_cast<uint8_t>(get(camera_FrontCamObject02_quality_level_object_04)); };
  inline uint8_t life_count_object_04() { return static_cast<uint8_t>(get(camera_FrontCamObject02_life_count_object_04)); };
  inline uint8_t moving_flag_04() { return static_cast<uint8_t>(get(camera_FrontCamObject02_moving_flag_04)); };
  // unit: [m]
  inline double width_object_04() { return static_cast<double>(get(camera_FrontCamObject02_width_object_04)); };
  inline uint8_t classification_object_04() { return static_cast<uint8_t>(get(camera_FrontCamObject02_classification_object_04)); };
  // unit: [m]
  inline double relative_X_position_04() { return static_cast<double>(get(camera_FrontCamObject02_relative_X_position_04)); };
  // unit: [m]
  inline double relative_Y_position_04() { return static_cast<double>(get(camera_FrontCamObject02_relative_Y_position_04)); };
  // unit: [m/s]
  inline double relative_X_velocity_04() { return static_cast<double>(get(camera_FrontCamObject02_relative_X_velocity_04)); };
  // unit: [m/s]
  inline double relative_Y_velocity_04() { return static_cast<double>(get(camera_FrontCamObject02_relative_Y_velocity_04)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_04() { return static_cast<double>(get(camera_FrontCamObject02_relative_X_acceleration_04)); };
  //* set interface
  inline void life_count_message_02(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_life_count_message_02); };
  // unit: [%]
  inline void quality_level_object_03(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_quality_level_object_03); };
  inline void life_count_object_03(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_life_count_object_03); };
  inline void moving_flag_03(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_moving_flag_03); };
  // unit: [m]
  inline void width_object_03(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_width_object_03); };
  inline void classification_object_03(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_classification_object_03); };
  // unit: [m]
  inline void relative_X_position_03(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_X_position_03); };
  // unit: [m]
  inline void relative_Y_position_03(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_Y_position_03); };
  // unit: [m/s]
  inline void relative_X_velocity_03(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_X_velocity_03); };
  // unit: [m/s]
  inline void relative_Y_velocity_03(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_Y_velocity_03); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_03(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_X_acceleration_03); };
  // unit: [%]
  inline void quality_level_object_04(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_quality_level_object_04); };
  inline void life_count_object_04(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_life_count_object_04); };
  inline void moving_flag_04(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_moving_flag_04); };
  // unit: [m]
  inline void width_object_04(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_width_object_04); };
  inline void classification_object_04(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject02_classification_object_04); };
  // unit: [m]
  inline void relative_X_position_04(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_X_position_04); };
  // unit: [m]
  inline void relative_Y_position_04(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_Y_position_04); };
  // unit: [m/s]
  inline void relative_X_velocity_04(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_X_velocity_04); };
  // unit: [m/s]
  inline void relative_Y_velocity_04(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_Y_velocity_04); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_04(const double& value) { set(static_cast<double>(value), camera_FrontCamObject02_relative_X_acceleration_04); };
};
#define camera_FrontCamObject03_life_count_message_03 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_quality_level_object_05 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject03_life_count_object_05 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_moving_flag_05 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_width_object_05 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject03_classification_object_05 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_relative_X_position_05 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject03_relative_Y_position_05 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject03_relative_X_velocity_05 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject03_relative_Y_velocity_05 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject03_relative_X_acceleration_05 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject03_quality_level_object_06 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject03_life_count_object_06 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_moving_flag_06 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_width_object_06 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject03_classification_object_06 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject03_relative_X_position_06 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject03_relative_Y_position_06 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject03_relative_X_velocity_06 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject03_relative_Y_velocity_06 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject03_relative_X_acceleration_06 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject03 : public MessageBase {
public:
  FrontCamObject03(uint8_t* arr = nullptr): MessageBase(&data[0], 386, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_03() { return static_cast<uint8_t>(get(camera_FrontCamObject03_life_count_message_03)); };
  // unit: [%]
  inline uint8_t quality_level_object_05() { return static_cast<uint8_t>(get(camera_FrontCamObject03_quality_level_object_05)); };
  inline uint8_t life_count_object_05() { return static_cast<uint8_t>(get(camera_FrontCamObject03_life_count_object_05)); };
  inline uint8_t moving_flag_05() { return static_cast<uint8_t>(get(camera_FrontCamObject03_moving_flag_05)); };
  // unit: [m]
  inline double width_object_05() { return static_cast<double>(get(camera_FrontCamObject03_width_object_05)); };
  inline uint8_t classification_object_05() { return static_cast<uint8_t>(get(camera_FrontCamObject03_classification_object_05)); };
  // unit: [m]
  inline double relative_X_position_05() { return static_cast<double>(get(camera_FrontCamObject03_relative_X_position_05)); };
  // unit: [m]
  inline double relative_Y_position_05() { return static_cast<double>(get(camera_FrontCamObject03_relative_Y_position_05)); };
  // unit: [m/s]
  inline double relative_X_velocity_05() { return static_cast<double>(get(camera_FrontCamObject03_relative_X_velocity_05)); };
  // unit: [m/s]
  inline double relative_Y_velocity_05() { return static_cast<double>(get(camera_FrontCamObject03_relative_Y_velocity_05)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_05() { return static_cast<double>(get(camera_FrontCamObject03_relative_X_acceleration_05)); };
  // unit: [%]
  inline uint8_t quality_level_object_06() { return static_cast<uint8_t>(get(camera_FrontCamObject03_quality_level_object_06)); };
  inline uint8_t life_count_object_06() { return static_cast<uint8_t>(get(camera_FrontCamObject03_life_count_object_06)); };
  inline uint8_t moving_flag_06() { return static_cast<uint8_t>(get(camera_FrontCamObject03_moving_flag_06)); };
  // unit: [m]
  inline double width_object_06() { return static_cast<double>(get(camera_FrontCamObject03_width_object_06)); };
  inline uint8_t classification_object_06() { return static_cast<uint8_t>(get(camera_FrontCamObject03_classification_object_06)); };
  // unit: [m]
  inline double relative_X_position_06() { return static_cast<double>(get(camera_FrontCamObject03_relative_X_position_06)); };
  // unit: [m]
  inline double relative_Y_position_06() { return static_cast<double>(get(camera_FrontCamObject03_relative_Y_position_06)); };
  // unit: [m/s]
  inline double relative_X_velocity_06() { return static_cast<double>(get(camera_FrontCamObject03_relative_X_velocity_06)); };
  // unit: [m/s]
  inline double relative_Y_velocity_06() { return static_cast<double>(get(camera_FrontCamObject03_relative_Y_velocity_06)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_06() { return static_cast<double>(get(camera_FrontCamObject03_relative_X_acceleration_06)); };
  //* set interface
  inline void life_count_message_03(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_life_count_message_03); };
  // unit: [%]
  inline void quality_level_object_05(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_quality_level_object_05); };
  inline void life_count_object_05(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_life_count_object_05); };
  inline void moving_flag_05(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_moving_flag_05); };
  // unit: [m]
  inline void width_object_05(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_width_object_05); };
  inline void classification_object_05(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_classification_object_05); };
  // unit: [m]
  inline void relative_X_position_05(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_X_position_05); };
  // unit: [m]
  inline void relative_Y_position_05(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_Y_position_05); };
  // unit: [m/s]
  inline void relative_X_velocity_05(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_X_velocity_05); };
  // unit: [m/s]
  inline void relative_Y_velocity_05(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_Y_velocity_05); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_05(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_X_acceleration_05); };
  // unit: [%]
  inline void quality_level_object_06(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_quality_level_object_06); };
  inline void life_count_object_06(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_life_count_object_06); };
  inline void moving_flag_06(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_moving_flag_06); };
  // unit: [m]
  inline void width_object_06(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_width_object_06); };
  inline void classification_object_06(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject03_classification_object_06); };
  // unit: [m]
  inline void relative_X_position_06(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_X_position_06); };
  // unit: [m]
  inline void relative_Y_position_06(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_Y_position_06); };
  // unit: [m/s]
  inline void relative_X_velocity_06(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_X_velocity_06); };
  // unit: [m/s]
  inline void relative_Y_velocity_06(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_Y_velocity_06); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_06(const double& value) { set(static_cast<double>(value), camera_FrontCamObject03_relative_X_acceleration_06); };
};
#define camera_FrontCamObject04_life_count_message_04 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_quality_level_object_07 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject04_life_count_object_07 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_moving_flag_07 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_width_object_07 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject04_classification_object_07 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_relative_X_position_07 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject04_relative_Y_position_07 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject04_relative_X_velocity_07 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject04_relative_Y_velocity_07 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject04_relative_X_acceleration_07 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject04_quality_level_object_08 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject04_life_count_object_08 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_moving_flag_08 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_width_object_08 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject04_classification_object_08 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject04_relative_X_position_08 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject04_relative_Y_position_08 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject04_relative_X_velocity_08 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject04_relative_Y_velocity_08 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject04_relative_X_acceleration_08 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject04 : public MessageBase {
public:
  FrontCamObject04(uint8_t* arr = nullptr): MessageBase(&data[0], 387, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_04() { return static_cast<uint8_t>(get(camera_FrontCamObject04_life_count_message_04)); };
  // unit: [%]
  inline uint8_t quality_level_object_07() { return static_cast<uint8_t>(get(camera_FrontCamObject04_quality_level_object_07)); };
  inline uint8_t life_count_object_07() { return static_cast<uint8_t>(get(camera_FrontCamObject04_life_count_object_07)); };
  inline uint8_t moving_flag_07() { return static_cast<uint8_t>(get(camera_FrontCamObject04_moving_flag_07)); };
  // unit: [m]
  inline double width_object_07() { return static_cast<double>(get(camera_FrontCamObject04_width_object_07)); };
  inline uint8_t classification_object_07() { return static_cast<uint8_t>(get(camera_FrontCamObject04_classification_object_07)); };
  // unit: [m]
  inline double relative_X_position_07() { return static_cast<double>(get(camera_FrontCamObject04_relative_X_position_07)); };
  // unit: [m]
  inline double relative_Y_position_07() { return static_cast<double>(get(camera_FrontCamObject04_relative_Y_position_07)); };
  // unit: [m/s]
  inline double relative_X_velocity_07() { return static_cast<double>(get(camera_FrontCamObject04_relative_X_velocity_07)); };
  // unit: [m/s]
  inline double relative_Y_velocity_07() { return static_cast<double>(get(camera_FrontCamObject04_relative_Y_velocity_07)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_07() { return static_cast<double>(get(camera_FrontCamObject04_relative_X_acceleration_07)); };
  // unit: [%]
  inline uint8_t quality_level_object_08() { return static_cast<uint8_t>(get(camera_FrontCamObject04_quality_level_object_08)); };
  inline uint8_t life_count_object_08() { return static_cast<uint8_t>(get(camera_FrontCamObject04_life_count_object_08)); };
  inline uint8_t moving_flag_08() { return static_cast<uint8_t>(get(camera_FrontCamObject04_moving_flag_08)); };
  // unit: [m]
  inline double width_object_08() { return static_cast<double>(get(camera_FrontCamObject04_width_object_08)); };
  inline uint8_t classification_object_08() { return static_cast<uint8_t>(get(camera_FrontCamObject04_classification_object_08)); };
  // unit: [m]
  inline double relative_X_position_08() { return static_cast<double>(get(camera_FrontCamObject04_relative_X_position_08)); };
  // unit: [m]
  inline double relative_Y_position_08() { return static_cast<double>(get(camera_FrontCamObject04_relative_Y_position_08)); };
  // unit: [m/s]
  inline double relative_X_velocity_08() { return static_cast<double>(get(camera_FrontCamObject04_relative_X_velocity_08)); };
  // unit: [m/s]
  inline double relative_Y_velocity_08() { return static_cast<double>(get(camera_FrontCamObject04_relative_Y_velocity_08)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_08() { return static_cast<double>(get(camera_FrontCamObject04_relative_X_acceleration_08)); };
  //* set interface
  inline void life_count_message_04(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_life_count_message_04); };
  // unit: [%]
  inline void quality_level_object_07(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_quality_level_object_07); };
  inline void life_count_object_07(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_life_count_object_07); };
  inline void moving_flag_07(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_moving_flag_07); };
  // unit: [m]
  inline void width_object_07(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_width_object_07); };
  inline void classification_object_07(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_classification_object_07); };
  // unit: [m]
  inline void relative_X_position_07(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_X_position_07); };
  // unit: [m]
  inline void relative_Y_position_07(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_Y_position_07); };
  // unit: [m/s]
  inline void relative_X_velocity_07(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_X_velocity_07); };
  // unit: [m/s]
  inline void relative_Y_velocity_07(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_Y_velocity_07); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_07(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_X_acceleration_07); };
  // unit: [%]
  inline void quality_level_object_08(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_quality_level_object_08); };
  inline void life_count_object_08(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_life_count_object_08); };
  inline void moving_flag_08(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_moving_flag_08); };
  // unit: [m]
  inline void width_object_08(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_width_object_08); };
  inline void classification_object_08(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject04_classification_object_08); };
  // unit: [m]
  inline void relative_X_position_08(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_X_position_08); };
  // unit: [m]
  inline void relative_Y_position_08(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_Y_position_08); };
  // unit: [m/s]
  inline void relative_X_velocity_08(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_X_velocity_08); };
  // unit: [m/s]
  inline void relative_Y_velocity_08(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_Y_velocity_08); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_08(const double& value) { set(static_cast<double>(value), camera_FrontCamObject04_relative_X_acceleration_08); };
};
#define camera_FrontCamObject05_life_count_message_05 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_quality_level_object_09 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject05_life_count_object_09 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_moving_flag_09 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_width_object_09 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject05_classification_object_09 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_relative_X_position_09 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject05_relative_Y_position_09 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject05_relative_X_velocity_09 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject05_relative_Y_velocity_09 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject05_relative_X_acceleration_09 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject05_quality_level_object_10 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject05_life_count_object_10 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_moving_flag_10 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_width_object_10 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject05_classification_object_10 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject05_relative_X_position_10 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject05_relative_Y_position_10 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject05_relative_X_velocity_10 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject05_relative_Y_velocity_10 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject05_relative_X_acceleration_10 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject05 : public MessageBase {
public:
  FrontCamObject05(uint8_t* arr = nullptr): MessageBase(&data[0], 388, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_05() { return static_cast<uint8_t>(get(camera_FrontCamObject05_life_count_message_05)); };
  // unit: [%]
  inline uint8_t quality_level_object_09() { return static_cast<uint8_t>(get(camera_FrontCamObject05_quality_level_object_09)); };
  inline uint8_t life_count_object_09() { return static_cast<uint8_t>(get(camera_FrontCamObject05_life_count_object_09)); };
  inline uint8_t moving_flag_09() { return static_cast<uint8_t>(get(camera_FrontCamObject05_moving_flag_09)); };
  // unit: [m]
  inline double width_object_09() { return static_cast<double>(get(camera_FrontCamObject05_width_object_09)); };
  inline uint8_t classification_object_09() { return static_cast<uint8_t>(get(camera_FrontCamObject05_classification_object_09)); };
  // unit: [m]
  inline double relative_X_position_09() { return static_cast<double>(get(camera_FrontCamObject05_relative_X_position_09)); };
  // unit: [m]
  inline double relative_Y_position_09() { return static_cast<double>(get(camera_FrontCamObject05_relative_Y_position_09)); };
  // unit: [m/s]
  inline double relative_X_velocity_09() { return static_cast<double>(get(camera_FrontCamObject05_relative_X_velocity_09)); };
  // unit: [m/s]
  inline double relative_Y_velocity_09() { return static_cast<double>(get(camera_FrontCamObject05_relative_Y_velocity_09)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_09() { return static_cast<double>(get(camera_FrontCamObject05_relative_X_acceleration_09)); };
  // unit: [%]
  inline uint8_t quality_level_object_10() { return static_cast<uint8_t>(get(camera_FrontCamObject05_quality_level_object_10)); };
  inline uint8_t life_count_object_10() { return static_cast<uint8_t>(get(camera_FrontCamObject05_life_count_object_10)); };
  inline uint8_t moving_flag_10() { return static_cast<uint8_t>(get(camera_FrontCamObject05_moving_flag_10)); };
  // unit: [m]
  inline double width_object_10() { return static_cast<double>(get(camera_FrontCamObject05_width_object_10)); };
  inline uint8_t classification_object_10() { return static_cast<uint8_t>(get(camera_FrontCamObject05_classification_object_10)); };
  // unit: [m]
  inline double relative_X_position_10() { return static_cast<double>(get(camera_FrontCamObject05_relative_X_position_10)); };
  // unit: [m]
  inline double relative_Y_position_10() { return static_cast<double>(get(camera_FrontCamObject05_relative_Y_position_10)); };
  // unit: [m/s]
  inline double relative_X_velocity_10() { return static_cast<double>(get(camera_FrontCamObject05_relative_X_velocity_10)); };
  // unit: [m/s]
  inline double relative_Y_velocity_10() { return static_cast<double>(get(camera_FrontCamObject05_relative_Y_velocity_10)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_10() { return static_cast<double>(get(camera_FrontCamObject05_relative_X_acceleration_10)); };
  //* set interface
  inline void life_count_message_05(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_life_count_message_05); };
  // unit: [%]
  inline void quality_level_object_09(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_quality_level_object_09); };
  inline void life_count_object_09(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_life_count_object_09); };
  inline void moving_flag_09(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_moving_flag_09); };
  // unit: [m]
  inline void width_object_09(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_width_object_09); };
  inline void classification_object_09(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_classification_object_09); };
  // unit: [m]
  inline void relative_X_position_09(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_X_position_09); };
  // unit: [m]
  inline void relative_Y_position_09(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_Y_position_09); };
  // unit: [m/s]
  inline void relative_X_velocity_09(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_X_velocity_09); };
  // unit: [m/s]
  inline void relative_Y_velocity_09(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_Y_velocity_09); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_09(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_X_acceleration_09); };
  // unit: [%]
  inline void quality_level_object_10(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_quality_level_object_10); };
  inline void life_count_object_10(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_life_count_object_10); };
  inline void moving_flag_10(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_moving_flag_10); };
  // unit: [m]
  inline void width_object_10(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_width_object_10); };
  inline void classification_object_10(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject05_classification_object_10); };
  // unit: [m]
  inline void relative_X_position_10(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_X_position_10); };
  // unit: [m]
  inline void relative_Y_position_10(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_Y_position_10); };
  // unit: [m/s]
  inline void relative_X_velocity_10(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_X_velocity_10); };
  // unit: [m/s]
  inline void relative_Y_velocity_10(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_Y_velocity_10); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_10(const double& value) { set(static_cast<double>(value), camera_FrontCamObject05_relative_X_acceleration_10); };
};
#define camera_FrontCamObject06_life_count_message_06 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_quality_level_object_11 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject06_life_count_object_11 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_moving_flag_11 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_width_object_11 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject06_classification_object_11 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_relative_X_position_11 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject06_relative_Y_position_11 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject06_relative_X_velocity_11 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject06_relative_Y_velocity_11 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject06_relative_X_acceleration_11 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject06_quality_level_object_12 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject06_life_count_object_12 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_moving_flag_12 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_width_object_12 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject06_classification_object_12 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject06_relative_X_position_12 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject06_relative_Y_position_12 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject06_relative_X_velocity_12 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject06_relative_Y_velocity_12 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject06_relative_X_acceleration_12 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject06 : public MessageBase {
public:
  FrontCamObject06(uint8_t* arr = nullptr): MessageBase(&data[0], 438, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_06() { return static_cast<uint8_t>(get(camera_FrontCamObject06_life_count_message_06)); };
  // unit: [%]
  inline uint8_t quality_level_object_11() { return static_cast<uint8_t>(get(camera_FrontCamObject06_quality_level_object_11)); };
  inline uint8_t life_count_object_11() { return static_cast<uint8_t>(get(camera_FrontCamObject06_life_count_object_11)); };
  inline uint8_t moving_flag_11() { return static_cast<uint8_t>(get(camera_FrontCamObject06_moving_flag_11)); };
  // unit: [m]
  inline double width_object_11() { return static_cast<double>(get(camera_FrontCamObject06_width_object_11)); };
  inline uint8_t classification_object_11() { return static_cast<uint8_t>(get(camera_FrontCamObject06_classification_object_11)); };
  // unit: [m]
  inline double relative_X_position_11() { return static_cast<double>(get(camera_FrontCamObject06_relative_X_position_11)); };
  // unit: [m]
  inline double relative_Y_position_11() { return static_cast<double>(get(camera_FrontCamObject06_relative_Y_position_11)); };
  // unit: [m/s]
  inline double relative_X_velocity_11() { return static_cast<double>(get(camera_FrontCamObject06_relative_X_velocity_11)); };
  // unit: [m/s]
  inline double relative_Y_velocity_11() { return static_cast<double>(get(camera_FrontCamObject06_relative_Y_velocity_11)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_11() { return static_cast<double>(get(camera_FrontCamObject06_relative_X_acceleration_11)); };
  // unit: [%]
  inline uint8_t quality_level_object_12() { return static_cast<uint8_t>(get(camera_FrontCamObject06_quality_level_object_12)); };
  inline uint8_t life_count_object_12() { return static_cast<uint8_t>(get(camera_FrontCamObject06_life_count_object_12)); };
  inline uint8_t moving_flag_12() { return static_cast<uint8_t>(get(camera_FrontCamObject06_moving_flag_12)); };
  // unit: [m]
  inline double width_object_12() { return static_cast<double>(get(camera_FrontCamObject06_width_object_12)); };
  inline uint8_t classification_object_12() { return static_cast<uint8_t>(get(camera_FrontCamObject06_classification_object_12)); };
  // unit: [m]
  inline double relative_X_position_12() { return static_cast<double>(get(camera_FrontCamObject06_relative_X_position_12)); };
  // unit: [m]
  inline double relative_Y_position_12() { return static_cast<double>(get(camera_FrontCamObject06_relative_Y_position_12)); };
  // unit: [m/s]
  inline double relative_X_velocity_12() { return static_cast<double>(get(camera_FrontCamObject06_relative_X_velocity_12)); };
  // unit: [m/s]
  inline double relative_Y_velocity_12() { return static_cast<double>(get(camera_FrontCamObject06_relative_Y_velocity_12)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_12() { return static_cast<double>(get(camera_FrontCamObject06_relative_X_acceleration_12)); };
  //* set interface
  inline void life_count_message_06(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_life_count_message_06); };
  // unit: [%]
  inline void quality_level_object_11(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_quality_level_object_11); };
  inline void life_count_object_11(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_life_count_object_11); };
  inline void moving_flag_11(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_moving_flag_11); };
  // unit: [m]
  inline void width_object_11(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_width_object_11); };
  inline void classification_object_11(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_classification_object_11); };
  // unit: [m]
  inline void relative_X_position_11(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_X_position_11); };
  // unit: [m]
  inline void relative_Y_position_11(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_Y_position_11); };
  // unit: [m/s]
  inline void relative_X_velocity_11(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_X_velocity_11); };
  // unit: [m/s]
  inline void relative_Y_velocity_11(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_Y_velocity_11); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_11(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_X_acceleration_11); };
  // unit: [%]
  inline void quality_level_object_12(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_quality_level_object_12); };
  inline void life_count_object_12(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_life_count_object_12); };
  inline void moving_flag_12(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_moving_flag_12); };
  // unit: [m]
  inline void width_object_12(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_width_object_12); };
  inline void classification_object_12(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject06_classification_object_12); };
  // unit: [m]
  inline void relative_X_position_12(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_X_position_12); };
  // unit: [m]
  inline void relative_Y_position_12(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_Y_position_12); };
  // unit: [m/s]
  inline void relative_X_velocity_12(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_X_velocity_12); };
  // unit: [m/s]
  inline void relative_Y_velocity_12(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_Y_velocity_12); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_12(const double& value) { set(static_cast<double>(value), camera_FrontCamObject06_relative_X_acceleration_12); };
};
#define camera_FrontCamObject07_life_count_message_07 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_quality_level_object_13 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject07_life_count_object_13 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_moving_flag_13 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_width_object_13 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject07_classification_object_13 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_relative_X_position_13 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject07_relative_Y_position_13 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject07_relative_X_velocity_13 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject07_relative_Y_velocity_13 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject07_relative_X_acceleration_13 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject07_quality_level_object_14 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject07_life_count_object_14 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_moving_flag_14 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_width_object_14 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject07_classification_object_14 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject07_relative_X_position_14 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject07_relative_Y_position_14 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject07_relative_X_velocity_14 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject07_relative_Y_velocity_14 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject07_relative_X_acceleration_14 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject07 : public MessageBase {
public:
  FrontCamObject07(uint8_t* arr = nullptr): MessageBase(&data[0], 439, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_07() { return static_cast<uint8_t>(get(camera_FrontCamObject07_life_count_message_07)); };
  // unit: [%]
  inline uint8_t quality_level_object_13() { return static_cast<uint8_t>(get(camera_FrontCamObject07_quality_level_object_13)); };
  inline uint8_t life_count_object_13() { return static_cast<uint8_t>(get(camera_FrontCamObject07_life_count_object_13)); };
  inline uint8_t moving_flag_13() { return static_cast<uint8_t>(get(camera_FrontCamObject07_moving_flag_13)); };
  // unit: [m]
  inline double width_object_13() { return static_cast<double>(get(camera_FrontCamObject07_width_object_13)); };
  inline uint8_t classification_object_13() { return static_cast<uint8_t>(get(camera_FrontCamObject07_classification_object_13)); };
  // unit: [m]
  inline double relative_X_position_13() { return static_cast<double>(get(camera_FrontCamObject07_relative_X_position_13)); };
  // unit: [m]
  inline double relative_Y_position_13() { return static_cast<double>(get(camera_FrontCamObject07_relative_Y_position_13)); };
  // unit: [m/s]
  inline double relative_X_velocity_13() { return static_cast<double>(get(camera_FrontCamObject07_relative_X_velocity_13)); };
  // unit: [m/s]
  inline double relative_Y_velocity_13() { return static_cast<double>(get(camera_FrontCamObject07_relative_Y_velocity_13)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_13() { return static_cast<double>(get(camera_FrontCamObject07_relative_X_acceleration_13)); };
  // unit: [%]
  inline uint8_t quality_level_object_14() { return static_cast<uint8_t>(get(camera_FrontCamObject07_quality_level_object_14)); };
  inline uint8_t life_count_object_14() { return static_cast<uint8_t>(get(camera_FrontCamObject07_life_count_object_14)); };
  inline uint8_t moving_flag_14() { return static_cast<uint8_t>(get(camera_FrontCamObject07_moving_flag_14)); };
  // unit: [m]
  inline double width_object_14() { return static_cast<double>(get(camera_FrontCamObject07_width_object_14)); };
  inline uint8_t classification_object_14() { return static_cast<uint8_t>(get(camera_FrontCamObject07_classification_object_14)); };
  // unit: [m]
  inline double relative_X_position_14() { return static_cast<double>(get(camera_FrontCamObject07_relative_X_position_14)); };
  // unit: [m]
  inline double relative_Y_position_14() { return static_cast<double>(get(camera_FrontCamObject07_relative_Y_position_14)); };
  // unit: [m/s]
  inline double relative_X_velocity_14() { return static_cast<double>(get(camera_FrontCamObject07_relative_X_velocity_14)); };
  // unit: [m/s]
  inline double relative_Y_velocity_14() { return static_cast<double>(get(camera_FrontCamObject07_relative_Y_velocity_14)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_14() { return static_cast<double>(get(camera_FrontCamObject07_relative_X_acceleration_14)); };
  //* set interface
  inline void life_count_message_07(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_life_count_message_07); };
  // unit: [%]
  inline void quality_level_object_13(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_quality_level_object_13); };
  inline void life_count_object_13(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_life_count_object_13); };
  inline void moving_flag_13(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_moving_flag_13); };
  // unit: [m]
  inline void width_object_13(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_width_object_13); };
  inline void classification_object_13(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_classification_object_13); };
  // unit: [m]
  inline void relative_X_position_13(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_X_position_13); };
  // unit: [m]
  inline void relative_Y_position_13(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_Y_position_13); };
  // unit: [m/s]
  inline void relative_X_velocity_13(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_X_velocity_13); };
  // unit: [m/s]
  inline void relative_Y_velocity_13(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_Y_velocity_13); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_13(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_X_acceleration_13); };
  // unit: [%]
  inline void quality_level_object_14(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_quality_level_object_14); };
  inline void life_count_object_14(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_life_count_object_14); };
  inline void moving_flag_14(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_moving_flag_14); };
  // unit: [m]
  inline void width_object_14(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_width_object_14); };
  inline void classification_object_14(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject07_classification_object_14); };
  // unit: [m]
  inline void relative_X_position_14(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_X_position_14); };
  // unit: [m]
  inline void relative_Y_position_14(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_Y_position_14); };
  // unit: [m/s]
  inline void relative_X_velocity_14(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_X_velocity_14); };
  // unit: [m/s]
  inline void relative_Y_velocity_14(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_Y_velocity_14); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_14(const double& value) { set(static_cast<double>(value), camera_FrontCamObject07_relative_X_acceleration_14); };
};
#define camera_FrontCamObject08_life_count_message_08 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_quality_level_object_15 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject08_life_count_object_15 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_moving_flag_15 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_width_object_15 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject08_classification_object_15 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_relative_X_position_15 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject08_relative_Y_position_15 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject08_relative_X_velocity_15 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject08_relative_Y_velocity_15 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject08_relative_X_acceleration_15 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject08_quality_level_object_16 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject08_life_count_object_16 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_moving_flag_16 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_width_object_16 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject08_classification_object_16 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject08_relative_X_position_16 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject08_relative_Y_position_16 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject08_relative_X_velocity_16 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject08_relative_Y_velocity_16 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject08_relative_X_acceleration_16 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject08 : public MessageBase {
public:
  FrontCamObject08(uint8_t* arr = nullptr): MessageBase(&data[0], 440, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_08() { return static_cast<uint8_t>(get(camera_FrontCamObject08_life_count_message_08)); };
  // unit: [%]
  inline uint8_t quality_level_object_15() { return static_cast<uint8_t>(get(camera_FrontCamObject08_quality_level_object_15)); };
  inline uint8_t life_count_object_15() { return static_cast<uint8_t>(get(camera_FrontCamObject08_life_count_object_15)); };
  inline uint8_t moving_flag_15() { return static_cast<uint8_t>(get(camera_FrontCamObject08_moving_flag_15)); };
  // unit: [m]
  inline double width_object_15() { return static_cast<double>(get(camera_FrontCamObject08_width_object_15)); };
  inline uint8_t classification_object_15() { return static_cast<uint8_t>(get(camera_FrontCamObject08_classification_object_15)); };
  // unit: [m]
  inline double relative_X_position_15() { return static_cast<double>(get(camera_FrontCamObject08_relative_X_position_15)); };
  // unit: [m]
  inline double relative_Y_position_15() { return static_cast<double>(get(camera_FrontCamObject08_relative_Y_position_15)); };
  // unit: [m/s]
  inline double relative_X_velocity_15() { return static_cast<double>(get(camera_FrontCamObject08_relative_X_velocity_15)); };
  // unit: [m/s]
  inline double relative_Y_velocity_15() { return static_cast<double>(get(camera_FrontCamObject08_relative_Y_velocity_15)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_15() { return static_cast<double>(get(camera_FrontCamObject08_relative_X_acceleration_15)); };
  // unit: [%]
  inline uint8_t quality_level_object_16() { return static_cast<uint8_t>(get(camera_FrontCamObject08_quality_level_object_16)); };
  inline uint8_t life_count_object_16() { return static_cast<uint8_t>(get(camera_FrontCamObject08_life_count_object_16)); };
  inline uint8_t moving_flag_16() { return static_cast<uint8_t>(get(camera_FrontCamObject08_moving_flag_16)); };
  // unit: [m]
  inline double width_object_16() { return static_cast<double>(get(camera_FrontCamObject08_width_object_16)); };
  inline uint8_t classification_object_16() { return static_cast<uint8_t>(get(camera_FrontCamObject08_classification_object_16)); };
  // unit: [m]
  inline double relative_X_position_16() { return static_cast<double>(get(camera_FrontCamObject08_relative_X_position_16)); };
  // unit: [m]
  inline double relative_Y_position_16() { return static_cast<double>(get(camera_FrontCamObject08_relative_Y_position_16)); };
  // unit: [m/s]
  inline double relative_X_velocity_16() { return static_cast<double>(get(camera_FrontCamObject08_relative_X_velocity_16)); };
  // unit: [m/s]
  inline double relative_Y_velocity_16() { return static_cast<double>(get(camera_FrontCamObject08_relative_Y_velocity_16)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_16() { return static_cast<double>(get(camera_FrontCamObject08_relative_X_acceleration_16)); };
  //* set interface
  inline void life_count_message_08(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_life_count_message_08); };
  // unit: [%]
  inline void quality_level_object_15(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_quality_level_object_15); };
  inline void life_count_object_15(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_life_count_object_15); };
  inline void moving_flag_15(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_moving_flag_15); };
  // unit: [m]
  inline void width_object_15(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_width_object_15); };
  inline void classification_object_15(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_classification_object_15); };
  // unit: [m]
  inline void relative_X_position_15(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_X_position_15); };
  // unit: [m]
  inline void relative_Y_position_15(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_Y_position_15); };
  // unit: [m/s]
  inline void relative_X_velocity_15(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_X_velocity_15); };
  // unit: [m/s]
  inline void relative_Y_velocity_15(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_Y_velocity_15); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_15(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_X_acceleration_15); };
  // unit: [%]
  inline void quality_level_object_16(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_quality_level_object_16); };
  inline void life_count_object_16(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_life_count_object_16); };
  inline void moving_flag_16(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_moving_flag_16); };
  // unit: [m]
  inline void width_object_16(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_width_object_16); };
  inline void classification_object_16(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject08_classification_object_16); };
  // unit: [m]
  inline void relative_X_position_16(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_X_position_16); };
  // unit: [m]
  inline void relative_Y_position_16(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_Y_position_16); };
  // unit: [m/s]
  inline void relative_X_velocity_16(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_X_velocity_16); };
  // unit: [m/s]
  inline void relative_Y_velocity_16(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_Y_velocity_16); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_16(const double& value) { set(static_cast<double>(value), camera_FrontCamObject08_relative_X_acceleration_16); };
};
#define camera_FrontCamObject09_life_count_message_09 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_quality_level_object_17 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject09_life_count_object_17 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_moving_flag_17 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_width_object_17 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject09_classification_object_17 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_relative_X_position_17 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject09_relative_Y_position_17 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject09_relative_X_velocity_17 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject09_relative_Y_velocity_17 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject09_relative_X_acceleration_17 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject09_quality_level_object_18 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject09_life_count_object_18 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_moving_flag_18 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_width_object_18 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject09_classification_object_18 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject09_relative_X_position_18 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject09_relative_Y_position_18 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject09_relative_X_velocity_18 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject09_relative_Y_velocity_18 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject09_relative_X_acceleration_18 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject09 : public MessageBase {
public:
  FrontCamObject09(uint8_t* arr = nullptr): MessageBase(&data[0], 441, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_09() { return static_cast<uint8_t>(get(camera_FrontCamObject09_life_count_message_09)); };
  // unit: [%]
  inline uint8_t quality_level_object_17() { return static_cast<uint8_t>(get(camera_FrontCamObject09_quality_level_object_17)); };
  inline uint8_t life_count_object_17() { return static_cast<uint8_t>(get(camera_FrontCamObject09_life_count_object_17)); };
  inline uint8_t moving_flag_17() { return static_cast<uint8_t>(get(camera_FrontCamObject09_moving_flag_17)); };
  // unit: [m]
  inline double width_object_17() { return static_cast<double>(get(camera_FrontCamObject09_width_object_17)); };
  inline uint8_t classification_object_17() { return static_cast<uint8_t>(get(camera_FrontCamObject09_classification_object_17)); };
  // unit: [m]
  inline double relative_X_position_17() { return static_cast<double>(get(camera_FrontCamObject09_relative_X_position_17)); };
  // unit: [m]
  inline double relative_Y_position_17() { return static_cast<double>(get(camera_FrontCamObject09_relative_Y_position_17)); };
  // unit: [m/s]
  inline double relative_X_velocity_17() { return static_cast<double>(get(camera_FrontCamObject09_relative_X_velocity_17)); };
  // unit: [m/s]
  inline double relative_Y_velocity_17() { return static_cast<double>(get(camera_FrontCamObject09_relative_Y_velocity_17)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_17() { return static_cast<double>(get(camera_FrontCamObject09_relative_X_acceleration_17)); };
  // unit: [%]
  inline uint8_t quality_level_object_18() { return static_cast<uint8_t>(get(camera_FrontCamObject09_quality_level_object_18)); };
  inline uint8_t life_count_object_18() { return static_cast<uint8_t>(get(camera_FrontCamObject09_life_count_object_18)); };
  inline uint8_t moving_flag_18() { return static_cast<uint8_t>(get(camera_FrontCamObject09_moving_flag_18)); };
  // unit: [m]
  inline double width_object_18() { return static_cast<double>(get(camera_FrontCamObject09_width_object_18)); };
  inline uint8_t classification_object_18() { return static_cast<uint8_t>(get(camera_FrontCamObject09_classification_object_18)); };
  // unit: [m]
  inline double relative_X_position_18() { return static_cast<double>(get(camera_FrontCamObject09_relative_X_position_18)); };
  // unit: [m]
  inline double relative_Y_position_18() { return static_cast<double>(get(camera_FrontCamObject09_relative_Y_position_18)); };
  // unit: [m/s]
  inline double relative_X_velocity_18() { return static_cast<double>(get(camera_FrontCamObject09_relative_X_velocity_18)); };
  // unit: [m/s]
  inline double relative_Y_velocity_18() { return static_cast<double>(get(camera_FrontCamObject09_relative_Y_velocity_18)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_18() { return static_cast<double>(get(camera_FrontCamObject09_relative_X_acceleration_18)); };
  //* set interface
  inline void life_count_message_09(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_life_count_message_09); };
  // unit: [%]
  inline void quality_level_object_17(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_quality_level_object_17); };
  inline void life_count_object_17(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_life_count_object_17); };
  inline void moving_flag_17(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_moving_flag_17); };
  // unit: [m]
  inline void width_object_17(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_width_object_17); };
  inline void classification_object_17(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_classification_object_17); };
  // unit: [m]
  inline void relative_X_position_17(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_X_position_17); };
  // unit: [m]
  inline void relative_Y_position_17(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_Y_position_17); };
  // unit: [m/s]
  inline void relative_X_velocity_17(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_X_velocity_17); };
  // unit: [m/s]
  inline void relative_Y_velocity_17(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_Y_velocity_17); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_17(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_X_acceleration_17); };
  // unit: [%]
  inline void quality_level_object_18(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_quality_level_object_18); };
  inline void life_count_object_18(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_life_count_object_18); };
  inline void moving_flag_18(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_moving_flag_18); };
  // unit: [m]
  inline void width_object_18(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_width_object_18); };
  inline void classification_object_18(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject09_classification_object_18); };
  // unit: [m]
  inline void relative_X_position_18(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_X_position_18); };
  // unit: [m]
  inline void relative_Y_position_18(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_Y_position_18); };
  // unit: [m/s]
  inline void relative_X_velocity_18(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_X_velocity_18); };
  // unit: [m/s]
  inline void relative_Y_velocity_18(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_Y_velocity_18); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_18(const double& value) { set(static_cast<double>(value), camera_FrontCamObject09_relative_X_acceleration_18); };
};
#define camera_FrontCamObject10_life_count_message_10 {ValueType::Unsigned, 1, 0, 16, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_quality_level_object_19 {ValueType::Unsigned, 1, 0, 24, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject10_life_count_object_19 {ValueType::Unsigned, 1, 0, 32, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_moving_flag_19 {ValueType::Unsigned, 1, 0, 40, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_width_object_19 {ValueType::Unsigned, 0.05, 0, 52, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject10_classification_object_19 {ValueType::Unsigned, 1, 0, 60, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_relative_X_position_19 {ValueType::Unsigned, 0.05, 0, 64, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject10_relative_Y_position_19 {ValueType::Unsigned, 0.05, -102.4, 78, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject10_relative_X_velocity_19 {ValueType::Unsigned, 0.05, -100, 91, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject10_relative_Y_velocity_19 {ValueType::Unsigned, 0.05, -25, 104, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject10_relative_X_acceleration_19 {ValueType::Signed, 0.05, 0, 115, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
#define camera_FrontCamObject10_quality_level_object_20 {ValueType::Unsigned, 1, 0, 152, 7, ByteOrder::LittleEndian} // unit: [%]
#define camera_FrontCamObject10_life_count_object_20 {ValueType::Unsigned, 1, 0, 160, 8, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_moving_flag_20 {ValueType::Unsigned, 1, 0, 168, 4, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_width_object_20 {ValueType::Unsigned, 0.05, 0, 180, 7, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject10_classification_object_20 {ValueType::Unsigned, 1, 0, 188, 3, ByteOrder::LittleEndian} // unit: []
#define camera_FrontCamObject10_relative_X_position_20 {ValueType::Unsigned, 0.05, 0, 192, 13, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject10_relative_Y_position_20 {ValueType::Unsigned, 0.05, -102.4, 206, 12, ByteOrder::LittleEndian} // unit: [m]
#define camera_FrontCamObject10_relative_X_velocity_20 {ValueType::Unsigned, 0.05, -100, 219, 12, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject10_relative_Y_velocity_20 {ValueType::Unsigned, 0.05, -25, 232, 10, ByteOrder::LittleEndian} // unit: [m/s]
#define camera_FrontCamObject10_relative_X_acceleration_20 {ValueType::Signed, 0.05, 0, 243, 9, ByteOrder::LittleEndian} // unit: [m/s^2]
class FrontCamObject10 : public MessageBase {
public:
  FrontCamObject10(uint8_t* arr = nullptr): MessageBase(&data[0], 507, 32, true, false) { arr == nullptr ? memset(&data[0], 0, length) : memcpy(&data[0], arr, length); };
  void checkMemoryPointer(void) { if(_data != &data[0]) _data = &data[0]; };
  uint8_t data[32];
  //* value table
  //* get interface
  inline uint8_t life_count_message_10() { return static_cast<uint8_t>(get(camera_FrontCamObject10_life_count_message_10)); };
  // unit: [%]
  inline uint8_t quality_level_object_19() { return static_cast<uint8_t>(get(camera_FrontCamObject10_quality_level_object_19)); };
  inline uint8_t life_count_object_19() { return static_cast<uint8_t>(get(camera_FrontCamObject10_life_count_object_19)); };
  inline uint8_t moving_flag_19() { return static_cast<uint8_t>(get(camera_FrontCamObject10_moving_flag_19)); };
  // unit: [m]
  inline double width_object_19() { return static_cast<double>(get(camera_FrontCamObject10_width_object_19)); };
  inline uint8_t classification_object_19() { return static_cast<uint8_t>(get(camera_FrontCamObject10_classification_object_19)); };
  // unit: [m]
  inline double relative_X_position_19() { return static_cast<double>(get(camera_FrontCamObject10_relative_X_position_19)); };
  // unit: [m]
  inline double relative_Y_position_19() { return static_cast<double>(get(camera_FrontCamObject10_relative_Y_position_19)); };
  // unit: [m/s]
  inline double relative_X_velocity_19() { return static_cast<double>(get(camera_FrontCamObject10_relative_X_velocity_19)); };
  // unit: [m/s]
  inline double relative_Y_velocity_19() { return static_cast<double>(get(camera_FrontCamObject10_relative_Y_velocity_19)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_19() { return static_cast<double>(get(camera_FrontCamObject10_relative_X_acceleration_19)); };
  // unit: [%]
  inline uint8_t quality_level_object_20() { return static_cast<uint8_t>(get(camera_FrontCamObject10_quality_level_object_20)); };
  inline uint8_t life_count_object_20() { return static_cast<uint8_t>(get(camera_FrontCamObject10_life_count_object_20)); };
  inline uint8_t moving_flag_20() { return static_cast<uint8_t>(get(camera_FrontCamObject10_moving_flag_20)); };
  // unit: [m]
  inline double width_object_20() { return static_cast<double>(get(camera_FrontCamObject10_width_object_20)); };
  inline uint8_t classification_object_20() { return static_cast<uint8_t>(get(camera_FrontCamObject10_classification_object_20)); };
  // unit: [m]
  inline double relative_X_position_20() { return static_cast<double>(get(camera_FrontCamObject10_relative_X_position_20)); };
  // unit: [m]
  inline double relative_Y_position_20() { return static_cast<double>(get(camera_FrontCamObject10_relative_Y_position_20)); };
  // unit: [m/s]
  inline double relative_X_velocity_20() { return static_cast<double>(get(camera_FrontCamObject10_relative_X_velocity_20)); };
  // unit: [m/s]
  inline double relative_Y_velocity_20() { return static_cast<double>(get(camera_FrontCamObject10_relative_Y_velocity_20)); };
  // unit: [m/s^2]
  inline double relative_X_acceleration_20() { return static_cast<double>(get(camera_FrontCamObject10_relative_X_acceleration_20)); };
  //* set interface
  inline void life_count_message_10(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_life_count_message_10); };
  // unit: [%]
  inline void quality_level_object_19(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_quality_level_object_19); };
  inline void life_count_object_19(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_life_count_object_19); };
  inline void moving_flag_19(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_moving_flag_19); };
  // unit: [m]
  inline void width_object_19(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_width_object_19); };
  inline void classification_object_19(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_classification_object_19); };
  // unit: [m]
  inline void relative_X_position_19(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_X_position_19); };
  // unit: [m]
  inline void relative_Y_position_19(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_Y_position_19); };
  // unit: [m/s]
  inline void relative_X_velocity_19(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_X_velocity_19); };
  // unit: [m/s]
  inline void relative_Y_velocity_19(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_Y_velocity_19); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_19(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_X_acceleration_19); };
  // unit: [%]
  inline void quality_level_object_20(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_quality_level_object_20); };
  inline void life_count_object_20(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_life_count_object_20); };
  inline void moving_flag_20(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_moving_flag_20); };
  // unit: [m]
  inline void width_object_20(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_width_object_20); };
  inline void classification_object_20(const uint8_t& value) { set(static_cast<double>(value), camera_FrontCamObject10_classification_object_20); };
  // unit: [m]
  inline void relative_X_position_20(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_X_position_20); };
  // unit: [m]
  inline void relative_Y_position_20(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_Y_position_20); };
  // unit: [m/s]
  inline void relative_X_velocity_20(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_X_velocity_20); };
  // unit: [m/s]
  inline void relative_Y_velocity_20(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_Y_velocity_20); };
  // unit: [m/s^2]
  inline void relative_X_acceleration_20(const double& value) { set(static_cast<double>(value), camera_FrontCamObject10_relative_X_acceleration_20); };
};

} // namespace camera
} // namespace dbc
#endif // CAMERA_DBC_HPP__
