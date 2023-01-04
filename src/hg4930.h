/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2023 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef HG4930_SRC_HG4930_H_  // NOLINT
#define HG4930_SRC_HG4930_H_

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include <cmath>
#include "core/core.h"
#endif

namespace bfs {

class Hg4930 {
 public:
  Hg4930() {}
  explicit Hg4930(HardwareSerial *bus) : uart_(bus) {}
  void Config(HardwareSerial *bus);
  void Begin();
  bool Read();
  inline bool new_imu_data() const {return new_imu_data_;}
  inline bool new_ins_data() const {return new_ins_data_;}
  inline bool imu_healthy() const {return status1_ & 0x10;}
  inline bool gyro_healthy() const {return status1_ & 0x20;}
  inline bool accel_healthy() const {return status1_ & 0x40;}
  inline bool gyro_ok() const {return status1_ & 0x80;}
  inline bool gyro_x_valid() const {return status1_ & 0x100;}
  inline bool gyro_y_valid() const {return status1_ & 0x200;}
  inline bool gyro_z_valid() const {return status1_ & 0x400;}
  inline bool imu_ok() const {return status1_ & 0x8000;}
  inline bool gyro_health_1() const {return status2_ & 0x100;}
  inline bool start_data_flag() const {return status2_ & 0x200;}
  inline bool process_test() const {return status2_ & 0x400;}
  inline bool memory_test() const {return status2_ & 0x800;}
  inline bool electronics_test() const {return status2_ & 0x1000;}
  inline bool gyro_health_2() const {return status2_ & 0x2000;}
  inline bool accel_health() const {return status2_ & 0x4000;}
  inline float accel_x_mps2() const {return accel_mps2_[0];}
  inline float accel_y_mps2() const {return accel_mps2_[1];}
  inline float accel_z_mps2() const {return accel_mps2_[2];}
  inline float gyro_x_radps() const {return gyro_radps_[0];}
  inline float gyro_y_radps() const {return gyro_radps_[1];}
  inline float gyro_z_radps() const {return gyro_radps_[2];}
  inline double delta_angle_x_rad() const {return delta_angle_rad_[0];}
  inline double delta_angle_y_rad() const {return delta_angle_rad_[1];}
  inline double delta_angle_z_rad() const {return delta_angle_rad_[2];}
  inline double delta_vel_x_mps() const {return delta_vel_mps_[0];}
  inline double delta_vel_y_mps() const {return delta_vel_mps_[1];}
  inline double delta_vel_z_mps() const {return delta_vel_mps_[2];}
  inline float die_temp_c() const {return die_temp_c_;}

 private:
  /* comm */
  HardwareSerial *uart_;
  static constexpr int32_t BAUD_ = 1000000;
  /* parsing */
  uint8_t c_;
  bool new_data_, new_imu_data_, new_ins_data_;
  uint8_t state_ = 0;
  bool imu_msg_ = false;
  bool ins_msg_ = false;
  static constexpr uint8_t HEADER_ = 0x0E;
  static constexpr uint8_t IMU_MSG_ = 0x01;
  static constexpr uint8_t INS_MSG_ = 0x02;
  static constexpr int8_t IMU_MSG_LEN_ = 20;
  static constexpr int8_t INS_MSG_LEN_ = 44;
  uint8_t buf_[INS_MSG_LEN_];
  uint16_t buf16_[INS_MSG_LEN_ / 2];
  uint16_t chk_;
  /* data */
  uint16_t status1_;
  uint16_t status2_;
  static constexpr float GYRO_SCALE_ = 0.00057220458984375f;
  static constexpr float ACCEL_SCALE_ = 0.011162109375f;
  static constexpr double DELTA_ANG_SCALE_ = pow(2.0, -33.0);
  static constexpr double DELTA_VEL_SCALE_ = pow(2.0, -27.0);
  float die_temp_c_;
  float accel_mps2_[3];
  float gyro_radps_[3];
  double delta_angle_rad_[3];
  double delta_vel_mps_[3];
  /* utility functions */
  bool Parse();
  uint16_t Checksum(uint16_t * data, const int8_t len);
};

}  // namespace bfs

#endif  // HG4930_SRC_HG4930_H_ NOLINT
