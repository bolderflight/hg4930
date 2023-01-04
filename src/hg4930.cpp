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

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "hg4930.h"  // NOLINT

namespace bfs {

void Hg4930::Config(HardwareSerial *bus) {
  uart_ = bus;
}

void Hg4930::Begin() {
  uart_->begin(BAUD_);
}

bool Hg4930::Read() {
  new_data_ = new_imu_data_ = new_ins_data_ = false;
  do {
    new_data_ = Parse();
  } while (uart_->available());
  return new_data_;
}

bool Hg4930::Parse() {
  while (uart_->available()) {
    c_ = uart_->read();
    /* Look for the header */
    if (state_ == 0) {
      if (c_ == HEADER_) {
        imu_msg_ = false;
        ins_msg_ = false;
        buf_[state_++] = c_;
      }
    /* Look for the message type */
    } else if (state_ == 1) {
      if (c_ == IMU_MSG_) {
        imu_msg_ = true;
        buf_[state_++] = c_;
      } else if (c_ == INS_MSG_) {
        ins_msg_ = true;
        buf_[state_++] = c_;
      } else {
        state_ = 0;
      }
    /* Parse the body */
    } else {
      if (imu_msg_) {
        /* Grab the payload */
        if (state_ < IMU_MSG_LEN_) {
          buf_[state_++] = c_;
          /* Check the checksum */
          if (state_ == IMU_MSG_LEN_) {
            /* Header */
            buf16_[0] = (static_cast<uint16_t>(buf_[1]) << 8) | buf_[0];
            /* Angular rate data */
            buf16_[1] = (static_cast<uint16_t>(buf_[3]) << 8) | buf_[2];
            buf16_[2] = (static_cast<uint16_t>(buf_[5]) << 8) | buf_[4];
            buf16_[3] = (static_cast<uint16_t>(buf_[7]) << 8) | buf_[6];
            /* Linear accel data */
            buf16_[4] = (static_cast<uint16_t>(buf_[9]) << 8) | buf_[8];
            buf16_[5] = (static_cast<uint16_t>(buf_[11]) << 8) | buf_[10];
            buf16_[6] = (static_cast<uint16_t>(buf_[13]) << 8) | buf_[12];
            /* Status */
            buf16_[7] = (static_cast<uint16_t>(buf_[15]) << 8) | buf_[14];
            buf16_[8] = (static_cast<uint16_t>(buf_[17]) << 8) | buf_[16];
            /* Checksum */
            buf16_[9] = (static_cast<uint16_t>(buf_[19]) << 8) | buf_[18];
            if (buf16_[9] == Checksum(buf16_, 9)) {
              gyro_radps_[0] = -1.0f * static_cast<int16_t>(buf16_[2]) *
                              GYRO_SCALE_;
              gyro_radps_[1] = static_cast<int16_t>(buf16_[3]) * GYRO_SCALE_;
              gyro_radps_[2] = -1.0f * static_cast<int16_t>(buf16_[1]) *
                              GYRO_SCALE_;
              accel_mps2_[0] = -1.0f * static_cast<int16_t>(buf16_[5]) *
                              ACCEL_SCALE_;
              accel_mps2_[1] = static_cast<int16_t>(buf16_[6]) * ACCEL_SCALE_;
              accel_mps2_[2] = -1.0f * static_cast<int16_t>(buf16_[4]) *
                              ACCEL_SCALE_;
              status1_ = buf16_[7];
              status2_ = buf16_[8];
              if (status2_ & 0x8000) {
                die_temp_c_ = static_cast<float>(status2_ & 0xFF);
              }
              state_ = 0;
              new_imu_data_ = true;
              return true;
            } else {
              state_ = 0;
            }
          }
        } else {
          state_ = 0;
        }
      } else {
        /* Grab the payload */
        if (state_ < INS_MSG_LEN_) { 
          buf_[state_++] = c_;
          /* Check the checksum */
          if (state_ == INS_MSG_LEN_) {
            /* Header */
            buf16_[0] = (static_cast<uint16_t>(buf_[1]) << 8) | buf_[0];
            /* Angular rate data */
            buf16_[1] = (static_cast<uint16_t>(buf_[3]) << 8) | buf_[2];
            buf16_[2] = (static_cast<uint16_t>(buf_[5]) << 8) | buf_[4];
            buf16_[3] = (static_cast<uint16_t>(buf_[7]) << 8) | buf_[6];
            /* Linear accel data */
            buf16_[4] = (static_cast<uint16_t>(buf_[9]) << 8) | buf_[8];
            buf16_[5] = (static_cast<uint16_t>(buf_[11]) << 8) | buf_[10];
            buf16_[6] = (static_cast<uint16_t>(buf_[13]) << 8) | buf_[12];
            /* Status */
            buf16_[7] = (static_cast<uint16_t>(buf_[15]) << 8) | buf_[14];
            buf16_[8] = (static_cast<uint16_t>(buf_[17]) << 8) | buf_[16];
            /* Delta angle */
            buf16_[9] = (static_cast<uint16_t>(buf_[19]) << 8) | buf_[18];
            buf16_[10] = (static_cast<uint16_t>(buf_[21]) << 8) | buf_[20];
            buf16_[11] = (static_cast<uint16_t>(buf_[23]) << 8) | buf_[22];
            buf16_[12] = (static_cast<uint16_t>(buf_[25]) << 8) | buf_[24];
            buf16_[13] = (static_cast<uint16_t>(buf_[27]) << 8) | buf_[26];
            buf16_[14] = (static_cast<uint16_t>(buf_[29]) << 8) | buf_[28];
            /* Delta velocity */
            buf16_[15] = (static_cast<uint16_t>(buf_[31]) << 8) | buf_[30];
            buf16_[16] = (static_cast<uint16_t>(buf_[33]) << 8) | buf_[32];
            buf16_[17] = (static_cast<uint16_t>(buf_[35]) << 8) | buf_[34];
            buf16_[18] = (static_cast<uint16_t>(buf_[37]) << 8) | buf_[36];
            buf16_[19] = (static_cast<uint16_t>(buf_[39]) << 8) | buf_[38];
            buf16_[20] = (static_cast<uint16_t>(buf_[41]) << 8) | buf_[40];
            /* Checksum */
            buf16_[21] = (static_cast<uint16_t>(buf_[43]) << 8) | buf_[42];
            if (buf16_[21] == Checksum(buf16_, 21)) {
              gyro_radps_[0] = -1.0f * static_cast<int16_t>(buf16_[2]) *
                               GYRO_SCALE_;
              gyro_radps_[1] = static_cast<int16_t>(buf16_[3]) * GYRO_SCALE_;
              gyro_radps_[2] = -1.0f * static_cast<int16_t>(buf16_[1]) *
                               GYRO_SCALE_;
              accel_mps2_[0] = -1.0f * static_cast<int16_t>(buf16_[5]) *
                               ACCEL_SCALE_;
              accel_mps2_[1] = static_cast<int16_t>(buf16_[6]) * ACCEL_SCALE_;
              accel_mps2_[2] = -1.0f * static_cast<int16_t>(buf16_[4]) *
                               ACCEL_SCALE_;
              status1_ = buf16_[7];
              status2_ = buf16_[8];
              if (status2_ & 0x8000) {
                die_temp_c_ = static_cast<float>(status2_ & 0xFF);
              }
              delta_angle_rad_[2] = -1.0 * static_cast<int32_t>(
                                    (static_cast<uint32_t>(buf16_[10]) << 16) |
                                    buf16_[9]) * DELTA_ANG_SCALE_;
              delta_angle_rad_[0] = -1.0 * static_cast<int32_t>(
                                    (static_cast<uint32_t>(buf16_[12]) << 16) |
                                    buf16_[11]) * DELTA_ANG_SCALE_;
              delta_angle_rad_[1] = static_cast<int32_t>(
                                    (static_cast<uint32_t>(buf16_[14]) << 16) |
                                    buf16_[13]) * DELTA_ANG_SCALE_;
              delta_vel_mps_[2] = -1.0 * static_cast<int32_t>(
                                  (static_cast<uint32_t>(buf16_[16]) << 16) |
                                  buf16_[15]) * DELTA_VEL_SCALE_;
              delta_vel_mps_[0] = -1.0 * static_cast<int32_t>(
                                  (static_cast<uint32_t>(buf16_[18]) << 16) |
                                  buf16_[17]) * DELTA_VEL_SCALE_;
              delta_vel_mps_[1] = static_cast<int32_t>(
                                  (static_cast<uint32_t>(buf16_[20]) << 16) |
                                  buf16_[19]) * DELTA_VEL_SCALE_;
              state_ = 0;
              new_ins_data_ = true;
              new_imu_data_ = true;
              return true;
            } else {
              state_ = 0;
            }
          }
        } else {
          state_ = 0;
        }
      }
    }
  }
  return false;
}

uint16_t Hg4930::Checksum(uint16_t * data, const int8_t len) {
  uint16_t chk = 0;
  for (int8_t i = 0; i < len; i++) {
    chk += data[i];
  }
  return chk;
}

}  // namespace bfs
