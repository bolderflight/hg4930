/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#include "hg4930.h"

/* HG4930 object on Serial1 */
bfs::Hg4930 imu(&Serial1);

int main() {
  /* Serial to print data */
  Serial.begin(115200);
  while (!Serial) {}
  /* Initialize IMU */
  imu.Begin();
  while (1) {
    /* Read data */
    if (imu.Read()) {
      Serial.print(imu.accel_x_mps2());
      Serial.print("\t");
      Serial.print(imu.accel_y_mps2());
      Serial.print("\t");
      Serial.print(imu.accel_z_mps2());
      Serial.print("\t");
      Serial.print(imu.gyro_x_radps());
      Serial.print("\t");
      Serial.print(imu.gyro_y_radps());
      Serial.print("\t");
      Serial.print(imu.gyro_z_radps());
      Serial.print("\t");
      Serial.print(imu.delta_angle_x_rad());
      Serial.print("\t");
      Serial.print(imu.delta_angle_y_rad());
      Serial.print("\t");
      Serial.print(imu.delta_angle_z_rad());
      Serial.print("\t");
      Serial.print(imu.delta_vel_x_mps());
      Serial.print("\t");
      Serial.print(imu.delta_vel_y_mps());
      Serial.print("\t");
      Serial.print(imu.delta_vel_z_mps());
      Serial.print("\t");
      Serial.print(imu.die_temp_c());
      Serial.print("\n");
    }
  }
}
