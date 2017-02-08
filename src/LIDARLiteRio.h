/*------------------------------------------------------------------------------

  LIDARLite National Instruments RoboRio Library
  LIDARLite.h

  This library provides quick access to all the basic functions of LIDAR-Lite
  via the National Instruments RoboRio I2c interface.

  Copyright (c) 2016 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  -----------------------------------------------------------------------------
  
  Copyright (c) FIRST 2008-2017. All Rights Reserved.
  Open Source Software - may be modified and shared by FRC teams. The code
  must be accompanied by the FIRST BSD license file in the root directory of
  the project.

------------------------------------------------------------------------------*/

#ifndef LIDARLiteRio_H
#define LIDARLiteRio_H

#include "I2C.h"
#include "LiveWindow/LiveWindowSendable.h"

typedef unsigned char byte;    // 8-bit unsigned entity.

namespace frc {

/**
 * LidarLite V3 on I2C.
 *
 * This class allows access to the Garmin LidarLite V3 sensor via the I2C bus
 * This class assumes the default (not alternate) sensor address of 0x62 (7-bit
 * address).
 */

class LIDARLite_I2C : public LiveWindowSendable {

 public:
  static const int kAddress = 0x62;

 protected:
  static const int kStatusRegister = 0x01;
  static const int kDistanceRegister = 0x8f;

 public:
  LIDARLite_I2C(I2C::Port port, int deviceAddress = kAddress, int configuration = 0);
  virtual ~LIDARLite_I2C() = default;

  LIDARLite_I2C(const LIDARLite_I2C&) = delete;
  LIDARLite_I2C& operator=(const LIDARLite_I2C&) = delete;

  void configure(int configuration = 0);
  void reset();
  int distance(bool biasCorrection = true);
  void read(int myAddress, int numOfBytes, byte* arrayToSave, bool monitorBusyFlag);

  bool getBit(byte input, int position);

  std::string GetSmartDashboardType() const override;
  void InitTable(std::shared_ptr<ITable> subtable) override;
  void UpdateTable() override;
  std::shared_ptr<ITable> GetTable() const override;
  void StartLiveWindowMode() override {}
  void StopLiveWindowMode() override {}

 protected:
  I2C m_i2c;

 private:
  std::shared_ptr<ITable> m_table;
};

}  // namespace frc

#endif // !LIDARLiteRio_H
