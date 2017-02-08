/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ADXL345_I2C.h"
#include "I2C.h"
#include "LiveWindow/LiveWindow.h"

using namespace frc;

const int LIDARLite::kAddress;
const int LIDARLite::kStatusRegister;

/*------------------------------------------------------------------------------
  Constructor

  Starts the sensor and I2C.

  Parameters
  ------------------------------------------------------------------------------
  port: I2C port to communication on
  deviceAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
  configuration: Default 0. Selects one of several preset configurations.
------------------------------------------------------------------------------*/
LIDARLite::LIDARLite(I2C::Port port, int deviceAddress, int configuration)
    : m_i2c(port, deviceAddress) {
  configure(configuration);
}/* LIDARLite Constructor */

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
------------------------------------------------------------------------------*/
void LIDARLite::configure(int configuration){
  switch(configuration)
  {
    case 0: // Default mode, balanced performance
      m_i2c.Write(0x02,0x80); // Default
      m_i2c.Write(0x04,0x08); // Default
      m_i2c.Write(0x1c,0x00); // Default
    break;

    case 1: // Short range, high speed
      m_i2c.Write(0x02,0x1d);
      m_i2c.Write(0x04,0x08); // Default
      m_i2c.Write(0x1c,0x00); // Default
    break;

    case 2: // Default range, higher speed short range
      m_i2c.Write(0x02,0x80); // Default
      m_i2c.Write(0x04,0x00);
      m_i2c.Write(0x1c,0x00); // Default
    break;

    case 3: // Maximum range
      m_i2c.Write(0x02,0xff);
      m_i2c.Write(0x04,0x08); // Default
      m_i2c.Write(0x1c,0x00); // Default
    break;

    case 4: // High sensitivity detection, high erroneous measurements
      m_i2c.Write(0x02,0x80); // Default
      m_i2c.Write(0x04,0x08); // Default
      m_i2c.Write(0x1c,0x80);
    break;

    case 5: // Low sensitivity detection, low erroneous measurements
      m_i2c.Write(0x02,0x80); // Default
      m_i2c.Write(0x04,0x08); // Default
      m_i2c.Write(0x1c,0xb0);
    break;
  }
}/* LIDARLite::configure */

/*------------------------------------------------------------------------------
  Reset

  Reset device. The device reloads default register settings, including the
  default I2C address. Re-initialization takes approximately 22ms.

------------------------------------------------------------------------------*/
void LIDARLite::reset(){
  m_i2c.Write(0x00, 0x00);
} /* LIDARLite::reset */

/*------------------------------------------------------------------------------
  Distance

  Take a distance measurement and read the result.

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 or 0x03 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
      The result is the measured distance in centimeters.

  Parameters
  ------------------------------------------------------------------------------
  biasCorrection: Default true. Take aquisition with receiver bias
    correction. If set to false measurements will be faster. Receiver bias
    correction must be performed periodically. (e.g. 1 out of every 100
    readings).
------------------------------------------------------------------------------*/
int LIDARLite::distance(bool biasCorrection){
  if(biasCorrection)
  {
    // Take acquisition & correlation processing with receiver bias correction
    m_i2c.Write(0x00,0x04);
  }
  else
  {
    // Take acquisition & correlation processing without receiver bias correction
    m_i2c.Write(0x00,0x03);
  }
  // Array to store high and low bytes of distance
  byte distanceArray[2];

  // Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
  // m_i2c.Read(int register, int count, uint8_t* buffer);
  read(0x8f, 2, &distanceArray, true);

  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return(distance);

}/* LIDARLite::distance */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device. The optional busy flag monitoring
  can be used to read registers that are updated at the end of a distance
  measurement to obtain the new data.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to read from.
  numOfBytes: numbers of bytes to read. Can be 1 or 2.
  arrayToSave: an array to store the read values.
  monitorBusyFlag: if true, the routine will repeatedly read the status
    register until the busy flag (LSB) is 0.
------------------------------------------------------------------------------*/
void LIDARLite::read(char myAddress, int numOfBytes, byte* arrayToSave[2], bool monitorBusyFlag){
  //Unlike the arduino library, busyMonitor will be used to monitor our state instead
  //of just whether we are or are not ready with results
  int busyMonitor = 0; //busyFlag monitors for when the device is done with a measurement
  if(monitorBusyFlag){
    busyMonitor = 9999; 
  }

  whlie(busyMonitor >= 0){
    //pull in the status register
    uint8_t test_byte;
    m_i2c.Read(LIDARLite::kStatusRegister, 1, &test_byte);

    //if the LSB is 0, then the reading is correct
    if(bitRead(test_byte, 0) == 0){
      busyMonitor = 0;
    }
    //Subtract 1
    busyMonitor--;
  }

  if(busyMonitor == -1){
    m_i2c.Read(myAddress, numOfBytes, arrayToSave); 
  }

} /* LIDARLite::read */

void LIDARLite::correlationRecordToSerial(char separator = '\n', int numberOfReadings = 256);




ADXL345_I2C::ADXL345_I2C(I2C::Port port, Range range, int deviceAddress)
    : m_i2c(port, deviceAddress) {
  // Turn on the measurements
  m_i2c.Write(kPowerCtlRegister, kPowerCtl_Measure);
  // Specify the data format to read
  SetRange(range);

  HAL_Report(HALUsageReporting::kResourceType_ADXL345,
             HALUsageReporting::kADXL345_I2C, 0);
  LiveWindow::GetInstance()->AddSensor("ADXL345_I2C", port, this);
}

void ADXL345_I2C::SetRange(Range range) {
  m_i2c.Write(kDataFormatRegister,
              kDataFormat_FullRes | static_cast<uint8_t>(range));
}

double ADXL345_I2C::GetX() { return GetAcceleration(kAxis_X); }

double ADXL345_I2C::GetY() { return GetAcceleration(kAxis_Y); }

double ADXL345_I2C::GetZ() { return GetAcceleration(kAxis_Z); }

/**
 * Get the acceleration of one axis in Gs.
 *
 * @param axis The axis to read from.
 * @return Acceleration of the ADXL345 in Gs.
 */
double ADXL345_I2C::GetAcceleration(ADXL345_I2C::Axes axis) {
  int16_t rawAccel = 0;
  m_i2c.Read(kDataRegister + static_cast<int>(axis), sizeof(rawAccel),
             reinterpret_cast<uint8_t*>(&rawAccel));
  return rawAccel * kGsPerLSB;
}

/**
 * Get the acceleration of all axes in Gs.
 *
 * @return An object containing the acceleration measured on each axis of the
 *         ADXL345 in Gs.
 */
ADXL345_I2C::AllAxes ADXL345_I2C::GetAccelerations() {
  AllAxes data = AllAxes();
  int16_t rawData[3];
  m_i2c.Read(kDataRegister, sizeof(rawData),
             reinterpret_cast<uint8_t*>(rawData));

  data.XAxis = rawData[0] * kGsPerLSB;
  data.YAxis = rawData[1] * kGsPerLSB;
  data.ZAxis = rawData[2] * kGsPerLSB;
  return data;
}

std::string ADXL345_I2C::GetSmartDashboardType() const {
  return "3AxisAccelerometer";
}

void ADXL345_I2C::InitTable(std::shared_ptr<ITable> subtable) {
  m_table = subtable;
  UpdateTable();
}

void ADXL345_I2C::UpdateTable() {
  m_table->PutNumber("X", GetX());
  m_table->PutNumber("Y", GetY());
  m_table->PutNumber("Z", GetZ());
}

std::shared_ptr<ITable> ADXL345_I2C::GetTable() const { return m_table; }