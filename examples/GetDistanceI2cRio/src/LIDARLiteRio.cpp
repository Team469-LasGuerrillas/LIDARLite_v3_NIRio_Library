/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LIDARLiteRio.h"
#include "I2C.h"
#include "LiveWindow/LiveWindow.h"

using namespace frc;

const int LIDARLite_I2C::kAddress;
const int LIDARLite_I2C::kStatusRegister;
const int LIDARLite_I2C::kDistanceRegister;
const int LIDARLite_I2C::kStatusFlag_Busy;
const int LIDARLite_I2C::kStatusFlag_ReferenceOverflow;
const int LIDARLite_I2C::kStatusFlag_SignalOverflow;
const int LIDARLite_I2C::kStatusFlag_InvalidSignal;
const int LIDARLite_I2C::kStatusFlag_SecondaryReturn;
const int LIDARLite_I2C::kStatusFlag_Health;
const int LIDARLite_I2C::kStatusFlag_ProcessError;

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
LIDARLite_I2C::LIDARLite_I2C(I2C::Port port, int deviceAddress, int configuration)
    : m_i2c(port, deviceAddress)
{
	deviceBusy = 0;
	referenceOverflow = 0;
	signalOverflow = 0;
	invalidSignal = 0;
	secondaryReturnValid = 0;
	sensorHealth = 0;
	processError = 0;
	status = 0x0;
	reset();
	configure(configuration);
}/* LIDARLite_I2C Constructor */

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
void LIDARLite_I2C::configure(int configuration){
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
}/* LIDARLite_I2C::configure */

/*------------------------------------------------------------------------------
  Reset

  Reset device. The device reloads default register settings, including the
  default I2C address. Re-initialization takes approximately 22ms.

------------------------------------------------------------------------------*/
void LIDARLite_I2C::reset(){
  m_i2c.Write(0x00, 0x00);
} /* LIDARLite_I2C::reset */

void LIDARLite_I2C::updateStatus(){
	deviceBusy = getBit(status, kStatusFlag_Busy);
	referenceOverflow = getBit(status, kStatusFlag_ReferenceOverflow);
	signalOverflow = getBit(status, kStatusFlag_SignalOverflow);
	invalidSignal = getBit(status, kStatusFlag_InvalidSignal);
	secondaryReturnValid = getBit(status, kStatusFlag_SecondaryReturn);
	sensorHealth = getBit(status, kStatusFlag_Health);
	processError = getBit(status, kStatusFlag_ProcessError);
}

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
int LIDARLite_I2C::distance(bool biasCorrection){
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
  uint8_t distanceArray[2];

  // Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
  read(kDistanceRegister, 2, distanceArray, true);

  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return(distance);

}/* LIDARLite_I2C::distance */

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
void LIDARLite_I2C::read(int myAddress, int numOfBytes, uint8_t * arrayToSave, bool monitorBusyFlag){
  //Unlike the arduino library, busyMonitor will be used to monitor our state instead
  //of just whether we are or are not ready with results
  int busyMonitor = 0; //busyFlag monitors for when the device is done with a measurement
  if(monitorBusyFlag){
    busyMonitor = 9999; 
  }

  while(busyMonitor >= 0){
    //pull in the status register
	lidar_read(LIDARLite_I2C::kStatusRegister, &status, 1);
    //m_i2c.Read(LIDARLite_I2C::kStatusRegister, 1, &status);
    updateStatus();
    //if the LSB is 0, then the reading is correct
    if(deviceBusy == 0){
      busyMonitor = 0;
    }
    //Subtract 1
    busyMonitor--;
  }

  if(busyMonitor == -1){
	  lidar_read(myAddress, arrayToSave, numOfBytes);
//    m_i2c.Read(myAddress, numOfBytes, arrayToSave);
  } else {
//	  printf("\tsensor reading fail\n");
  }
  printf("t");
//  printf("\tbyte1:%x ", arrayToSave[0]);
//  printf("byte2:%x\n", arrayToSave[1]);

} /* LIDARLite_I2C::read */

int LIDARLite_I2C::getBit(uint8_t input, int position){
	return ((input >> position) & 0x1)? 1 : 0;
}/* LIDARLite_I2C::getBit */

void LIDARLite_I2C::lidar_write(uint8_t dev_register, uint8_t * dataToSend, int sendSize){
	uint8_t buffer[sendSize + 1];
	buffer[0] = dev_register;
	for(int i = 1; i < sendSize + 1; i++){
		buffer[i] = dataToSend[i - 1];
	}
	m_i2c.Transaction(buffer, sendSize + 1, NULL, 0);
}

void LIDARLite_I2C::lidar_read(uint8_t dev_register, uint8_t * dataReceived, int receiveSize){
//	lidar_write(dev_register, NULL, 0);
	m_i2c.Transaction(&dev_register, 1, dataReceived, receiveSize);
}

std::string LIDARLite_I2C::GetSmartDashboardType() const {
  return "LIDARLite_v3";
}

void LIDARLite_I2C::InitTable(std::shared_ptr<ITable> subtable) {
  m_table = subtable;
  UpdateTable();
}

void LIDARLite_I2C::UpdateTable() {
  int newDist = distance(true);
  m_table->PutNumber("Bias_Correct_Distance", newDist);
  m_table->PutNumber("LIDARLite_busyFlag", deviceBusy);
  m_table->PutNumber("LIDARLite_referenceOverflow", referenceOverflow);
  m_table->PutNumber("LIDARLite_signalOverflow", signalOverflow);
  m_table->PutNumber("LIDARLite_invalidSignal", invalidSignal);
  m_table->PutNumber("LIDARLite_secondaryValid", secondaryReturnValid);
  m_table->PutNumber("LIDARLite_sensorHealth", sensorHealth);
  m_table->PutNumber("LIDARLite_processError", processError);
//  m_table->PutNumber("Biased_Distance", distance(false));
}

std::shared_ptr<ITable> LIDARLite_I2C::GetTable() const { return m_table; }
