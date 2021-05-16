/*!
 * @file BNO055.cpp
 *
 *  @mainpage Bosch BNO055 Orientation sensor and AHRS system
 *
 *  @section Information
 *
 *  This is a library for the Bosch BNO055 Orientation sensor and AHRS system in ardunio framework
 * 
 *  This driver is written by inspiring from [Adafruit Unified BNO055 Driver (AHRS/Orientation)](https://github.com/adafruit/Adafruit_BNO055).
 *
 *  Find more detail on the sensor on https://www.bosch-sensortec.com/products/smart-sensors/bno055/
 *
 *  This library is only supporting I2C interface not SPI.
 *
 *  @section author Author
 *
 *  MReza Naeemabadi
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "Arduino.h"

#include <limits.h>
#include <math.h>

#include "BNO055.h"

/*!
 *  @brief  Instantiates a new BNO055 class
 *  @param  address
 *          i2c address
 *  @param  theWire
 *          Wire object
 */
bool BNO055::Initialize(uint8_t address, BNO055_opmode_t mode, TwoWire &theWire)
{
  _address = address;
  _wire = &theWire;
  return begin(mode);
}

bool BNO055::Initialize(BNO055_opmode_t mode, TwoWire &theWire)
{

  _wire = &theWire;

  _address = BNO055_ADDRESS_A;
  if (read(BNO055_CHIP_ID_ADDR) == BNO055_ID)
    return begin(mode);

  delay(1000);
  if (read(BNO055_CHIP_ID_ADDR) == BNO055_ID)
    return begin(mode);

  _address = BNO055_ADDRESS_B;
  if (read(BNO055_CHIP_ID_ADDR) == BNO055_ID)
    return begin(mode);

  if (read(BNO055_CHIP_ID_ADDR) == BNO055_ID)
    return begin(mode);

  return false;
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool BNO055::begin(BNO055_opmode_t mode)
{
#if defined(ARDUINO_SAMD_ZERO) && (_address == BNO055_ADDRESS_A)
#error \
    "On an arduino Zero, BNO055's ADR pin must be high. Fix that, then delete this line."
  _address = BNO055_ADDRESS_B;
#endif

  /* Enable I2C */
  _wire->begin();

  // BNO055 clock stretches for 500us or more!
#ifdef ESP8266
  _wire->setClockStretchLimit(1000); // Allow for 1000us of clock stretching
#endif

  /* Make sure we have the right device */
  uint8_t id = read(BNO055_CHIP_ID_ADDR);
  if (id != BNO055_ID)
  {
    delay(1000); // hold on for boot
    id = read(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID)
      return false; // still not? ok bail
  }

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);

  /* Reset */
  write(BNO055_SYS_TRIGGER_ADDR, 0x20);
  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  delay(30);
  while (read(BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  write(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */

  write(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(mode);
  delay(20);

  return true;
}

/*!
 *  @brief  Puts the chip in the specified operating mode
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 */
void BNO055::setMode(BNO055_opmode_t mode)
{
  _mode = mode;
  write(BNO055_OPR_MODE_ADDR, _mode);
  delay(30);
}

/*!
 *  @brief  Changes the chip's axis remap
 *  @param  remapcode
 *          remap code possible values
 *          [REMAP_CONFIG_P0
 *           REMAP_CONFIG_P1 (default)
 *           REMAP_CONFIG_P2
 *           REMAP_CONFIG_P3
 *           REMAP_CONFIG_P4
 *           REMAP_CONFIG_P5
 *           REMAP_CONFIG_P6
 *           REMAP_CONFIG_P7]
 */
void BNO055::setAxisRemap(
    BNO055_axis_remap_config_t remapcode)
{
  BNO055_opmode_t modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}

/*!
 *  @brief  Changes the chip's axis signs
 *  @param  remapsign
 *          remap sign possible values
 *          [REMAP_SIGN_P0
 *           REMAP_SIGN_P1 (default)
 *           REMAP_SIGN_P2
 *           REMAP_SIGN_P3
 *           REMAP_SIGN_P4
 *           REMAP_SIGN_P5
 *           REMAP_SIGN_P6
 *           REMAP_SIGN_P7]
 */
void BNO055::setAxisSign(BNO055_axis_remap_sign_t remapsign)
{
  BNO055_opmode_t modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}

/*!
 *  @brief  Use the external 32.768KHz crystal
 *  @param  usextal
 *          use external crystal boolean
 */
void BNO055::setExtCrystalUse(boolean usextal)
{
  BNO055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  if (usextal)
  {
    write(BNO055_SYS_TRIGGER_ADDR, 0x80);
  }
  else
  {
    write(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}

/*!
 *   @brief  Gets the latest system status info
 *   @param  system_status
 *           system status info
 *   @param  self_test_result
 *           self test result
 *   @param  system_error
 *           system error info
 */
void BNO055::getSystemStatus(uint8_t *system_status,
                             uint8_t *self_test_result,
                             uint8_t *system_error)
{
  write(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  if (system_status != 0)
    *system_status = read(BNO055_SYS_STAT_ADDR);

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0)
    *self_test_result = read(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0)
    *system_error = read(BNO055_SYS_ERR_ADDR);

  delay(200);
}

/*!
 *  @brief  Gets the chip revision numbers
 *  @param  info
 *          revision info
 */
void BNO055::getRevInfo(BNO055_rev_info_t *info)
{
  uint8_t a, b;

  memset(info, 0, sizeof(BNO055_rev_info_t));

  /* Check the accelerometer revision */
  info->accel_rev = read(BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev = read(BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev = read(BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev = read(BNO055_BL_REV_ID_ADDR);

  a = read(BNO055_SW_REV_ID_LSB_ADDR);
  b = read(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

/*!
 *  @brief  Prints the chip revision numbers
 *          revision info - Use getRevInfo in FreeRTOS to fetch the information
 */
void BNO055::printSensorInfo()
{
  BNO055_rev_info_t sensorInfo;
  getRevInfo(&sensorInfo);
  printf("BNO055 >> Software Revision ID  %X\r\n", sensorInfo.sw_rev);
  printf("BNO055 >> Bootloader Version    %X\r\n", sensorInfo.bl_rev);
  printf("BNO055 >> Accelerometer chip ID %X\r\n", sensorInfo.accel_rev);
  printf("BNO055 >> Gyroscope chip ID     %X\r\n", sensorInfo.gyro_rev);
  printf("BNO055 >> Magnetometer chip ID  %X\r\n", sensorInfo.mag_rev);
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void BNO055::getCalibration(uint8_t *sys, uint8_t *gyro,
                            uint8_t *accel, uint8_t *mag)
{
  uint8_t calData = read(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL)
  {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL)
  {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL)
  {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL)
  {
    *mag = calData & 0x03;
  }
}

/*!
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @return calibration status in structure data format
 */
BNO055_calibration_state_t BNO055::getCalibration()
{
  BNO055_calibration_state_t cal;
  uint8_t calData = read(BNO055_CALIB_STAT_ADDR);

  cal.SystemCalibrationState = (calData >> 6) & 0x03;
  cal.GyroscropeCalibrationState = (calData >> 4) & 0x03;
  cal.AccelerometerCalibrationState = (calData >> 2) & 0x03;
  cal.MagnetometerCalibrationState = calData & 0x03;

  return cal;
}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t BNO055::getTemp()
{
  int8_t temp = (int8_t)(read(BNO055_TEMP_ADDR));
  return temp;
}

/*!
 *  @brief  Gets all measurments from the sensor (not processed) - use this method when using NDOF 
 *  @return structure of all measurments
 */
BNO055_raw_measurment_data_t BNO055::getFullMeasurment()
{
  BNO055_raw_measurment_data_t values;
  uint8_t buffer[6];

  // Read Accelerometer measurment
  read((BNO055_reg_t)BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
  values.Acceleration.X = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.Acceleration.Y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.Acceleration.Z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Read Gyroscope measurment
  read((BNO055_reg_t)BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6);
  values.AngularVelocity.X = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.AngularVelocity.Y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.AngularVelocity.Z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Read Magnetometer measurment
  read((BNO055_reg_t)BNO055_MAG_DATA_X_LSB_ADDR, buffer, 6);
  values.MagneticField.X = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.MagneticField.Y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.MagneticField.Z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Read Linear Acceleration
  read((BNO055_reg_t)BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
  values.LinearAcceleration.X = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.LinearAcceleration.Y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.LinearAcceleration.Z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Read Gravity
  read((BNO055_reg_t)BNO055_GRAVITY_DATA_X_LSB_ADDR, buffer, 6);
  values.GravityVector.X = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.GravityVector.Y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.GravityVector.Z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Read Euler Angles
  read((BNO055_reg_t)BNO055_EULER_H_LSB_ADDR, buffer, 6);
  values.EulerAngles.Heading = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.EulerAngles.Roll = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.EulerAngles.Pitch = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  // Read quaternion
  read(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  values.Quaternion.W = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  values.Quaternion.X = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  values.Quaternion.Y = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
  values.Quaternion.Z = ((int16_t)buffer[6]) | (((int16_t)buffer[7]) << 8);

  return values;
}

/*!
 *  @brief  Gets all measurments from the sensor (processed) - use this method when using NDOF
 *  @param  in_mg_scale set true if you wish to receive measurments in mg otherwise the acceleration measurments will be in m/s2
 *  @param  in_dps_scale set true if you wish to receive measurments in deg/sec otherwise the angle measurments will be in rad/sec
 *  @return structure of all measurments
 */
BNO055_measurment_data_t BNO055::getFullMeasurment(bool in_mg_scale, bool in_dps_scale)
{
  BNO055_measurment_data_t values;
  uint8_t buffer[6];

  double acceleration_scale = in_mg_scale ? 1 : 0.01;
  double angle_scale = in_dps_scale ? (1.0 / 16) : (1.0 / 900);
  // Read Accelerometer measurment
  read((BNO055_reg_t)BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
  values.Acceleration.X = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) * acceleration_scale;
  values.Acceleration.Y = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) * acceleration_scale;
  values.Acceleration.Z = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) * acceleration_scale;

  // Read Gyroscope measurment
  read((BNO055_reg_t)BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6);
  values.AngularVelocity.X = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) * angle_scale;
  values.AngularVelocity.Y = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) * angle_scale;
  values.AngularVelocity.Z = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) * angle_scale;

  // Read Magnetometer measurment
  read((BNO055_reg_t)BNO055_MAG_DATA_X_LSB_ADDR, buffer, 6);
  values.MagneticField.X = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) / 16;
  values.MagneticField.Y = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) / 16;
  values.MagneticField.Z = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) / 16;

  // Read Linear Acceleration
  read((BNO055_reg_t)BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
  values.LinearAcceleration.X = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) * acceleration_scale;
  values.LinearAcceleration.Y = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) * acceleration_scale;
  values.LinearAcceleration.Z = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) * acceleration_scale;

  // Read Gravity
  read((BNO055_reg_t)BNO055_GRAVITY_DATA_X_LSB_ADDR, buffer, 6);
  values.GravityVector.X = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) * acceleration_scale;
  values.GravityVector.Y = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) * acceleration_scale;
  values.GravityVector.Z = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) * acceleration_scale;

  // Read Euler Angles
  read((BNO055_reg_t)BNO055_EULER_H_LSB_ADDR, buffer, 6);
  values.EulerAngles.Heading = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) * angle_scale;
  values.EulerAngles.Roll = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) * angle_scale;
  values.EulerAngles.Pitch = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) * angle_scale;

  // Read quaternion
  read(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  values.Quaternion.W = (double)(((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8)) / 16384;
  values.Quaternion.X = (double)(((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8)) / 16384;
  values.Quaternion.Y = (double)(((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8)) / 16384;
  values.Quaternion.Z = (double)(((int16_t)buffer[6]) | (((int16_t)buffer[7]) << 8)) / 16384;

  return values;
}

/*!
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @return  vector from specified source
 */
imu::Vector<3> BNO055::getVector(sensor_type_t sensor_type)
{
  imu::Vector<3> xyz;
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  read((BNO055_reg_t)sensor_type, buffer, 6);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (sensor_type)
  {
  case SENSOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case SENSOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case SENSOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((double)x) / 16.0;
    xyz[1] = ((double)y) / 16.0;
    xyz[2] = ((double)z) / 16.0;
    break;
  case SENSOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case SENSOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  case SENSOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double)x) / 100.0;
    xyz[1] = ((double)y) / 100.0;
    xyz[2] = ((double)z) / 100.0;
    break;
  }

  return xyz;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
imu::Quaternion BNO055::getQuat()
{
  uint8_t buffer[8];
  memset(buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  read(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /*!
   * Assign to Quaternion
   * See
   * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));
  imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  return quat;
}

/*!
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset (buffer size should be 22)
 *  @return true if read is successful
 */
bool BNO055::getSensorOffsets(uint8_t *calibData)
{
  if (isFullyCalibrated())
  {
    BNO055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);

    read(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

    setMode(lastMode);
    return true;
  }
  return false;
}

/*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool BNO055::getSensorOffsets(
    BNO055_offsets_t &offsets_type)
{
  if (isFullyCalibrated())
  {
    BNO055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* Accel offset range depends on the G-range:
       +/-2g  = +/- 2000 mg
       +/-4g  = +/- 4000 mg
       +/-8g  = +/- 8000 mg
       +/-1§g = +/- 16000 mg */
    offsets_type.accel_offset_x = (read(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                  (read(ACCEL_OFFSET_X_LSB_ADDR));
    offsets_type.accel_offset_y = (read(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                  (read(ACCEL_OFFSET_Y_LSB_ADDR));
    offsets_type.accel_offset_z = (read(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                  (read(ACCEL_OFFSET_Z_LSB_ADDR));

    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    offsets_type.mag_offset_x =
        (read(MAG_OFFSET_X_MSB_ADDR) << 8) | (read(MAG_OFFSET_X_LSB_ADDR));
    offsets_type.mag_offset_y =
        (read(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read(MAG_OFFSET_Y_LSB_ADDR));
    offsets_type.mag_offset_z =
        (read(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read(MAG_OFFSET_Z_LSB_ADDR));

    /* Gyro offset range depends on the DPS range:
      2000 dps = +/- 32000 LSB
      1000 dps = +/- 16000 LSB
       500 dps = +/- 8000 LSB
       250 dps = +/- 4000 LSB
       125 dps = +/- 2000 LSB
       ... where 1 DPS = 16 LSB */
    offsets_type.gyro_offset_x =
        (read(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read(GYRO_OFFSET_X_LSB_ADDR));
    offsets_type.gyro_offset_y =
        (read(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read(GYRO_OFFSET_Y_LSB_ADDR));
    offsets_type.gyro_offset_z =
        (read(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read(GYRO_OFFSET_Z_LSB_ADDR));

    /* Accelerometer radius = +/- 1000 LSB */
    offsets_type.accel_radius =
        (read(ACCEL_RADIUS_MSB_ADDR) << 8) | (read(ACCEL_RADIUS_LSB_ADDR));

    /* Magnetometer radius = +/- 960 LSB */
    offsets_type.mag_radius =
        (read(MAG_RADIUS_MSB_ADDR) << 8) | (read(MAG_RADIUS_LSB_ADDR));

    setMode(lastMode);
    return true;
  }
  return false;
}

/*!
 *  @brief  Writes an array of calibration values to the sensor's offset
 *  @param  calibData
 *          calibration data
 */
void BNO055::setSensorOffsets(const uint8_t *calibData)
{
  BNO055_opmode_t lastMode = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  /* A writeLen() would make this much cleaner */
  write(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
  write(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
  write(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
  write(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
  write(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
  write(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

  write(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
  write(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
  write(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
  write(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
  write(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
  write(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

  write(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
  write(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
  write(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
  write(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
  write(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
  write(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

  write(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
  write(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

  write(MAG_RADIUS_LSB_ADDR, calibData[20]);
  write(MAG_RADIUS_MSB_ADDR, calibData[21]);

  setMode(lastMode);
}

/*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 *          accel_offset_x = acceleration offset x
 *          accel_offset_y = acceleration offset y
 *          accel_offset_z = acceleration offset z
 *
 *          mag_offset_x   = magnetometer offset x
 *          mag_offset_y   = magnetometer offset y
 *          mag_offset_z   = magnetometer offset z
 *
 *          gyro_offset_x  = gyroscrope offset x
 *          gyro_offset_y  = gyroscrope offset y
 *          gyro_offset_z  = gyroscrope offset z
 */
void BNO055::setSensorOffsets(
    const BNO055_offsets_t &offsets_type)
{
  BNO055_opmode_t lastMode = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  write(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
  write(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
  write(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
  write(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
  write(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
  write(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

  write(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
  write(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
  write(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
  write(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
  write(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
  write(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

  write(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
  write(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
  write(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
  write(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
  write(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
  write(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

  write(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
  write(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

  write(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
  write(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

  setMode(lastMode);
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool BNO055::isFullyCalibrated()
{
  uint8_t system, gyro, accel, mag;
  getCalibration(&system, &gyro, &accel, &mag);

  switch (_mode)
  {
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

/*!
 *  @brief  Enter Suspend mode (i.e., sleep)
 */
void BNO055::enterSuspendMode()
{
  BNO055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write(BNO055_PWR_MODE_ADDR, 0x02);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}

/*!
 *  @brief  Enter Normal mode (i.e., wake)
 */
void BNO055::enterNormalMode()
{
  BNO055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write(BNO055_PWR_MODE_ADDR, 0x00);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}

// ! Page 1 methods
/*!
 *  @brief  Getting Accelerometer Range
 *  @return Accelerometer range (2G-16G) check BNO_acc_range_t for more detail
 */
BNO055::BNO_acc_range_t BNO055::getAccelerometerRange()
{
  uint8_t value = read(ACC_CONFIG_ADDR, PAGE_ONE) & 0x03;
  return (BNO_acc_range_t)value;
}

/*!
 *  @brief  Getting Accelerometer Bandwidth
 *  @return Accelerometer bandwidth (7.81-1000Hz) check BNO_acc_BW_t for more detail
 */
BNO055::BNO_acc_BW_t BNO055::getAccelerometerBW()
{
  uint8_t value = (read(ACC_CONFIG_ADDR, PAGE_ONE) >> 2) & 0x07;
  return (BNO_acc_BW_t)value;
}

/*!
 *  @brief  Getting Accelerometer operation mode
 *  @return Accelerometer operation mode check BNO_acc_operation_mode_t for more detail
 */
BNO055::BNO_acc_operation_mode_t BNO055::getAccelerometerOperationMode()
{
  uint8_t value = (read(ACC_CONFIG_ADDR, PAGE_ONE) >> 5) & 0x07;
  return (BNO_acc_operation_mode_t)value;
}

/*!
 *  @brief  Setting Accelerometer Range
 *  @param  range check BNO_acc_range_t for more detail
 */
void BNO055::setAccelerometerRange(BNO_acc_range_t range)
{
  uint8_t value = read(ACC_CONFIG_ADDR, PAGE_ONE);
  value = (value & 0xFC) | range;
  write(ACC_CONFIG_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Accelerometer bandwidth
 *  @param  bandwitdh check BNO_acc_BW_t for more detail
 */
void BNO055::setAccelerometerBW(BNO_acc_BW_t bandwitdh)
{
  uint8_t value = read(ACC_CONFIG_ADDR, PAGE_ONE);
  value = (value & 0xE3) | (bandwitdh << 2);
  write(ACC_CONFIG_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Accelerometer operation mode
 *  @param  operation_mode check BNO_acc_operation_mode_t for more detail
 */
void BNO055::setAccelerometerOperationMode(BNO_acc_operation_mode_t mode)
{
  uint8_t value = read(ACC_CONFIG_ADDR, PAGE_ONE);
  value = (value & 0x1F) | (mode << 5);
  write(ACC_CONFIG_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Getting Gyroscope Range
 *  @return Gyroscope range (125-2000d/s) check BNO_gyr_range_t for more detail
 */
BNO055::BNO_gyr_range_t BNO055::getGyroscopeRange()
{
  uint8_t value = read(GYR_CONFIG_0_ADDR, PAGE_ONE) & 0x07;
  return (BNO_gyr_range_t)value;
}

/*!
 *  @brief  Getting Gyroscope Bandwidth
 *  @return Gyroscope bandwitdh (12-523Hz) check BNO_gyr_BW_t for more detail
 */
BNO055::BNO_gyr_BW_t BNO055::getGyroscopeBW()
{
  uint8_t value = (read(GYR_CONFIG_0_ADDR, PAGE_ONE) >> 3) & 0x07;
  return (BNO_gyr_BW_t)value;
}

/*!
 *  @brief  Getting Gyroscope Operation Mode
 *  @return Gyroscope Operation Mode check BNO_gyr_operation_mode_t for more detail
 */
BNO055::BNO_gyr_operation_mode_t BNO055::getGyroscopeOperationMode()
{
  uint8_t value = read(GYR_CONFIG_1_ADDR, PAGE_ONE) & 0x07;
  return (BNO_gyr_operation_mode_t)value;
}

/*!
 *  @brief  Setting Gyroscope range
 *  @param  range 125-2000d/s check BNO_gyr_range_t for more detail
 */
void BNO055::setGyroscopeRange(BNO_gyr_range_t range)
{
  uint8_t value = read(GYR_CONFIG_0_ADDR, PAGE_ONE);
  value = (value & 0xF8) | range;
  write(GYR_CONFIG_0_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Gyroscope Bandwidth
 *  @param  bandwidth 12-523Hz check BNO_gyr_BW_t for more detail
 */
void BNO055::setGyroscopeBW(BNO_gyr_BW_t bandwidth)
{
  uint8_t value = read(GYR_CONFIG_0_ADDR, PAGE_ONE);
  value = (value & 0xC7) | (bandwidth << 3);
  write(GYR_CONFIG_0_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Gyroscope Operation Mode
 *  @param  mode Operation Mode check BNO_gyr_operation_mode_t for more detail
 */
void BNO055::setGyroscopeOperationMode(BNO_gyr_operation_mode_t mode)
{
  uint8_t value = read(GYR_CONFIG_1_ADDR, PAGE_ONE);
  value = (value & 0xF8) | mode;
  write(GYR_CONFIG_1_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Getting Magnetometer Operation Mode
 *  @return Magnetometer Operation Mode check BNO_mag_operation_mode_t for more detail
 */
BNO055::BNO_mag_operation_mode_t BNO055::getMagnetometerOperationMode()
{
  uint8_t value = (read(MAG_CONFIG_ADDR, PAGE_ONE) >> 3) & 0x03;
  return (BNO_mag_operation_mode_t)value;
}

/*!
 *  @brief  Getting Magnetometer Power Mode
 *  @return Magnetometer Power Mode check BNO_mag_power_mode_t for more detail
 */
BNO055::BNO_mag_power_mode_t BNO055::getMagnetometerPowerMode()
{
  uint8_t value = (read(MAG_CONFIG_ADDR, PAGE_ONE) >> 5) & 0x03;
  return (BNO_mag_power_mode_t)value;
}

/*!
 *  @brief  Getting Magnetometer Bandwidth
 *  @return Magnetometer bandwidth check BNO_mag_BW_t for more detail
 */
BNO055::BNO_mag_BW_t BNO055::getMagnetometerBW()
{
  uint8_t value = read(MAG_CONFIG_ADDR, PAGE_ONE) & 0x07;
  return (BNO_mag_BW_t)value;
}

/*!
 *  @brief  Setting Magnetometer Power Mode
 *  @param  mode Magnetometer Power Mode check BNO_mag_power_mode_t for more detail
 */
void BNO055::setMagnetometerPowerMode(BNO_mag_power_mode_t mode)
{
  uint8_t value = read(MAG_CONFIG_ADDR, PAGE_ONE);
  value = (value & 0x9F) | (mode << 5);
  write(MAG_CONFIG_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Magnetometer Operation Mode
 *  @param  mode Operation Mode check BNO_mag_operation_mode_t for more detail
 */
void BNO055::setMagnetometerOperationMode(BNO_mag_operation_mode_t mode)
{
  uint8_t value = read(MAG_CONFIG_ADDR, PAGE_ONE);
  value = (value & 0xE7) | (mode << 3);
  write(MAG_CONFIG_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Magnetometer Bandwidth
 *  @param  bandwidth Bandwidth check BNO_mag_BW_t for more detail
 */
void BNO055::settMagnetometerBW(BNO_mag_BW_t bandwidth)
{
  uint8_t value = read(MAG_CONFIG_ADDR, PAGE_ONE);
  value = (value & 0xF8) | bandwidth;
  write(MAG_CONFIG_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Getting Interrupt Mask. This shows which interrupt are triggering INT pin 
 *  @return may contain single or multiple interrupt source check BNO_interrupts_t for more datail
 */
BNO055::BNO_interrupts_t BNO055::getExternalInterruptEnable()
{
  uint8_t value = read(INT_MASK_ADDR, PAGE_ONE);
  return (BNO_interrupts_t)value;
}

/*!
 *  @brief  Setting Interrupt Mask for triggering INT pin.
 *  Please be aware this method will overwrite the old setting but doesn't enable/disable interrupts
 *  @param status may contain single or multiple interrupt source. Use | activate multiple interrupts.
 *  check BNO_interrupts_t for more datail
 */
void BNO055::setExternalInterruptEnable(BNO_interrupts_t source)
{
  write(INT_MASK_ADDR, source, PAGE_ONE);
}

/*!
 *  @brief  Setting Interrupt Mask for triggering INT pin.
 *  @param status Target interrupt source. don't use multiple interrupts.
 *  check BNO_interrupts_t for more datail
 *  @param enable true to enable interrupt.
 */
void BNO055::setExternalInterruptEnable(BNO_interrupts_t source, bool enable)
{
  switch (source)
  {
  case BNO_interrupts_t::ACCELEROMETER_BSX_DATA_READY:
    if (enable)
      set(INT_MASK_ADDR, 0, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 0, PAGE_ONE);
    break;

  case BNO_interrupts_t::MAGNETOMETER_DATA_READY:
    if (enable)
      set(INT_MASK_ADDR, 1, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 1, PAGE_ONE);
    break;

  case BNO_interrupts_t::GYROSCOPE_ANY_MOTION:
    if (enable)
      set(INT_MASK_ADDR, 2, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 2, PAGE_ONE);
    break;

  case BNO_interrupts_t::GYROSCOPE_HIGH_RATE:
    if (enable)
      set(INT_MASK_ADDR, 3, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 3, PAGE_ONE);
    break;

  case BNO_interrupts_t::GYROSCOPE_DATA_READY:
    if (enable)
      set(INT_MASK_ADDR, 4, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 4, PAGE_ONE);
    break;

  case BNO_interrupts_t::ACCELEROMETER_HIGH_G:
    if (enable)
      set(INT_MASK_ADDR, 5, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 5, PAGE_ONE);
    break;

  case BNO_interrupts_t::ACCELEROMETER_ANY_MOTION:
    if (enable)
      set(INT_MASK_ADDR, 6, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 6, PAGE_ONE);
    break;

  case BNO_interrupts_t::ACCELEROMETER_NO_MOTION:
    if (enable)
      set(INT_MASK_ADDR, 7, PAGE_ONE);
    else
      unset(INT_MASK_ADDR, 7, PAGE_ONE);
    break;

  default:
    break;
  }
}

/*!
 *  @brief  Getting the enabled intterupt source. This method doesn't change the interrupt mask
 *  @return may contain single or multiple interrupt source check BNO_interrupts_t for more datail
 */
BNO055::BNO_interrupts_t BNO055::getInterruptEnable()
{
  uint8_t value = read(INT_EN_ADDR, PAGE_ONE);
  return (BNO_interrupts_t)value;
}

/*!
 *  @brief  Enabling the interrupt source(s).
 *  Please be aware this method will overwrite the old setting but doesn't change the interrupt mask
 *  @param status may contain single or multiple interrupt source. Use | activate multiple interrupts.
 *  check BNO_interrupts_t for more datail
 */
void BNO055::setInterruptEnable(BNO_interrupts_t status)
{
  write(INT_EN_ADDR, status, PAGE_ONE);
}

/*!
 *  @brief  Enabling the interrupt source.
 *  @param status Target interrupt source. don't use multiple interrupts.
 *  check BNO_interrupts_t for more datail
 *  @param enable true to enable interrupt.
 */
void BNO055::setInterruptEnable(BNO_interrupts_t source, bool enable)
{
  switch (source)
  {
  case BNO_interrupts_t::ACCELEROMETER_BSX_DATA_READY:
    if (enable)
      set(INT_EN_ADDR, 0, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 0, PAGE_ONE);
    break;

  case BNO_interrupts_t::MAGNETOMETER_DATA_READY:
    if (enable)
      set(INT_EN_ADDR, 1, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 1, PAGE_ONE);
    break;

  case BNO_interrupts_t::GYROSCOPE_ANY_MOTION:
    if (enable)
      set(INT_EN_ADDR, 2, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 2, PAGE_ONE);
    break;

  case BNO_interrupts_t::GYROSCOPE_HIGH_RATE:
    if (enable)
      set(INT_EN_ADDR, 3, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 3, PAGE_ONE);
    break;

  case BNO_interrupts_t::GYROSCOPE_DATA_READY:
    if (enable)
      set(INT_EN_ADDR, 4, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 4, PAGE_ONE);
    break;

  case BNO_interrupts_t::ACCELEROMETER_HIGH_G:
    if (enable)
      set(INT_EN_ADDR, 5, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 5, PAGE_ONE);
    break;

  case BNO_interrupts_t::ACCELEROMETER_ANY_MOTION:
    if (enable)
      set(INT_EN_ADDR, 6, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 6, PAGE_ONE);
    break;

  case BNO_interrupts_t::ACCELEROMETER_NO_MOTION:
    if (enable)
      set(INT_EN_ADDR, 7, PAGE_ONE);
    else
      unset(INT_EN_ADDR, 7, PAGE_ONE);
    break;

  default:
    break;
  }
}

/*!
 *  @brief  Disabling all interrupt source(s).
 */
void BNO055::disableInterruptEnable()
{
  write(INT_EN_ADDR, 0x00, PAGE_ONE);
}

/*!
 *  @brief  Disconnecting all interrupt source(s) to interrupt pin.
 */
void BNO055::disableExternalInterruptEnable()
{
  write(INT_MASK_ADDR, 0x00, PAGE_ONE);
}

/*!
 *  @brief  Getting the Threshold of Any Motion Interrupt for Accelerometer
 *  @return The raw value of motion Threshold
 */
uint8_t BNO055::getAccelerometerAnyMotionThreshold()
{
  return read(ACC_AM_THRES_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Getting the Threshold of Any Motion Interrupt for Accelerometer
 *  @return The motion Threshold in mg scle
 */
float BNO055::getAccelerometerAnyMotionThreshold_mg()
{
  BNO_acc_range_t range = getAccelerometerRange();
  uint8_t value = read(ACC_AM_THRES_ADDR, PAGE_ONE);
  switch (range)
  {
  case BNO_acc_range_t::G_02:
    return value * 3.91f;

  case BNO_acc_range_t::G_04:
    return value * 7.81f;

  case BNO_acc_range_t::G_08:
    return value * 15.63f;

  case BNO_acc_range_t::G_16:
    return value * 31.25f;

  default:
    return 0;
  }
}

/*!
 *  @brief  Setting the Threshold of Any Motion Interrupt for Accelerometer
 *  @param  threshold the raw value of any motion threshold see section 4.4.10 section for more detail
 */
void BNO055::setAccelerometerAnyMotionThreshold(uint8_t threshold)
{
  write(ACC_AM_THRES_ADDR, threshold, PAGE_ONE);
}

/*!
 *  @brief  Setting the Threshold of Any Motion Interrupt for Accelerometer
 *  @param  threshold in mg scale
 */
void BNO055::setAccelerometerAnyMotionThreshold(float thr)
{
  BNO_acc_range_t range = getAccelerometerRange();
  uint8_t value;
  switch (range)
  {
  case BNO_acc_range_t::G_02:
    value = (uint8_t)(thr / 3.91f);
    break;
  case BNO_acc_range_t::G_04:
    value = (uint8_t)(thr / 7.81f);
    break;
  case BNO_acc_range_t::G_08:
    value = (uint8_t)(thr / 15.63f);
    break;
  case BNO_acc_range_t::G_16:
    value = (uint8_t)(thr / 31.25f);
    break;
  default:
    value = 0;
    break;
  }
  write(ACC_AM_THRES_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Getting Accelerometer Interrupt settings
 *  @return Interrupt settings, my contain single or multiple items check BNO_acc_interrupt_settings_t
 */
BNO055::BNO_interrupts_t BNO055::readInterruptSource()
{
  return (BNO_interrupts_t)read(BNO055_INTR_STAT_ADDR, PAGE_ZERO);
}

/*!
 *  @brief  Getting Accelerometer Interrupt settings
 *  @return Interrupt settings, my contain single or multiple items check BNO_acc_interrupt_settings_t
 */
BNO055::BNO_acc_interrupt_settings_t BNO055::getAccelerometerInterruptSettings()
{
  return (BNO_acc_interrupt_settings_t)read(ACC_INT_SETTINGS_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Setting Accelerometer Interrupt settings
 *  @param Interrupt_Settings, my contain single or multiple items check BNO_acc_interrupt_settings_t
 */
void BNO055::setAccelerometerInterruptSettings(BNO_acc_interrupt_settings_t setting)
{
  write(ACC_INT_SETTINGS_ADDR, setting, PAGE_ONE);
}

/*!
 *  @brief  Getting Accelerometer Shock(High G) duration
 *  @return Duration
 */
uint8_t BNO055::getAccelerometerShockDuration()
{
  return read(ACC_HG_DURATION_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Setting Accelerometer Shock(High G) duration
 *  @param  duration Duration
 */
void BNO055::setAccelerometerShockDuration(uint8_t duration)
{
  write(ACC_HG_DURATION_ADDR, duration, PAGE_ONE);
}

/*!
 *  @brief  Getting Accelerometer Shock(High G) threshold
 *  @return Threshold (raw value)
 */
uint8_t BNO055::getAccelerometerShockThreshold()
{
  return read(ACC_HG_THRES_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Setting Accelerometer No/Slow motion threshold
 *  @param  threshold Threshold (raw value)
 */
void BNO055::setAccelerometerShockThreshold(uint8_t threshold)
{
  write(ACC_HG_THRES_ADDR, threshold, PAGE_ONE);
}

/*!
 *  @brief  Getting Accelerometer No/Slow motion threshold
 *  @return Threshold (raw value)
 */
uint8_t BNO055::getAccelerometerNoSlowMotionThreshold()
{
  return read(ACC_NM_THRES_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Getting Accelerometer No/Slow motion threshold
 *  @return Threshold in mg scale
 */
float BNO055::getAccelerometerNoSlowMotionThreshold_mg()
{
  BNO_acc_range_t range = getAccelerometerRange();
  uint8_t value = read(ACC_NM_THRES_ADDR, PAGE_ONE);
  switch (range)
  {
  case BNO_acc_range_t::G_02:
    return value * 3.91f;

  case BNO_acc_range_t::G_04:
    return value * 7.81f;

  case BNO_acc_range_t::G_08:
    return value * 15.63f;

  case BNO_acc_range_t::G_16:
    return value * 31.25f;

  default:
    return 0;
  }
}

/*!
 *  @brief  Setting Accelerometer No/Slow motion threshold
 *  @param  threshold Threshold (raw value)
 */
void BNO055::setAccelerometerNoSlowMotionThreshold(uint8_t thr)
{
  write(ACC_NM_THRES_ADDR, thr, PAGE_ONE);
}

/*!
 *  @brief  Setting Accelerometer No/Slow motion threshold
 *  @param  threshold Threshold (in mg)
 */
void BNO055::setAccelerometerNoSlowMotionThreshold(float thr)
{
  BNO_acc_range_t range = getAccelerometerRange();
  uint8_t value;
  switch (range)
  {
  case BNO_acc_range_t::G_02:
    value = (uint8_t)(thr / 3.91f);
    break;
  case BNO_acc_range_t::G_04:
    value = (uint8_t)(thr / 7.81f);
    break;
  case BNO_acc_range_t::G_08:
    value = (uint8_t)(thr / 15.63f);
    break;
  case BNO_acc_range_t::G_16:
    value = (uint8_t)(thr / 31.25f);
    break;
  default:
    value = 0;
    break;
  }
  write(ACC_NM_THRES_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Getting true if BNO055 is set in the No Motion mode
 *  @return True is No Motion mode
 */
bool BNO055::isNoMotionModeActive()
{
  return (read(ACC_NM_SET_ADDR, PAGE_ONE) & 0x01) ? false : true;
}

/*!
 *  @brief  Getting true if BNO055 is set in the Slow Motion mode
 *  @return True is Slow Motion mode
 */
bool BNO055::isSlowMotionModeActive()
{
  return (read(ACC_NM_SET_ADDR, PAGE_ONE) & 0x01) ? true : false;
}

/*!
 *  @brief  Setting the BNO055 into the No Motion mode
 */
void BNO055::activateNoMotionMode()
{
  write(ACC_NM_SET_ADDR, read(ACC_NM_SET_ADDR, PAGE_ONE) & 0xFE, PAGE_ONE);
}

/*!
 *  @brief  Setting the BNO055 into the No Motion mode
 */
void BNO055::activateSlowMotionMode()
{
  write(ACC_NM_SET_ADDR, read(ACC_NM_SET_ADDR, PAGE_ONE) | 0x01, PAGE_ONE);
}

/*!
 *  @brief  Getting the raw value of No/Slow Motion duration set in the register
 *  @return No/Slow motion duration (raw value)
 */
uint8_t BNO055::getNoSlowMotionDuration()
{
  return (read(ACC_NM_SET_ADDR, PAGE_ONE) >> 1);
}

/*!
 *  @brief  Getting the duration of No/Slow Motion set in the register in seconds
 *  @return No/Slow motion duration in seconds. Returns 0 if the invalid duration is set
 */
uint16_t BNO055::getNoSlowMotionDurationInSec()
{
  uint8_t dur = read(ACC_NM_SET_ADDR, PAGE_ONE) >> 1;
  if (dur < 16)
    return dur + 1;
  if (dur < 22)
    return (dur - 16) * 8 + 40;
  if (dur < 32)
    return 0;
  if (dur < 64)
    return (dur - 32) * 8 + 88;
  return 0;
}

/*!
 *  @brief  Setting the raw value of No/Slow Motion duration
 *  @param duration Duration (0x00 - 0x3F)
 * (!) Be aware these ranges are ignored (not supported):
 *  More than 63 (0x3F), 22-31 (0x16-1F)
 */
void BNO055::setNoSlowMotionDuration(uint8_t duration)
{
  if (duration > 63)
    return;
  if (duration > 21 && duration < 32)
    return;

  write(ACC_NM_SET_ADDR, (duration << 1) & (read(ACC_NM_SET_ADDR, PAGE_ONE) & 0x01), PAGE_ONE);
}

/*!
 *  @brief  Setting the No/Slow Motion duration in seconds and return the actuall value is set
 *  @param duration in seconds
 * (!) Be aware if recieved value is not supported the method will set it the highest possible value
 */
uint16_t BNO055::setNoSlowMotionDuration(uint16_t duration)
{
  uint8_t no_slow_motion_bit = read(ACC_NM_SET_ADDR, PAGE_ONE) & 0x01;

  setAccelerometerNoSlowMotionSetting(no_slow_motion_bit == 0x01, duration);

  return getNoSlowMotionDurationInSec();
}

/*!
 *  @brief  Getting the Accelerometer No/Slow motion settings
 *  @return No/Slow motion settings (raw value)
 */
uint8_t BNO055::getAccelerometerNoSlowMotionSetting()
{
  return read(ACC_NM_SET_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Setting the raw value of No/Slow Motion duration (0x00 - 0x3F) & No/Slow motion mode
 *  @param  isSlowMode true if slow mode and false if no motion mode
 *  @param  duration (raw value)
 * (!) Be aware the following duration ranges are ignored (not supported):
 * More than 63 (0x3F), 22-31 (0x16-1F)
 */
void BNO055::setAccelerometerNoSlowMotionSetting(bool isSlowMode, uint8_t duration)
{
  if ((duration > 63) || (duration > 21 && duration < 32))
  {
    if (isSlowMode)
      activateSlowMotionMode();
    else
      activateNoMotionMode();
    return;
  }

  if (isSlowMode)
    write(ACC_NM_SET_ADDR, (duration << 1) | 0x01, PAGE_ONE);
  else
    write(ACC_NM_SET_ADDR, (duration << 1) & 0xFE, PAGE_ONE);
}

/*!
 *  @brief  Setting the No/Slow Motion duration in seconds and No/Slow Motion mode
 *  @param  isSlowMode true if slow mode and false if no motion mode
 *  @param  duration in seconds (0-336)
 * (!) Be aware if recieved value is not supported the method will set it the highest possible value
 */
void BNO055::setAccelerometerNoSlowMotionSetting(bool isSlowMode, uint16_t duration)
{
  uint8_t no_slow_motion_bit = isSlowMode ? 0x01 : 0x00;
  if (duration > 336)
    write(ACC_NM_SET_ADDR, (63 << 1) & no_slow_motion_bit, PAGE_ONE);
  if (duration == 0)
    write(ACC_NM_SET_ADDR, no_slow_motion_bit, PAGE_ONE);
  else if (duration < 17)
    write(ACC_NM_SET_ADDR, ((uint8_t)(duration - 1) << 1) & no_slow_motion_bit, PAGE_ONE);
  else if (duration < 81)
    write(ACC_NM_SET_ADDR, (((uint8_t)ceil((duration - 40) / 8.0) + 16) << 1) & no_slow_motion_bit, PAGE_ONE);
  else
    write(ACC_NM_SET_ADDR, (((uint8_t)ceil((duration - 88) / 8.0) + 32) << 1) & no_slow_motion_bit, PAGE_ONE);
}

/*!
 *  @brief  Getting Gyroscope Interrupt settings
 *  @return Interrupt settings, my contain single or multiple items check BNO_gyr_interrupt_settings_t
 */
BNO055::BNO_gyr_interrupt_settings_t BNO055::getGyroscopeInterruptSettings()
{
  return (BNO_gyr_interrupt_settings_t)read(GYR_INT_SETTING_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Setting Gyroscope Interrupt settings
 *  @param Interrupt_Settings, my contain single or multiple items check BNO_gyr_interrupt_settings_t
 */
void BNO055::setGyroscopeInterruptSettings(BNO_gyr_interrupt_settings_t setting)
{
  write(GYR_INT_SETTING_ADDR, setting, PAGE_ONE);
}

/*!
 *  @brief  Getting Gyroscope High Rate Settings
 *  @return raw value of High rate Settings
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
uint8_t BNO055::getGyroscopeHighRateSettings(BNO_axis_t axis)
{
  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    return read(GYR_HR_X_SET_ADDR, PAGE_ONE);
  case BNO_axis_t::AXIS_Y:
    return read(GYR_HR_Y_SET_ADDR, PAGE_ONE);
  case BNO_axis_t::AXIS_Z:
    return read(GYR_HR_Z_SET_ADDR, PAGE_ONE);
  default:
    return 0;
  }
}

/*!
 *  @brief  Getting Gyroscope High Rate Hysteresis Threshold
 *  @return raw value of High rate hysteresis threshold 
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
uint8_t BNO055::getGyroscopeHighRateHysteresisThreshold(BNO_axis_t axis)
{
  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    return (read(GYR_HR_X_SET_ADDR, PAGE_ONE) & 0x60) >> 5;
  case BNO_axis_t::AXIS_Y:
    return (read(GYR_HR_Y_SET_ADDR, PAGE_ONE) & 0x60) >> 5;
  case BNO_axis_t::AXIS_Z:
    return (read(GYR_HR_Z_SET_ADDR, PAGE_ONE) & 0x60) >> 5;
  default:
    return 0;
  }
}

/*!
 *  @brief  Getting Gyroscope High Rate Hysteresis Threshold
 *  @return High rate hysteresis threshold  in d/s scale
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
float BNO055::getGyroscopeHighRateHysteresisThreshold_dps(BNO_axis_t axis)
{
  switch (getGyroscopeRange())
  {
  case BNO_gyr_range_t::dps_125:
    return (255 + 256 * getGyroscopeHighRateHysteresisThreshold(axis)) * 4 * 3.91f;

  case BNO_gyr_range_t::dps_250:
    return (255 + 256 * getGyroscopeHighRateHysteresisThreshold(axis)) * 4 * 7.81f;

  case BNO_gyr_range_t::dps_500:
    return (255 + 256 * getGyroscopeHighRateHysteresisThreshold(axis)) * 4 * 15.625f;

  case BNO_gyr_range_t::dps_1000:
    return (255 + 256 * getGyroscopeHighRateHysteresisThreshold(axis)) * 4 * 31.25f;

  case BNO_gyr_range_t::dps_2000:
    return (255 + 256 * getGyroscopeHighRateHysteresisThreshold(axis)) * 4 * 62.5f;

  default:
    return 0;
  }
}

/*!
 *  @brief  Getting Gyroscope High Rate Threshold
 *  @return raw value of High rate threshold 
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
uint8_t BNO055::getGyroscopeHighRateThreshold(BNO_axis_t axis)
{
  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    return (read(GYR_HR_X_SET_ADDR, PAGE_ONE) & 0x1F);
  case BNO_axis_t::AXIS_Y:
    return (read(GYR_HR_Y_SET_ADDR, PAGE_ONE) & 0x1F);
  case BNO_axis_t::AXIS_Z:
    return (read(GYR_HR_Z_SET_ADDR, PAGE_ONE) & 0x1F);
  default:
    return 0;
  }
}

/*!
 *  @brief  Getting Gyroscope High Rate Threshold
 *  @return High rate threshold in d/s scale
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
float BNO055::getGyroscopeHighRateThreshold_dps(BNO_axis_t axis)
{
  switch (getGyroscopeRange())
  {
  case BNO_gyr_range_t::dps_125:
    return getGyroscopeHighRateThreshold(axis) * 3.91f;

  case BNO_gyr_range_t::dps_250:
    return getGyroscopeHighRateThreshold(axis) * 7.81f;

  case BNO_gyr_range_t::dps_500:
    return getGyroscopeHighRateThreshold(axis) * 15.625f;

  case BNO_gyr_range_t::dps_1000:
    return getGyroscopeHighRateThreshold(axis) * 31.25f;

  case BNO_gyr_range_t::dps_2000:
    return getGyroscopeHighRateThreshold(axis) * 62.5f;

  default:
    return 0;
  }
}

/*!
 *  @brief  Getting Gyroscope High Rate Duration
 *  @return raw value of High rate duration
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
uint8_t BNO055::getGyroscopeHighRateDuration(BNO_axis_t axis)
{
  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    return read(GYR_DUR_X_ADDR, PAGE_ONE);
  case BNO_axis_t::AXIS_Y:
    return read(GYR_DUR_Y_ADDR, PAGE_ONE);
  case BNO_axis_t::AXIS_Z:
    return read(GYR_DUR_Z_ADDR, PAGE_ONE);
  default:
    return 0;
  }
}

/*!
 *  @brief  Getting Gyroscope High Rate Duration (ms)
 *  @return High rate duration in mili seconds scale
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
float BNO055::getGyroscopeHighRateDuration_ms(BNO_axis_t axis)
{
  return (1 + getGyroscopeHighRateDuration(axis)) * 2.5f;
}

/*!
 *  @brief  Setting Gyroscope High Rate Settings
 *  @param value raw value of Gyroscope High Rate settings
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateSettings(BNO_axis_t axis, uint8_t value)
{
  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Y:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Z:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  }
}

/*!
 *  @brief  Setting Gyroscope High Rate Hysteresis Threshold
 *  @param thr raw value of Gyroscope High Rate Hysteresis Threshold. 
 * if thr is more than 3 it is ignored.
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateHysteresisThreshold(BNO_axis_t axis, uint8_t thr)
{
  if (thr > 3)
    return;
  uint8_t value = (getGyroscopeHighRateSettings(axis) & 0x9F) | (thr << 5);

  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Y:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Z:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  }
}

/*!
 *  @brief  Setting Gyroscope High Rate Hysteresis Threshold (d/s)
 *  @param thr Gyroscope High Rate Hysteresis Threshold in d/s
 * if thr is more than accepted value, maxium value is used.
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateHysteresisThreshold_dps(BNO_axis_t axis, float thr)
{
  BNO_gyr_range_t range = getGyroscopeRange();
  uint8_t value;
  switch (range)
  {
  case BNO_gyr_range_t::dps_125:
    value = ((thr / (3.91f * 4)) - 255) / 256;
    break;

  case BNO_gyr_range_t::dps_250:
    value = ((thr / (7.81f * 4)) - 255) / 256;
    break;

  case BNO_gyr_range_t::dps_500:
    value = ((thr / (15.56f * 4)) - 255) / 256;
    break;

  case BNO_gyr_range_t::dps_1000:
    value = ((thr / (31.25f * 4)) - 255) / 256;
    break;

  case BNO_gyr_range_t::dps_2000:
    value = ((thr / (62.5f * 4)) - 255) / 256;
    break;
  default:
    value = 0;
    break;
  }

  if (value > 3)
    value = 3;
  setGyroscopeHighRateHysteresisThreshold(axis, value);
}

/*!
 *  @brief  Setting Gyroscope High Rate Threshold
 *  @param thr raw value of Gyroscope High Rate Threshold. 
 * if thr is more than 31 (0x1F) it is ignored.
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateThreshold(BNO_axis_t axis, uint8_t thr)
{
  if (thr > 0x1f)
    return;
  uint8_t value = (getGyroscopeHighRateSettings(axis) & 0xE0) | thr;

  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Y:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Z:
    write(GYR_HR_X_SET_ADDR, value, PAGE_ONE);
    break;
  }
}

/*!
 *  @brief  Setting Gyroscope High Rate Threshold in d/s
 *  @param thr Gyroscope High Rate Threshold in d/s scale
 * if thr is more than accepted value, maxium value is used.
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateThreshold_dps(BNO_axis_t axis, float thr)
{
  BNO_gyr_range_t range = getGyroscopeRange();
  uint8_t value;
  switch (range)
  {
  case BNO_gyr_range_t::dps_125:
    value = thr / 3.91f;
    break;

  case BNO_gyr_range_t::dps_250:
    value = thr / 7.81f;
    break;

  case BNO_gyr_range_t::dps_500:
    value = thr / 15.56f;
    break;

  case BNO_gyr_range_t::dps_1000:
    value = thr / 31.25f;
    break;

  case BNO_gyr_range_t::dps_2000:
    value = thr / 62.5f;
    break;
  default:
    value = 0;
    break;
  }

  if (value > 0x1F)
    value = 0x1F;
  setGyroscopeHighRateThreshold(axis, value);
}

/*!
 *  @brief  Setting Gyroscope High Rate duration
 *  @param dur raw value of High Rate Duration
 * if thr is more than accepted value, maxium value is used.
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateDuration(BNO_axis_t axis, uint8_t dur)
{
  switch (axis)
  {
  case BNO_axis_t::AXIS_X:
    write(GYR_DUR_X_ADDR, dur, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Y:
    write(GYR_DUR_Y_ADDR, dur, PAGE_ONE);
    break;
  case BNO_axis_t::AXIS_Z:
    write(GYR_DUR_Z_ADDR, dur, PAGE_ONE);
    break;
  }
}

/*!
 *  @brief  Setting Gyroscope High Rate duration in ms
 *  @param dur High Rate Duration in mili seconds (scale)
 * if thr is more than accepted value, maxium value is used.
 * if thr is less than accepted value, minimum value is used.
 *  @param axis target axis can be X,Y, or Z. check BNO_axis_t for more detail
 */
void BNO055::setGyroscopeHighRateDuration_ms(BNO_axis_t axis, float dur)
{
  uint8_t val;
  if ((dur / 2.5f) - 1 < 0)
    val = 0;
  else
    val = (dur / 2.5f) - 1;
  setGyroscopeHighRateDuration(axis, val);
}

/*!
 *  @brief  Getting Gyroscope Any Motion Threshold
 *  @return raw value of Any Motion Threshold
 */
uint8_t BNO055::getGyroscopeAnyMotionThreshold()
{
  return read(GYR_AM_THRES_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Getting Gyroscope Any Motion Threshold (d/s)
 *  @return Any Motion Threshold in d/s scale
 */
float BNO055::getGyroscopeAnyMotionThreshold_dps()
{
  BNO_gyr_range_t range = getGyroscopeRange();
  switch (range)
  {
  case BNO_gyr_range_t::dps_125:
    return getGyroscopeAnyMotionThreshold() * 0.0625f;

  case BNO_gyr_range_t::dps_250:
    return getGyroscopeAnyMotionThreshold() * 0.125f;

  case BNO_gyr_range_t::dps_500:
    return getGyroscopeAnyMotionThreshold() * 0.25f;

  case BNO_gyr_range_t::dps_1000:
    return getGyroscopeAnyMotionThreshold() * 0.5f;

  case BNO_gyr_range_t::dps_2000:
    return getGyroscopeAnyMotionThreshold() * 1.0f;

  default:
    return -1; // this should never happen
  }
}

/*!
 *  @brief  Getting Gyroscope Any Motion Threshold
 *  @param thr raw value of Any Motion Threshold.
 * if value is bigger than 0x7F it will fall back to the maximum value 0x7F
 */
void BNO055::setGyroscopeAnyMotionThreshold(uint8_t thr)
{
  if (thr > 0x7f)
    thr = 0x7F;
  write(GYR_AM_THRES_ADDR, thr, PAGE_ONE);
}

/*!
 *  @brief  Getting Gyroscope Any Motion Threshold
 *  @param thr raw value of Any Motion Threshold.
 * if value is bigger than maximum allowed it will fall back to the maximum value
 */
void BNO055::setGyroscopeAnyMotionThreshold(float thr)
{
  BNO_gyr_range_t range = getGyroscopeRange();
  uint8_t value;
  switch (range)
  {
  case BNO_gyr_range_t::dps_125:
    value = thr / 0.0625f;
    break;

  case BNO_gyr_range_t::dps_250:
    value = thr / 0.125f;
    break;

  case BNO_gyr_range_t::dps_500:
    value = thr / 0.25f;
    break;

  case BNO_gyr_range_t::dps_1000:
    value = thr / 0.5f;
    break;

  case BNO_gyr_range_t::dps_2000:
    value = thr;
    break;

  default:
    value = 0;
    break;
  }
  setGyroscopeAnyMotionThreshold(value);
}

/*!
 *  @brief  Getting Gyroscope Any Motion Settings
 *  @return raw value of Any Motion Settings
 */
uint8_t BNO055::getGyroscopeAnyMotionSettings()
{
  return read(GYR_AM_SET_ADDR, PAGE_ONE);
}

/*!
 *  @brief  Getting Gyroscope Any Motion Awake Duration
 *  @return Awake duration value, for more detail please check Gyr_any_motion_awake_duration_t
 */
BNO055::Gyr_any_motion_awake_duration_t BNO055::getGyroscopeAnyMotionAwakeDuration()
{
  return (Gyr_any_motion_awake_duration_t)((getGyroscopeAnyMotionSettings() >> 2) & 0x03);
}

/*!
 *  @brief  Getting Gyroscope Any Motion Slope Samples
 *  @return Slop sample. Any motion interrupts triggers if [slope sample + 1] * 4 consecutive data points
 * are above the any motion interrupt threshold.
 */
uint8_t BNO055::getGyroscopeAnyMotionSlopeSamples()
{
  return (getGyroscopeAnyMotionSettings() & 0x03);
}

/*!
 *  @brief  Getting Gyroscope Any Motion settings
 *  @param value raw value of Any Motion settings.
 * if value is bigger than 0x0F (not supported!), writting the settings is ignored.
 */
void BNO055::setGyroscopeAnyMotionSettings(uint8_t value)
{
  if (value > 0x0F)
    return;
  write(GYR_AM_SET_ADDR, value, PAGE_ONE);
}

/*!
 *  @brief  Setting Gyroscope Any Motion Awake Duration
 *  @param thr Awake duration. for more details check Gyr_any_motion_awake_duration_t
 * if equvalent value of thr is more than 0x03 (not supported), the settings while be ignored.
 */
void BNO055::setGyroscopeAnyMotionAwakeDuration(Gyr_any_motion_awake_duration_t dur)
{
  if (dur > 0x03)
    return;
  uint8_t value = getGyroscopeAnyMotionSettings() & 0xF3;
  setGyroscopeAnyMotionSettings(value | (dur << 2));
}

/*!
 *  @brief  Setting Gyroscope Any Motion Slope Samples
 *  @param slope_samples Slope Samples. Any motion interrupts triggers if [slope sample + 1] * 4 
 * consecutive data points are above the any motion interrupt threshold.
 * if slope_samples is more than 0x03 (not supported), the settings while be ignored.
 */
void BNO055::setGyroscopeAnyMotionSlopeSamples(uint8_t slope_samples)
{
  if (slope_samples > 0x03)
    return;
  uint8_t value = getGyroscopeAnyMotionSettings() & 0xFC;
  setGyroscopeAnyMotionSettings(value | slope_samples);
}

// ! Internal methods

/*!
 *  @brief writing the PageId before reading or writing any registery
 *  @param pageId Page Id for accessing the the registaries
 */
bool BNO055::writePageId(uint8_t pageId)
{
  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)BNO055_PAGE_ID_ADDR);
  _wire->write((uint8_t)pageId);
#else
  _wire->send(BNO055_PAGE_ID_ADDR);
  _wire->send(pageId);
#endif
  _wire->endTransmission();

  /* ToDo: Check for error! */
  return true;
}

/*!
 *  @brief  Writes an 8 bit value over I2C with masking
 *  @param reg registry addresss
 *  @param value value will be written on the address
 *  @param mask mask is AND with the current value
 *  @param pageId Page Id. Default value for the PageId is page zero 
 */
bool BNO055::maskwrite(BNO055_reg_t _register, byte value, byte mask, uint8_t pageId)
{
  if (!writePageId(pageId))
    return false;
  uint8_t val = (read(_register) & mask) | value;

  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)_register);
  _wire->write((uint8_t)value);
#else
  _wire->send(reg);
  _wire->send(value);
#endif
  _wire->endTransmission();

  return true;
}

/*!
 *  @brief  Sets bit in a register
 *  @param _register registry addresss
 *  @param _bit bit position has to be set
 *  @param pageId Page Id. Default value for the PageId is page zero 
 */
bool BNO055::set(BNO055_reg_t _register, const uint8_t &_bit, uint8_t pageId)
{
  uint8_t value = read(_register, pageId);
  value |= (1 << _bit);
  return write(_register, value, pageId);
}

/*!
 *  @brief  Sets bit in a register
 *  @param _register registry addresss
 *  @param _bit bit position has to be set
 *  @param pageId Page Id. Default value for the PageId is page zero 
 */
bool BNO055::unset(BNO055_reg_t _register, const uint8_t &_bit, uint8_t pageId)
{
  uint8_t value = read(_register, pageId);
  value &= ~(1 << _bit);
  return write(_register, value, pageId);
}

/*!
 *  @brief  Writes an 8 bit value over I2C
 *  @param reg registry addresss
 *  @param value value will be written on the address
 *  @param pageId Page Id. Default value for the PageId is page zero 
 */
bool BNO055::write(BNO055_reg_t reg, byte value, uint8_t pageId)
{

  if (!writePageId(pageId))
    return false;

  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)value);
#else
  _wire->send(reg);
  _wire->send(value);
#endif
  _wire->endTransmission();

  /* ToDo: Check for error! */
  return true;
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 *  @param reg registry addresss
 *  @param pageId Page Id. Default value for the PageId is page zero 
 */
byte BNO055::read(BNO055_reg_t reg, uint8_t pageId)
{
  byte value = 0;

  if (!writePageId(pageId))
    return false;

  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)reg);
#else
  _wire->send(reg);
#endif
  _wire->endTransmission();
  _wire->requestFrom(_address, (byte)1);
#if ARDUINO >= 100
  value = _wire->read();
#else
  value = _wire->receive();
#endif

  return value;
}

/*!
 *  @brief  Reads the specified number of bytes over I2C
 * Default value for the PageId is page zero 
 */
bool BNO055::read(BNO055_reg_t reg, byte *buffer,
                  uint8_t len, uint8_t pageId)
{
  if (!writePageId(pageId))
    return false;

  _wire->beginTransmission(_address);
#if ARDUINO >= 100
  _wire->write((uint8_t)reg);
#else
  _wire->send(reg);
#endif
  _wire->endTransmission();
  _wire->requestFrom(_address, (byte)len);

  for (uint8_t i = 0; i < len; i++)
  {
#if ARDUINO >= 100
    buffer[i] = _wire->read();
#else
    buffer[i] = _wire->receive();
#endif
  }

  /* ToDo: Check for errors! */
  return true;
}
