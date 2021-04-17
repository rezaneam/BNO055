/*!
 *  @file BNO055.h
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef __BNO055_H__
#define __BNO055_H__

#include "Arduino.h"
#include <Wire.h>

#include "utility/imumaths.h"

/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)

/** A structure to represent offsets **/
typedef struct
{
  int16_t accel_offset_x; /**< x acceleration offset */
  int16_t accel_offset_y; /**< y acceleration offset */
  int16_t accel_offset_z; /**< z acceleration offset */

  int16_t mag_offset_x; /**< x magnetometer offset */
  int16_t mag_offset_y; /**< y magnetometer offset */
  int16_t mag_offset_z; /**< z magnetometer offset */

  int16_t gyro_offset_x; /**< x gyroscrope offset */
  int16_t gyro_offset_y; /**< y gyroscrope offset */
  int16_t gyro_offset_z; /**< z gyroscrope offset */

  int16_t accel_radius; /**< acceleration radius */

  int16_t mag_radius; /**< magnetometer radius */
} BNO055_offsets_t;

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          BNO055 Sensor
 */
class BNO055
{
public:
  /** BNO055 Registers **/
  typedef enum
  {
    /* Page ID */
    PAGE_ZERO = 0x00,
    PAGE_ONE = 0x01,

    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,

    // ! * PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,       // Default value is 0xA0  BNO055 Chip ID
    BNO055_ACCEL_REV_ID_ADDR = 0x01,  // Default value is 0xFB  Accelerometer Chip ID
    BNO055_MAG_REV_ID_ADDR = 0x02,    // Default value is 0x32  Magnetometer Chip ID
    BNO055_GYRO_REV_ID_ADDR = 0x03,   // Default value is 0x0F  Gyroscope Chip ID
    BNO055_SW_REV_ID_LSB_ADDR = 0x04, // Default value is 0x11  Software Revision Minor Version
    BNO055_SW_REV_ID_MSB_ADDR = 0x05, // Default value is 0x03  Software Revision Major Version
    BNO055_BL_REV_ID_ADDR = 0X06,     // Default value is 0x15  Bootloader Version

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08, // Default value is 0x00 Acceleration Data X LSB
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09, // Default value is 0x00 Acceleration Data X MSB
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A, // Default value is 0x00 Acceleration Data Y LSB
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B, // Default value is 0x00 Acceleration Data Y MSB
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C, // Default value is 0x00 Acceleration Data Z LSB
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D, // Default value is 0x00 Acceleration Data Z MSB

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E, // Default value is 0x00 Magnetometer Data X LSB
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F, // Default value is 0x00 Magnetometer Data X MSB
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10, // Default value is 0x00 Magnetometer Data Y LSB
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11, // Default value is 0x00 Magnetometer Data Y MSB
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12, // Default value is 0x00 Magnetometer Data Z LSB
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13, // Default value is 0x00 Magnetometer Data Z MSB

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14, // Default value is 0x00 Gyroscope Data X LSB
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15, // Default value is 0x00 Gyroscope Data X MSB
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16, // Default value is 0x00 Gyroscope Data Y LSB
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17, // Default value is 0x00 Gyroscope Data Y MSB
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18, // Default value is 0x00 Gyroscope Data Z LSB
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19, // Default value is 0x00 Gyroscope Data Z MSB

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR = 0X1A, // Default value is 0x00 Euler Heading Angle LSB
    BNO055_EULER_H_MSB_ADDR = 0X1B, // Default value is 0x00 Euler Heading Angle MSB
    BNO055_EULER_R_LSB_ADDR = 0X1C, // Default value is 0x00 Euler Roll Angle LSB
    BNO055_EULER_R_MSB_ADDR = 0X1D, // Default value is 0x00 Euler Roll Angle MSB
    BNO055_EULER_P_LSB_ADDR = 0X1E, // Default value is 0x00 Euler Pitch Angle LSB
    BNO055_EULER_P_MSB_ADDR = 0X1F, // Default value is 0x00 Euler Pitch Angle MSB

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20, // Default value is 0x00 Quaternion W LSB
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21, // Default value is 0x00 Quaternion W MSB
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22, // Default value is 0x00 Quaternion X LSB
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23, // Default value is 0x00 Quaternion X MSB
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24, // Default value is 0x00 Quaternion Y LSB
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25, // Default value is 0x00 Quaternion Y MSB
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26, // Default value is 0x00 Quaternion Z LSB
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27, // Default value is 0x00 Quaternion Z MSB

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28, // Default value is 0x00 Linear Acceleration X LSB
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29, // Default value is 0x00 Linear Acceleration X MSB
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A, // Default value is 0x00 Linear Acceleration Y LSB
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B, // Default value is 0x00 Linear Acceleration Y MSB
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C, // Default value is 0x00 Linear Acceleration Z LSB
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D, // Default value is 0x00 Linear Acceleration Z MSB

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E, // Default value is 0x00 Gravity Vector X LSB
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F, // Default value is 0x00 Gravity Vector X MSB
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30, // Default value is 0x00 Gravity Vector Y LSB
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31, // Default value is 0x00 Gravity Vector Y MSB
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32, // Default value is 0x00 Gravity Vector Z LSB
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33, // Default value is 0x00 Gravity Vector Z MSB

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0X34, // Default value is 0x00 Temperture

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0X35, // Default value is 0x00 Calibration Status for System [7,6], Gyroscope [5,4], Accelerometer [3,2], Magnetometer [1,0]
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0X3B,
    BNO055_DATA_SELECT_ADDR = 0X3C,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0X5B,
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0X61,
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A,

    // ! * PAGE1 REGISTER DEFINITION START*/
    ACC_CONFIG_ADDR = 0x08,       // Default value is 0x0D => 4G, Normal, 62.5Hz
    MAG_CONFIG_ADDR = 0x09,       // Default value is 0x0B => Normal, Regular, 10Hz
    GYR_CONFIG_0_ADDR = 0x0A,     // Default value is 0x38 => 2000dps, Normal, 32Hz
    GYR_CONFIG_1_ADDR = 0x0B,     // Default value is 0x00
    ACC_SLEEP_CONFIG_ADDR = 0x0C, // Default value is 0x00
    GYR_SLEEP_CONFIG_ADDR = 0x0D, // Default value is 0x00
    INT_MASK_ADDR = 0x0F,         // Default value is 0x00
    INT_EN_ADDR = 0x10,           // Default value is 0x00
    ACC_AM_THRES_ADDR = 0x11,     // Default value is 0x14 => 78.2mg/156.2mg/312.6mg/625mg for 2G, 4G, 8G, 16G ranges
    ACC_INT_SETTINGS_ADDR = 0x12, // Default value is 0x03 => Any motion duration 4 decimal
    ACC_HG_DURATION_ADDR = 0x13,  // Default value is 0x0F
    ACC_HG_THRES_ADDR = 0x14,     // Default value is 0xC0
    ACC_NM_THRES_ADDR = 0x15,     // Default value is 0x0A
    ACC_NM_SET_ADDR = 0x16,       // Default value is 0x0B
    GYR_INT_SETTING_ADDR = 0x17,  // Default value is 0x00
    GYR_HR_X_SET_ADDR = 0x18,     // Default value is 0x01
    GYR_DUR_X_ADDR = 0x19,        // Default value is 0x19
    GYR_HR_Y_SET_ADDR = 0x1A,     // Default value is 0x01
    GYR_DUR_Y_ADDR = 0x1B,        // Default value is 0x19
    GYR_HR_Z_SET_ADDR = 0x1C,     // Default value is 0x01
    GYR_DUR_Z_ADDR = 0x1D,        // Default value is 0x19
    GYR_AM_THRES_ADDR = 0x1E,     // Default value is 0x04
    GYR_AM_SET_ADDR = 0x1F,       // Default value is 0x0A

  } BNO055_reg_t;

  /** BNO055 power settings */
  typedef enum
  {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
  } BNO055_powermode_t;

  /** Operation mode settings **/
  typedef enum
  {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
  } BNO055_opmode_t;

  /** Remap settings **/
  typedef enum
  {
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
  } BNO055_axis_remap_config_t;

  /** Remap Signs **/
  typedef enum
  {
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
  } BNO055_axis_remap_sign_t;

  /** Accelerometer configuration **/
  typedef enum
  {
    G_02 = 0b00,
    G_04 = 0x01,
    G_08 = 0b10,
    G_16 = 0b11
  } BNO_acc_range_t;

  typedef enum
  {
    Hz_7_81 = 0b000,
    Hz_15_63 = 0b001,
    Hz_31_25 = 0b010,
    Hz_62_50 = 0b011,
    Hz_125 = 0b100,
    Hz_250 = 0b101,
    Hz_500 = 0b110,
    Hz_1000 = 0b111
  } BNO_acc_BW_t;

  typedef enum
  {
    ACC_NORMAL = 0b000,
    ACC_SUSPEND = 0b001,
    ACC_LOW_POWER_1 = 0b010,
    ACC_STANDBY = 0b011,
    ACC_LOW_POWER_2 = 0b100,
    ACC_DEEP_SUSPEND = 0b101
  } BNO_acc_operation_mode_t;

  /** Magnetometer configuration **/
  typedef enum
  {
    Hz_2 = 0b000,
    Hz_6 = 0b001,
    Hz_8 = 0b010,
    Hz_10 = 0b011,
    Hz_15 = 0b100,
    Hz_20 = 0b101,
    Hz_25 = 0b110,
    Hz_30 = 0b111
  } BNO_mag_BW_t;

  typedef enum
  {
    MAG_LOW_POWER = 0b00,
    MAG_REGULAR = 0b01,
    MAG_ENHANCED_REGULAR = 0b10,
    MAG_HIGH_ACCURACY = 0b11
  } BNO_mag_operation_mode_t;

  typedef enum
  {
    MAG_NORMAL = 0b00,
    MAG_SLEEP = 0b01,
    MAG_SUSPEND = 0b10,
    MAG_FORCE_MODE = 0b11
  } BNO_mag_power_mode_t;

  /** Gyroscope configuration **/
  typedef enum
  {
    dps_2000 = 0b000,
    dps_1000 = 0b001,
    dps_500 = 0b010,
    dps_250 = 0b011,
    dps_125 = 0b100
  } BNO_gyr_range_t;

  typedef enum
  {
    Hz_523 = 0b000,
    Hz_230 = 0b001,
    Hz_116 = 0b010,
    Hz_47 = 0b011,
    Hz_23 = 0b100,
    Hz_12 = 0b101,
    Hz_64 = 0b110,
    Hz_32 = 0b111
  } BNO_gyr_BW_t;

  typedef enum
  {
    GYR_NORMAL = 0b000,
    GYR_FAST_POWER_UP = 0b001,
    GYR_DEEP_SUSPEND = 0b010,
    GYR_SUSPEND = 0b011,
    GYR_ADVANCED_POWER_SAVE = 0b100,
  } BNO_gyr_operation_mode_t;

  /** Interrupts **/
  typedef enum
  {
    ACCELEROMETER_NO_MOTION = 0x80,
    ACCELEROMETER_ANY_MOTION = 0x40,
    ACCELEROMETER_HIGH_G = 0x20,
    GYROSCOPE_DATA_READY = 0x10,
    GYROSCOPE_HIGH_RATE = 0x08,
    GYROSCOPE_ANY_MOTION = 0x04,
    MAGNETOMETER_DATA_READY = 0x02,
    ACCELEROMETER_BSX_DATA_READY = 0x01
  } BNO_interrupts_t;

  typedef enum
  {
    ACC_HIGH_G_Z_AXIS = 0x80,
    ACC_HIGH_G_Y_AXIS = 0x40,
    ACC_HIGH_G_X_AXIS = 0x20,
    ACC_ANY_OR_NO_MOTION_Z_AXIS = 0x10,
    ACC_ANY_OR_NO_MOTION_Y_AXIS = 0x08,
    ACC_ANY_OR_NO_MOTION_X_AXIS = 0x04,
    ACC_ANY_MOTION_DURATION_4_DECIMAL = 0x03,
    ACC_ANY_MOTION_DURATION_3_DECIMAL = 0x02,
    ACC_ANY_MOTION_DURATION_2_DECIMAL = 0x01,
    ACC_ANY_MOTION_DURATION_1_DECIMAL = 0x00
  } BNO_acc_interrupt_settings_t;

  typedef enum
  {
    GYR_HIGH_RATE_FILTER = 0x80,
    GYR_ANY_MOTION_FILTER = 0x40,
    GYR_HIGH_RATE_Z_AXIS = 0x20,
    GYR_HIGH_RATE_Y_AXIS = 0x10,
    GYR_HIGH_RATE_X_AXIS = 0x08,
    GYR_ANY_MOTION_Z_AXIS = 0x04,
    GYR_ANY_MOTION_Y_AXIS = 0x02,
    GYR_ANY_MOTION_X_AXIS = 0x01
  } BNO_gyr_interrupt_settings_t;

  typedef enum
  {
    dur_8_samples = 0x00,
    dur_16_samples = 0x01,
    dur_32_samples = 0x02,
    dur_64_samples = 0x03
  } Gyr_any_motion_awake_duration_t;

  typedef enum
  {
    AXIS_X = 0x00,
    AXIS_Y = 0x01,
    AXIS_Z = 0x02
  } BNO_axis_t;

  /** A structure to represent revisions **/
  typedef struct
  {
    uint8_t accel_rev; /**< acceleration rev */
    uint8_t mag_rev;   /**< magnetometer rev */
    uint8_t gyro_rev;  /**< gyroscrope rev */
    uint16_t sw_rev;   /**< SW rev */
    uint8_t bl_rev;    /**< bootloader rev */
  } BNO055_rev_info_t;

  /** Vector Mappings **/
  typedef enum
  {
    SENSOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    SENSOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
    SENSOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
    SENSOR_EULER = BNO055_EULER_H_LSB_ADDR,
    SENSOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    SENSOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
  } sensor_type_t;

  BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A,
         TwoWire *theWire = &Wire);

  bool begin(BNO055_opmode_t mode = OPERATION_MODE_NDOF);
  void setMode(BNO055_opmode_t mode);
  void setAxisRemap(BNO055_axis_remap_config_t remapcode);
  void setAxisSign(BNO055_axis_remap_sign_t remapsign);
  void getRevInfo(BNO055_rev_info_t *);
  void setExtCrystalUse(boolean usextal);
  void getSystemStatus(uint8_t *system_status, uint8_t *self_test_result,
                       uint8_t *system_error);
  void getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel,
                      uint8_t *mag);

  imu::Vector<3> getVector(sensor_type_t sensor_type);
  imu::Quaternion getQuat();
  int8_t getTemp();

  /* Functions to deal with raw calibration data */
  bool getSensorOffsets(uint8_t *calibData);
  bool getSensorOffsets(BNO055_offsets_t &offsets_type);
  void setSensorOffsets(const uint8_t *calibData);
  void setSensorOffsets(const BNO055_offsets_t &offsets_type);
  bool isFullyCalibrated();

  /* Power managments functions */
  void enterSuspendMode();
  void enterNormalMode();

  /* Missing methods in page 0 */
  BNO_interrupts_t readInterruptSource();

  /* Accelerometer configuration */
  BNO_acc_range_t getAccelerometerRange();
  BNO_acc_BW_t getAccelerometerBW();
  BNO_acc_operation_mode_t getAccelerometerOperationMode();
  void setAccelerometerRange(BNO_acc_range_t range);
  void setAccelerometerBW(BNO_acc_BW_t bw);
  void setAccelerometerOperationMode(BNO_acc_operation_mode_t mode);

  /* Magnetometer configuration */
  BNO_mag_operation_mode_t getMagnetometerOperationMode();
  BNO_mag_power_mode_t getMagnetometerPowerMode();
  BNO_mag_BW_t getMagnetometerBW();
  void setMagnetometerPowerMode(BNO_mag_power_mode_t mode);
  void settMagnetometerBW(BNO_mag_BW_t bw);
  void setMagnetometerOperationMode(BNO_mag_operation_mode_t mode);

  /* Gyroscope configuration */
  BNO_gyr_range_t getGyroscopeRange();
  BNO_gyr_BW_t getGyroscopeBW();
  BNO_gyr_operation_mode_t getGyroscopeOperationMode();
  void setGyroscopeRange(BNO_gyr_range_t range);
  void setGyroscopeBW(BNO_gyr_BW_t bw);
  void setGyroscopeOperationMode(BNO_gyr_operation_mode_t mode);

  /* Interrupt */
  BNO_interrupts_t getExternalInterruptEnable();
  void setExternalInterruptEnable(BNO_interrupts_t status);
  BNO_interrupts_t getInterruptEnable();
  void setInterruptEnable(BNO_interrupts_t status);

  /* Accelerometer Interrupt configuration */
  uint8_t getAccelerometerAnyMotionThreshold();
  float getAccelerometerAnyMotionThreshold_mg();
  void setAccelerometerAnyMotionThreshold(uint8_t thr);
  void setAccelerometerAnyMotionThreshold(float thr);

  BNO_acc_interrupt_settings_t getAccelerometerInterruptSettings();
  void setAccelerometerInterruptSettings(BNO_acc_interrupt_settings_t setting);

  uint8_t getAccelerometerShockDuration();
  void setAccelerometerShockDuration(uint8_t duration);
  uint8_t getAccelerometerShockThreshold();
  void setAccelerometerShockThreshold(uint8_t threshold);

  uint8_t getAccelerometerNoSlowMotionThreshold();
  float getAccelerometerNoSlowMotionThreshold_mg();
  void setAccelerometerNoSlowMotionThreshold(uint8_t thr);
  void setAccelerometerNoSlowMotionThreshold(float thr);

  bool isNoMotionModeActive();
  bool isSlowMotionModeActive();
  void activateNoMotionMode();
  void activateSlowMotionMode();
  uint8_t getNoSlowMotionDuration();
  uint16_t getNoSlowMotionDurationInSec();
  void setNoSlowMotionDuration(uint8_t duration);
  uint16_t setNoSlowMotionDuration(uint16_t duration);
  uint8_t getAccelerometerNoSlowMotionSetting();
  void setAccelerometerNoSlowMotionSetting(bool isSlowMode, uint8_t duration);
  void setAccelerometerNoSlowMotionSetting(bool isSlowMode, uint16_t duration);

  /* Gyroscope Interrupt Configuration */
  BNO_gyr_interrupt_settings_t getGyroscopeInterruptSettings();
  void setGyroscopeInterruptSettings(BNO_gyr_interrupt_settings_t setting);

  uint8_t getGyroscopeHighRateSettings(BNO_axis_t axis);
  uint8_t getGyroscopeHighRateHysteresisThreshold(BNO_axis_t axis);
  float getGyroscopeHighRateHysteresisThreshold_dps(BNO_axis_t axis);
  uint8_t getGyroscopeHighRateThreshold(BNO_axis_t axis);
  float getGyroscopeHighRateThreshold_dps(BNO_axis_t axis);
  uint8_t getGyroscopeHighRateDuration(BNO_axis_t axis);
  float getGyroscopeHighRateDuration_ms(BNO_axis_t axis);
  void setGyroscopeHighRateSettings(BNO_axis_t axis, uint8_t value);
  void setGyroscopeHighRateHysteresisThreshold(BNO_axis_t axis, uint8_t thr);
  void setGyroscopeHighRateHysteresisThreshold_dps(BNO_axis_t axis, float thr);
  void setGyroscopeHighRateThreshold(BNO_axis_t axis, uint8_t thr);
  void setGyroscopeHighRateThreshold_dps(BNO_axis_t axis, float thr);
  void setGyroscopeHighRateDuration(BNO_axis_t axis, uint8_t thr);
  void setGyroscopeHighRateDuration_ms(BNO_axis_t axis, float thr);

  uint8_t getGyroscopeAnyMotionThreshold();
  float getGyroscopeAnyMotionThreshold_dps();
  void setGyroscopeAnyMotionThreshold(uint8_t thr);
  void setGyroscopeAnyMotionThreshold(float thr);

  uint8_t getGyroscopeAnyMotionSettings();
  Gyr_any_motion_awake_duration_t getGyroscopeAnyMotionAwakeDuration();
  uint8_t getGyroscopeAnyMotionSlopeSamples();
  void setGyroscopeAnyMotionSettings(uint8_t value);
  void setGyroscopeAnyMotionAwakeDuration(Gyr_any_motion_awake_duration_t dur);
  void setGyroscopeAnyMotionSlopeSamples(uint8_t slope_samples);

private:
  bool writePageId(uint8_t pageId);
  byte read8(BNO055_reg_t, uint8_t pageId = 0x00);
  bool readLen(BNO055_reg_t, byte *buffer, uint8_t len, uint8_t pageId = 0x00);
  bool write8(BNO055_reg_t, byte value, uint8_t pageId = 0x00);

  uint8_t _address;
  TwoWire *_wire;

  int32_t _sensorID;
  BNO055_opmode_t _mode;
};

#endif
