# [Bosch BNO055 Driver](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)

This library is written based on [Adafruit Unified BNO055 Driver](https://github.com/adafruit/Adafruit_BNO055) while redundant dependency to [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor) is removed.

<p align="center">
  <img width="800px" src="https://www.bosch-sensortec.com/media/boschsensortec/products/smart_sensors/16_19/bosch-sensortec_website-relaunch_stage_bno055_res_800x450.jpg">
</p>

## What is new in this driver

Driver is optimized for a better operation with [FreeRTOS](https://www.freertos.org/).

Moreover this library includes additional features in the `page 1` in the register map as follows.

- Reading/Writting Accelerometer Range, bandwidth, Operation Mode
- Reading/Writting Gyroscope Range, bandwidth, Operation Mode
- Reading/Writting Magnetometer Power Mode, bandwidth, Operation Mode
- Reading/Writting Interrupt Mask
- Reading/Writting Interrupt Activation status
- Reading Interrupt source
- Reading/Writting Accelerometer Any Motion/No Motion Threshold
- Reading/Writting Accelerometer Interrupt Settings
- Reading/Writting Accelerometer Shock Duration/Threshold
- Reading/Writting No Motion/Slow Motion toggle switch
- Reading/Writting Accelerometer No/Slow Motion Setting
- Reading/Writting Gyroscope Interrupt Settings
- Reading/Writting Gyroscope High Rate Settings
- Reading/Writting Gyroscope High Rate Hysteresis Threshold
- Reading/Writting Gyroscope High Rate Duration/Threshold
- Reading/Writting Gyroscope Any Motion Settings
- Reading/Writting Gyroscope Any Motion Threshold
- Reading/Writting Gyroscope Any Motion Awake Duration
- Reading/Writting Gyroscope Any Motion Slope Samples

## BNO055 as an AHRS

BNO055 is not just a `MARG` (Magnetic, Angular Rate, and Gravity) sensor equipped with a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer, but also supports an [AHRS](https://en.wikipedia.org/wiki/Attitude_and_heading_reference_system) (Attitude and heading reference system) functionality.

This module is a System in Package (SiP) that can estimate the orientation of a rigid body (module itself) thanks to [BSX3.0 FusionLib](https://www.bosch-sensortec.com/software-tools/software/sensor-fusion-software/) software. `BSX3.0 FusionLib` helps users to perform the most minor calibration procedure to provide reliable measurements.

The absolute and relative orientation of the module is worked out in `Fusion modes`. The absolute orientation is provided in `COMPASS`, `NDOF_FMC_OFF`, and `NDOF` modes, and relative orientation is provided in `IMU` & `M4G` modes.
The orientation is provided in both [Quaternion](https://en.wikipedia.org/wiki/Quaternion) and `Euler Angles`.
