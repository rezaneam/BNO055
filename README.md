# [Bosch BNO055 Driver](https://www.bosch-sensortec.com/products/smart-sensors/bno055/) 
This library is written based on [Adafruit Unified BNO055 Driver](https://github.com/adafruit/Adafruit_BNO055) while redundant dependency to [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor) is removed.
<p align="center">
  <img width="800px" src="https://www.bosch-sensortec.com/media/boschsensortec/products/smart_sensors/16_19/bosch-sensortec_website-relaunch_stage_bno055_res_800x450.jpg">
</p>

## What is new in this driver
Driver is optimized for a better operation with [FreeRTOS](https://www.freertos.org/). Moreover this library includes additional features in the `page 1` as follows.
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
