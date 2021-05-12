# DFRobot_LIS
LIS Family Triple Axis Accelerometer

CHIP                | Work Well    | Work Wrong  | Remarks
------------------ | :----------: | :----------| -----
H3LIS200DL       |      √       |              |             
LIS331HH      |      √       |              |  
LIS2DW12       |      √       |              |             
IIS2DLPC      |      √       |              |              

![SEN0405正面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0405_TOP.png)
![SEN0405反面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0405_BOTTOM.png)
![SEN0407正面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0407_TOP.png)
![SEN0407反面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0407_BOTTOM.png)
![SEN0408正面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0408_TOP.png)
![SEN0408反面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0408_BOTTOM.png)
![SEN0409正面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0409_TOP.png)
![SEN0409反面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0409_BOTTOM.png)
![SEN0411正面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0411_TOP.png)
![SEN0411反面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0411_BOTTOM.png)
![SEN0412正面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0412_TOP.png)
![SEN0412反面svg效果图](https://github.com/DFRobot/DFRobot_LIS/tree/master/resources/images/SEN0412_BOTTOM.png)

## 产品链接（链接到英文商城）
    SKU：SEN0405
    SKU：SEN0407
    SKU：SEN0408
    SKU：SEN0409
    SKU：SEN0411
    SKU：SEN0412

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Method_H3LIS200DL_LIS331HH](#Method_H3LIS200DL_LIS331HH)
* [Method_LIS2DW12_IIS2DLPC](#Method_LIS2DW12_IIS2DLPC)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

Provide an Arduino library to get Three-axis acceleration by reading data from LIS.

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Method_H3LIS200DL_LIS331HH 
```C++
  DFRobot_LIS();
  /**
   * @brief Initialize the function
   * @return true(Succeed)/false(Failed)
   */
  bool begin(void);
 
  /**
   * @brief Get chip id
   * @return 8 bit serial number
   */
  uint8_t getID();
  
  /**
   * @brief Enable interrupt
   * @param source Interrupt pin selection
              eINT1 = 0,/<int1 >/
              eINT2,/<int2>/
   * @param event Interrupt event selection
                   eXLowerThanTh ,/<The acceleration in the x direction is less than the threshold>/
                   eXHigherThanTh ,/<The acceleration in the x direction is greater than the threshold>/
                   eYLowerThanTh,/<The acceleration in the y direction is less than the threshold>/
                   eYHigherThanTh,/<The acceleration in the y direction is greater than the threshold>/
                   eZLowerThanTh,/<The acceleration in the z direction is less than the threshold>/
                   eZHigherThanTh,/<The acceleration in the z direction is greater than the threshold>/
   */
  void enableInterruptEvent(eInterruptSource_t source, eInterruptEvent_t event);
  
  /**
   * @brief Set measurement range
   * @param range Range(g)
                  eH3lis200dl_100g, //±100g
                  eH3lis200dl_200g, //±200g
                  
                  eLis331hh_6g = 6,//±6g
                  eLis331hh_12g = 12 //±12g
                  eLis331hh_24g = 24 //±24g  
    @return true(Set successfully)/false(Set failed)
   */
  bool setRange(eRange_t range);
  
  /**
   * @brief Set data measurement rate
   * @param rate rate(HZ)
                  ePowerDown_0HZ   //Measurement off
                  eLowPower_halfHZ //0.5 hz
                  eLowPower_1HZ
                  eLowPower_2HZ
                  eLowPower_5HZ
                  eLowPower_10HZ
                  eNormal_50HZ
                  eNormal_100HZ
                  eNormal_400HZ
                  eNormal_1000HZ
   */
  void setAcquireRate(ePowerMode_t rate);
  
  /**
   * @brief Set data filtering mode
   * @param mode Four modes
                 eCutOffMode1 = 0,
                 eCutOffMode2,
                 eCutOffMode3,
                 eCutOffMode4,
                 eShutDown, no filering
     eg: Select eCutOffMode1 in 50HZ, and the filtered frequency is 1HZ
   *|---------------------------High-pass filter cut-off frequency configuration-----------------------------|
   *|--------------------------------------------------------------------------------------------------------|
   *|                |    ft [Hz]      |        ft [Hz]       |       ft [Hz]        |        ft [Hz]        |
   *|   mode         |Data rate = 50 Hz|   Data rate = 100 Hz |  Data rate = 400 Hz  |   Data rate = 1000 Hz |
   *|--------------------------------------------------------------------------------------------------------|
   *|  eCutOffMode1  |     1           |         2            |            8         |             20        |
   *|--------------------------------------------------------------------------------------------------------|
   *|  eCutOffMode2  |    0.5          |         1            |            4         |             10        |
   *|--------------------------------------------------------------------------------------------------------|
   *|  eCutOffMode3  |    0.25         |         0.5          |            2         |             5         |
   *|--------------------------------------------------------------------------------------------------------|
   *|  eCutOffMode4  |    0.125        |         0.25         |            1         |             2.5       |
   *|--------------------------------------------------------------------------------------------------------|
   */
  void setHFilterMode(eHighPassFilter_t mode);

  /**
   * @brief Set the threshold of interrupt source 1 interrupt
   * @param threshold The threshold we set before is within measurement range(unit:g)
   */
  void setInt1Th(uint8_t threshold);

  /**
   * @brief Set interrupt source 2 interrupt generation threshold
   * @param threshold The threshold we set before is within measurement range(unit:g）
   */
  void setInt2Th(uint8_t threshold);

  /**
   * @brief Enable sleep wake function
   * @param enable true(enable)\false(disable)
   * @return false Indicate enable failed/true Indicate enable successful
   */
  bool enableSleep(bool enable);
  
  /**
   * @brief Check whether the interrupt event'event' is generated in interrupt 1
   * @param event Interrupt event
                   eXLowerThanTh ,/<The acceleration in the x direction is less than the threshold>/
                   eXHigherThanTh ,/<The acceleration in the x direction is greater than the threshold>/
                   eYLowerThanTh,/<The acceleration in the y direction is less than the threshold>/
                   eYHigherThanTh,/<The acceleration in the y direction is greater than the threshold>/
                   eZLowerThanTh,/<The acceleration in the z direction is less than the threshold>/
                   eZHigherThanTh,/<The acceleration in the z direction is greater than the threshold>/
   * @return true This event generated
             false This event not generated
   */
  bool getInt1Event(eInterruptEvent_t event);

  /**
   * @brief Check whether the interrupt event'event' is generated in interrupt 2
   * @param event Interrupt event
                   eXLowerThanTh ,/<The acceleration in the x direction is less than the threshold>/
                   eXHigherThanTh ,/<The acceleration in the x direction is greater than the threshold>/
                   eYLowerThanTh,/<The acceleration in the y direction is less than the threshold>/
                   eYHigherThanTh,/<The acceleration in the y direction is greater than the threshold>/
                   eZLowerThanTh,/<The acceleration in the z direction is less than the threshold>/
                   eZHigherThanTh,/<The acceleration in the z direction is greater than the threshold>/
   * @return true This event generated
             false This event not generated
   */
  bool getInt2Event(eInterruptEvent_t event);
  
  /**
   * @brief Get the acceleration in the x direction
   * @return acceleration from x 
   */
  int32_t readAccX();
  
  /**
   * @brief Get the acceleration in the y direction
   * @return acceleration from y
   */
  int32_t readAccY();
  
  /**
   * @brief Get the acceleration in the z direction
   * @return acceleration from z
   */
  int32_t readAccZ();
  
  /**
   * @brief Get the acceleration in the three directions of xyz
   * @param accx Store the variable of acceleration in x direction
   * @param accy Store the variable of acceleration in y direction
   * @param accz Store the variable of acceleration in z direction
   * @return true(Get data successfully/false(Data not ready)
   */
  bool getAcceFromXYZ(int32_t &accx,int32_t &accy,int32_t &accz);

  /**
   * @brief Get whether the sensor is in sleep mode
   * @return true(In sleep mode)/false(In normal mode)
   */
  bool getSleepState();
  
  /**
   * @brief Set the sleep state flag
   * @param into true(Flag the current mode as sleep mode)
                 false(Flag the current mode as normal mode)
   */
  void setSleepFlag(bool into);
```
## Method_LIS2DW12_IIS2DLPC
```C++
  DFRobot_LIS2DW12();
  
  /**
   * @brief Initialize the function
   * @return true(Initialization succeed)/fasle(Initialization failed)
   */
  bool begin(void);
 
  /**
   * @brief Get chip id
   * @return 8 bit serial number
   */
  uint8_t getID();
 
  /**
   * @brief Software reset to restore the value of all registers to the default value
   */
  void softReset();
  
  /**
   * @brief Enable the chip to continuously collect data
   * @param enable  true(continuous update)/false( output registers not updated until MSB and LSB read)
   */
  void continRefresh(bool enable);
  
  /**
   * @brief Set the filter processing mode
   * @param path path of filtering
                 eLPF   = 0x00,/< low-pass filter path selected>/
                 eHPF   = 0x10,/<high-pass filter path selected>/
   */
  void setFilterPath(ePath_t path);

  /**
   * @brief Set the  bandwidth of the data
   * @param bw bandwidth
                 eRateDiv_2  ,/<Rate/2 (up to Rate = 800 Hz, 400 Hz when Rate = 1600 Hz)>/
                 eRateDiv_4  ,/<Rate/4 (High Power/Low power)>*
                 eRateDiv_10 ,/<Rate/10 (HP/LP)>/
                 eRateDiv_20 ,/<Rate/20 (HP/LP)>/
   */
  void setFilterBandwidth(eBWFilter_t bw);
  
  /**
   * @brief Set power mode, there are two modes for the sensor to measure acceleration
   * @n       1.Continuous measurement In this mode, the sensor will continuously measure and store data in its register
   * @n       2.Single data conversion on demand mode In this mode, the sensor will not make a measurement unless it receives an external request
   * @param mode  power modes to choose from
                  eHighPerformance_14bit         /<High-Performance Mode,14-bit resolution>/
                  eContLowPwr4_14bit             /<Continuous measurement,Low-Power Mode 4(14-bit resolution)>/
                  eContLowPwr3_14bit             /<Continuous measurement,Low-Power Mode 3(14-bit resolution)>/
                  eContLowPwr2_14bit             /<Continuous measurement,Low-Power Mode 2(14-bit resolution)/
                  eContLowPwr1_12bit             /<Continuous measurement,Low-Power Mode 1(12-bit resolution)>/
                  eSingleLowPwr4_14bit           /<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution)>/
                  eSingleLowPwr3_14bit           /<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution)>/
                  eSingleLowPwr2_14bit           /<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution)>/
                  eSingleLowPwr1_12bit           /<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution)>/
                  eHighPerformanceLowNoise_14bit /<High-Performance Mode,Low-noise enabled,14-bit resolution>/
                  eContLowPwrLowNoise4_14bit     /<Continuous measurement,Low-Power Mode 4(14-bit resolution,Low-noise enabled)>/
                  eContLowPwrLowNoise3_14bit     /<Continuous measurement,Low-Power Mode 3(14-bit resolution,Low-noise enabled)>/
                  eContLowPwrLowNoise2_14bit     /<Continuous measurement,Low-Power Mode 2(14-bit resolution,Low-noise enabled)>/
                  eContLowPwrLowNoise1_12bit     /<Continuous measurement,Low-Power Mode 1(12-bit resolution,Low-noise enabled)>/
                  eSingleLowPwrLowNoise4_14bit   /<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution),Low-noise enabled>/
                  eSingleLowPwrLowNoise3_14bit   /<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution),Low-noise enabled>/
                  eSingleLowPwrLowNoise2_14bit   /<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution),Low-noise enabled>/
                  eSingleLowPwrLowNoise1_12bit   /<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution),Low-noise enabled>/
   */
  void setPowerMode(ePowerMode_t mode);
  
  /**
   * @brief Chip data collection rate setting
   * @param rate  Accelerometer frequency, 0-1600hz selection
                  eRate_0hz            /<Measurement off>/
                  eRate_1hz6           /<1.6hz, use only under low-power mode>/
                  eRate_12hz5          /<12.5hz>/
                  eRate_25hz        
                  eRate_50hz        
                  eRate_100hz       
                  eRate_200hz       
                  eRate_400hz         /<Use only under High-Performance mode>/
                  eRate_800hz         /<Use only under High-Performance mode>/
                  eRate_1k6hz         /<Use only under High-Performance mode>/
                  eSetSwTrig          /<The software triggers a single measurement>/
   */
  void setDataRate(eRate_t rate);
  
  /**
   * @brief Set the free fall time, or the number of free-fall samples. In a measurement, it will not be determined as a free fall event unless the samples are enough.
   * @param dur Freefall samples, range：0~31
   * @n time = dur * (1/rate)(unit:s)
     |                            An example of a linear relationship between an argument and time                            |
     |------------------------------------------------------------------------------------------------------------------------|
     |                |                     |                          |                          |                           |
     |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
     |------------------------------------------------------------------------------------------------------------------------|
     |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
     |------------------------------------------------------------------------------------------------------------------------|
   */
  void setFreeFallDur(uint8_t dur);
  
  /**
   * @brief Select the interrupt event generated on the int1 pin
   * @param event  Interrupt event, when it occurs, a level jump will be generated on the int1 pin
                   eDoubleTap    = 0x08,/<Double tap event>/
                   eFreeFall     = 0x10,/<Free-fall event>/
                   eWakeUp       = 0x20,/<Wake-up event>/
                   eSingleTap    = 0x40,/<Single tap event>/
                   e6D           = 0x80,/<An event that changes the status of facing up/down/left/right/forward/back>/
   */
  void setInt1Event(eInt1Event_t event);
  
  /**
   * @brief Select the interrupt event generated on the int2 pin
   * @param event Interrupt event, when it occurs, a level jump will be generated on the int2 pin
                  eSleepChange = 0x40,/<Sleep change status routed to INT2 pad>/
                  eSleepState  = 0x80,/<Enable routing of SLEEP_STATE on INT2 pad>/
   */
  void setInt2Event(eInt2Event_t event);
  
  /**
   * @brief Set wake-up duration, when using the detection mode of eDetectAct in setActMode() function, it will collect data at a normal rate
   * @n     after the chip is awakened. Then after a period of time, the chip will continue to hibernate, collecting data at a frequency of 12.5hz.
   * @param dur duration,range: 0~3
     @n time = dur * (1/rate)(unit:s)
     |                           An example of a linear relationship between an argument and time                             |
     |------------------------------------------------------------------------------------------------------------------------|
     |                |                     |                          |                          |                           |
     |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
     |------------------------------------------------------------------------------------------------------------------------|
     |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
     |------------------------------------------------------------------------------------------------------------------------|
   */
  void setWakeUpDur(uint8_t dur);

  /**
   * @brief Set the wake-up threshold, when the acceleration in a certain direction is greater than this value, a wake-up event will be triggered
   * @param th threshold ,unit:mg, the value is within the measurement range
   */
  void setWakeUpThreshold(float th);
  
  /**
   * @brief Set the mode of motion detection, the first mode will not detect whether the module is moving; the second, once set, will measure data at a lower 
   * @n frequency to save consumption, and return to normal after detecting motion; the third can only detect whether the module is in sleep state.
   * @param mode Motion detection mode
                eNoDetection         /<No detection>/
                eDetectAct           /<Detect movement,the chip automatically goes to 12.5 Hz rate in the low-power mode>/
                eDetectStatMotion    /<Detect Motion, the chip detects acceleration below a fixed threshold but does not change either rate or operating mode>/
   */
  void setActMode(eActDetect_t mode);
  
  /**
   * @brief Set the range
   * @param range 量程
                   e2_g     = 2, /<±2g>/
                   e4_g     = 4, /<±4g>/
                   e8_g     = 8, /<±8g>/
                   e16_g    = 16, /< ±16g>/
   */
  void setRange(eRange_t range);
  
  /**
   * @brief Enable detect tap events in the Z direction
   * @param enable ture(Enable tap detection)\false(Disable tap detection)
   */
  void enableTapDetectionOnZ(bool enable);
  
  /**
   * @brief Enable detect tap events in the Y direction
   * @param enable ture(Enable tap detection)\false(Disable tap detection)
   */
  void enableTapDetectionOnY(bool enable);

  /**
   * @brief Enable detect tap events in the X direction
   * @param enable ture(Enable tap detection)\false(Disable tap detection)
   */
  void enableTapDetectionOnX(bool enable);

  /**
   * @brief Set the tap threshold in the X direction
   * @param th Threshold(mg),Can only be used in the range of 0~2g
   */
  void setTapThresholdOnX(float th);
  
  /**
   * @brief Set the tap threshold in the Y direction
   * @param th Threshold(mg),Can only be used in the range of 0~2g
   */
  void setTapThresholdOnY(float th);

  /**
   * @brief Set the tap threshold in the Z direction
   * @param th Threshold(mg),Can only be used in the range of 0~2g
   */
  void setTapThresholdOnZ(float th);
  
  /**
   * @brief Duration of maximum time gap for double-tap recognition. When double-tap 
   * @n recognition is enabled, this register expresses the maximum time between two 
   * @n successive detected taps to determine a double-tap event.
   * @param dur duration, range:0~15
   * @n time = dur * (1/rate)(unit:s)
     |                        An example of a linear relationship between an argument and time                                |
     |------------------------------------------------------------------------------------------------------------------------|
     |                |                     |                          |                          |                           |
     |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
     |------------------------------------------------------------------------------------------------------------------------|
     |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
     |------------------------------------------------------------------------------------------------------------------------|
   */
  void setTapDur(uint8_t dur);
  
  /**
   * @brief Set the tap detection mode, detect single tap or both single tap and double tap 
   * @param mode Tap detection mode
                     eOnlySingle   /<Detect single tap>/
                     eBothSingleDouble /<Detect both single tap and double tap>/
   */
  void setTapMode(eTapMode_t mode);

  /**
   * @brief Set Thresholds for 4D/6D, when the threshold of rotation exceeds the specified angle, a direction change event will occur.
   * @param degree   eDegrees80   /<80°>/
                     eDegrees70   /<70°>/
                     eDegrees60   /<60°>/
                     eDegrees50   /<50°>/
   */
  void set6DThreshold(e6DTh_t degree);

  /**
   * @brief Read the acceleration in the x direction
   * @return  Acceleration data from x(mg), the measurement range is ±2g, ±4g, ±8g or ±16g, set by the setRange() funciton.
   */
  int16_t readAccX();
  
  /**
   * @brief Read the acceleration in the y direction
   * @return  Acceleration data from y(mg), the measurement range is ±2g, ±4g, ±8g or ±16g, set by the setRange() funciton.
   */
  int16_t readAccY();
  
  /**
   * @brief Read the acceleration in the z direction
   * @return  Acceleration data from z(mg), the measurement range is ±2g, ±4g, ±8g or ±16g, set by the setRange() funciton.
   */
  int16_t readAccZ();
  
  /**
   * @brief Detect motion
   * @return true(Motion generated)/false(No motion)
   */
  bool actDetected();
  
  /**
   * @brief Detect free fall 
   * @return true(Free-fall detected)/false(No free-fall)
   */
  bool freeFallDetected();
  
  /**
   * @brief Detect whether the direction of the chip changes when the chip is facing up/down/left/right/forward/back (ie 6D)
   * @return true(a change in position detected)/false(no event detected)
   */
  bool oriChangeDetected();
  
  /**
   * @brief Only in 6D (facing up/down/left/right/forward/backward) state can the function get the orientation of the sensor relative to the positive z-axis.
   * @return    eXDown  /<X is now down>/
                eXUp    /<X is now up>/
                eYDown  /<Y is now down>/
                eYUp    /<Y is now up>/
                eZDown  /<Z is now down>/
                eZUp    /<Z is now up>/
   */
  eOrient_t getOrientation();
  
  /**
   * @brief Tap detection, can detect it is double tap or single tap
   * @return   eSTap       /<Single Tap>/
               eDTap       /<double Tap>/
               eNoTap,     //No tap 
                */
  eTap_t tapDetect();
  
  /**
   * @brief Tap direction source detection
   * @return   eDirXUp   /<Tap is detected in the positive direction of X>/
               eDirXDown /<Tap is detected in the negative direction of X>/
               eDirYUp   /<Tap is detected in the positive direction of Y>/
               eDirYDown /<Tap is detected in the negative direction of Y>/
               eDirZUp   /<Tap is detected in the positive direction of Z>/
               eDirZDown /<Tap is detected in the negative direction of Z>/
   */
  eTapDir_t getTapDirection();
  
  /**
   * @brief Wake-up motion direction detection
   * @return   eDirX    /<The chip is woken up by the motion in X direction>/
               eDirY    /<The chip is woken up by the motion in Y direction>/
               eDirZ    /<The chip is woken up by the motion in Z direction>/
               eDirError,/<Error detected>/
   */
  eWakeUpDir_t getWakeUpDir();
  
  /**
   * @brief In Single data conversion on demand mode, request a measurement.
   */
  void demandData();
```
## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
FireBeetle-ESP8266        |      √       |              |             | 
FireBeetle-ESP32        |      √       |              |             | 
Arduino MEGA2560        |      √       |              |             | 
Arduino Leonardo|      √       |              |             | 
Micro:bit        |      √       |              |             | 
FireBeetle-M0        |      √       |              |             | 
Raspberry Pi      |      √       |              |             | 

## History

- data 2021-2-1
- version V1.0


## Credits

Written by(li.feng@dfrobot.com), 2020. (Welcome to our [website](https://www.dfrobot.com/))
