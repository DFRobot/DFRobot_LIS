/*!
 * @file DFRobot_LIS2DW12.h
 * @brief Define the basic structure of class DFRobot_LIS2DW12
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2020-12-23
 * @url https://github.com/DFRobot/DFRobot_LIS
 */

#ifndef DFROBOT_LIS2DW12_H
#define DFROBOT_LIS2DW12_H
#include "Arduino.h"

#include <Wire.h>
#include <SPI.h>
//#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

#define LIS2DW12_I2C_ADDR  (0x19)  ///<LIS2DW12 IIC address
#define IIS2DLPC_I2C_ADDR  (0x19)  ///<IIS2DLPC IIC address

#define ERR_OK              0      ///<ok
#define ERR_DATA_BUS       -1      ///<error in data bus
#define ERR_IC_VERSION     -2      ///<chip version mismatch


class DFRobot_LIS2DW12
{
public:
  #define REG_CARD_ID      0x0F     ///<The chip id
  #define REG_CTRL_REG1    0x20     ///<Control register 1
  #define REG_CTRL_REG4    0x23     ///<Control register 2
  #define REG_CTRL_REG2    0x21     ///<Control register 3
  #define REG_CTRL_REG3    0x22     ///<Control register 4
  #define REG_CTRL_REG5    0x24     ///<Control register 5
  #define REG_CTRL_REG6    0x25     ///<Control register 6
  #define REG_CTRL_REG7    0x3F     ///<Control register 7
  #define REG_STATUS_REG   0x27     ///<Status register
  #define REG_OUT_X_L      0x28     ///<The low order of the X-axis acceleration register
  #define REG_OUT_X_H      0x29     ///<The high point of the X-axis acceleration register
  #define REG_OUT_Y_L      0x2A     ///<The low order of the Y-axis acceleration register
  #define REG_OUT_Y_H      0x2B     ///<The high point of the Y-axis acceleration register
  #define REG_OUT_Z_L      0x2C     ///<The low order of the Z-axis acceleration register
  #define REG_OUT_Z_H      0x2D     ///<The high point of the Z-axis acceleration register
  #define REG_WAKE_UP_DUR  0x35     ///<Wakeup and sleep duration configuration register
  #define REG_FREE_FALL    0x36     ///<Free fall event register
  #define REG_STATUS_DUP   0x37     ///<Interrupt event status register
  #define REG_WAKE_UP_SRC  0x38     ///<Wakeup source register
  #define REG_TAP_SRC      0x39     ///<Tap source register
  #define REG_SIXD_SRC     0x3A     ///<6D source register
  #define REG_ALL_INT_SRC  0x3B     ///<Reading this register, all related interrupt function flags routed to the INT pads are reset simultaneously
  
  #define REG_TAP_THS_X    0x30     ///<4D configuration enable and TAP threshold configuration .
  #define REG_TAP_THS_Y    0x31     ///<Threshold for tap recognition @ FS = ±2 g on Y direction
  #define REG_TAP_THS_Z    0x32     ///<Threshold for tap recognition @ FS = ±2 g on Z direction
  #define REG_INT_DUR      0x33     ///<Interrupt duration register
  #define REG_WAKE_UP_THS  0x34     ///<Wakeup threshold register
  #define SPI_READ_BIT     0X80     ///<bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the device is read.*/

public:

/**
 * @fn  ePowerMode_t
 * @brief Power mode, there are two modes for the sensor to measure acceleration
 * @n 1.Continuous measurement. In this mode, the sensor will continuously measure and store the data in data register.
 * @n 2.Single data conversion on demand mode. In this mode, the sensor will make a measurement only when it receives an external request.
 */
typedef enum{
  eHighPerformance_14bit                  = 0x04,/**<High-Performance Mode,14-bit resolution>*/
  eContLowPwr4_14bit                      = 0x03,/**<Continuous measurement,Low-Power Mode 4(14-bit resolution)>*/
  eContLowPwr3_14bit                      = 0x02,/**<Continuous measurement,Low-Power Mode 3(14-bit resolution)>*/
  eContLowPwr2_14bit                      = 0x01,/**<Continuous measurement,Low-Power Mode 2(14-bit resolution)>*/
  eContLowPwr1_12bit                      = 0x00,/**<Continuous measurement,Low-Power Mode 1(12-bit resolution)>*/
  eSingleLowPwr4_14bit                    = 0x0b,/**<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution)>*/
  eSingleLowPwr3_14bit                    = 0x0a,/**<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution)>*/
  eSingleLowPwr2_14bit                    = 0x09,/**<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution)>*/
  eSingleLowPwr1_12bit                    = 0x08,/**<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution)>*/
  eHighPerformanceLowNoise_14bit          = 0x14,/**<High-Performance Mode,Low-noise enabled,14-bit resolution>*/
  eContLowPwrLowNoise4_14bit              = 0x13,/**<Continuous measurement,Low-Power Mode 4(14-bit resolution,Low-noise enabled)>*/
  eContLowPwrLowNoise3_14bit              = 0x12,/**<Continuous measurement,Low-Power Mode 3(14-bit resolution,Low-noise enabled)>*/
  eContLowPwrLowNoise2_14bit              = 0x11,/**<Continuous measurement,Low-Power Mode 2(14-bit resolution,Low-noise enabled)>*/
  eContLowPwrLowNoise1_12bit              = 0x10,/**<Continuous measurement,Low-Power Mode 1(12-bit resolution,Low-noise enabled)>*/
  eSingleLowPwrLowNoise4_14bit            = 0x1b,/**<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution),Low-noise enabled>*/
  eSingleLowPwrLowNoise3_14bit            = 0x1a,/**<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution),Low-noise enabled>*/
  eSingleLowPwrLowNoise2_14bit            = 0x19,/**<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution),Low-noise enabled>*/
  eSingleLowPwrLowNoise1_12bit            = 0x18,/**<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution),Low-noise enabled>*/
}ePowerMode_t;

/**
 * @fn eRange_t
 * @brief Sensor range
 */
typedef enum{
  e2_g     = 2, /**<±2g>*/
  e4_g     = 4, /**<±4g>*/
  e8_g     = 8, /**<±8g>*/
  e16_g    = 16, /**<±16g>*/
}eRange_t;

/**
 * @fn ePath_t
 * @brief Filtering mode
 */
typedef enum {
  eLPF        = 0x00,/**< low-pass filter path selected>*/
  eHPF        = 0x10,/**<high-pass filter path selected>*/
} ePath_t;

/**
 * @fn eBWFilter_t
 * @brief Bandwidth selection
 */
typedef enum {
  eRateDiv_2     = 0,/**<Rate/2 (up to Rate = 800 Hz, 400 Hz when Rate = 1600 Hz)>*/
  eRateDiv_4     = 1,/**<Rate/4 (High Power/Low power)>*/
  eRateDiv_10    = 2,/**<Rate/10 (HP/LP)>*/
  eRateDiv_20    = 3,/**<Rate/20 (HP/LP)>*/
}eBWFilter_t;

/**
 * @fn  eRate_t
 * @brief Data collection rate
 */
typedef enum {
  eRate_0hz            = 0x00,/**<Measurement off>*/
  eRate_1hz6           = 0x01,/**<1.6hz, use only under low-power mode>*/
  eRate_12hz5          = 0x02,/**<12.5hz>*/
  eRate_25hz           = 0x03,
  eRate_50hz           = 0x04,
  eRate_100hz          = 0x05,
  eRate_200hz          = 0x06,
  eRate_400hz          = 0x07,/**<Use only under High-Performance mode>*/
  eRate_800hz          = 0x08,/**<Use only under High-Performance mode>*/
  eRate_1k6hz          = 0x09,/**<Use only under High-Performance mode>*/
  eSetSwTrig           = 0x12,/**<The software triggers a single measurement>*/
} eRate_t;

/**
 * @fn eActDetect_t
 * @brief Motion detection mode
 */
typedef enum {
  eNoDetection        = 0,/**<No detection>*/
  eDetectAct          = 1,/**<Detect movement,the chip automatically goes to 12.5 Hz rate in the low-power mode>*/
  eDetectStatMotion   = 3,/**<Detect Motion, the chip detects acceleration below a fixed threshold but does not change either rate or operating mode>*/
} eActDetect_t;

/**
 * @fn eInt1Event_t
 * @brief Interrupt source 1 trigger event setting
 */
typedef enum{
  eDoubleTap    = 0x08,/**<Double tap event>*/
  eFreeFall     = 0x10,/**<Free fall event>*/
  eWakeUp       = 0x20,/**<Wake-up event>*/
  eSingleTap    = 0x40,/**<Single tap event>*/
  e6D           = 0x80,/**<An event that changes the status of facing up/down/left/right/forward/back>*/
}eInt1Event_t;

/**
 * @fn eInt2Event_t
 * @brief Interrupt source 2 trigger event setting
 */
typedef enum{
  eSleepChange = 0x40,/**<Sleep change status routed to INT2 pad>*/
  eSleepState  = 0x80,/**<Enable routing of SLEEP_STATE on INT2 pad>*/
}eInt2Event_t;

/**
 * @fn eTapMode_t
 * @brief tap detection mode
 */
typedef enum {
  eOnlySingle          = 0,/**<Only detect tap events.>*/
  eBothSingleDouble    = 1,/**<Both single-tap and double-tap events are detected.>*/
} eTapMode_t;

/**
 * @fn e6DTh_t
 * @brief Position detection angle transition threshold
 */
typedef enum {
  eDegrees80  ,/**<80 degrees.>*/
  eDegrees70  ,/**<70 degrees.>*/
  eDegrees60  ,/**<60 degrees.>*/
  eDegrees50  ,/**<50 degrees.>*/
} e6DTh_t;

/**
 * @fn  eTap_t
 * @brief tap or double tap
 */
typedef enum {
  eSTap  = 0 ,/**<single tap>*/
  eDTap      ,/**<double tap>*/
  eNoTap,
} eTap_t;

/**
 * @fn eTapDir_t
 * @brief which direction is tap event detected
 */
typedef enum {
  eDirXUp   = 0,/**<Tap is detected in the positive direction of X>*/
  eDirXDown = 1,/**<Tap is detected in the negative direction of X>*/
  eDirYUp   = 2,/**<Tap is detected in the positive direction of Y>*/
  eDirYDown = 3,/**<Tap is detected in the negative direction of Y>*/
  eDirZUp   = 4,/**<Tap is detected in the positive direction of Z>*/
  eDirZDown = 5,/**<Tap is detected in the negative direction of Z>*/
  eDirNone  = 6,
}eTapDir_t;

/**
 * @fn eWakeUpDir_t
 * @brief which direction is wake up event detected
 */
typedef enum {
  eDirX = 0,/**<Chip woken up by motion in X direction>*/
  eDirY = 1,/**<Chip woken up by motion in Y direction>*/
  eDirZ = 2,/**<Chip woken up by motion in Z direction>*/
  eDirError =4,/**<Direction detection error>*/
}eWakeUpDir_t;

/**
 * @fn eOrient_t
 * @brief orientation
 */
typedef enum {
  eXDown = 0 , /**<X is now down>*/
  eXUp   = 1 , /**<X is now up>*/
  eYDown = 2 , /**<Y is now down>*/
  eYUp   = 3 , /**<Y is now up>*/
  eZDown = 4 , /**<Z is now down>*/
  eZUp   = 5 , /**<Z is now up>*/
  eNONE,
} eOrient_t;

public:
  DFRobot_LIS2DW12();
  
  /**
   * @fn begin
   * @brief Initialize the function
   * @return true(Initialization succeed)/fasle(Iniatialization failed)
   */
  bool begin(void);
 
  /**
   * @fn getID
   * @brief Get chip id
   * @return 8 bit serial number
   */
  uint8_t getID();
 
  /**
   * @fn softReset
   * @brief Software reset to restore the value of all registers to the default value
   */
  void softReset();
  
  /**
   * @fn continRefresh
   * @brief Enable the chip to collect continuously
   * @param enable  true(continuous update)/false( output registers not updated until MSB and LSB read)
   */
  void continRefresh(bool enable);
  
  /**
   * @fn setFilterPath
   * @brief Set the filter processing mode
   * @param path path of filtering
   * @n          eLPF   = 0x00,/< low-pass filter path selected>/
   * @n          eHPF   = 0x10,/<high-pass filter path selected>/
   */
  void setFilterPath(ePath_t path);

  /**
   * @fn setFilterBandwidth
   * @brief Set the  bandwidth of the data
   * @param bw bandwidth
   * @n        eRateDiv_2  ,/<Rate/2 (up to Rate = 800 Hz, 400 Hz when Rate = 1600 Hz)>/
   * @n        eRateDiv_4  ,/<Rate/4 (High Power/Low power)>*
   * @n        eRateDiv_10 ,/<Rate/10 (HP/LP)>/
   * @n        eRateDiv_20 ,/<Rate/20 (HP/LP)>/
   */
  void setFilterBandwidth(eBWFilter_t bw);
  
  /**
   * @fn setPowerMode
   * @brief Set power mode, there are two modes for the sensor to measure acceleration
   * @n       1.Continuous measurement In this mode, the sensor will continuously measure and store the data in data register
   * @n       2.Single data conversion on demand mode In this mode, the sensor will make a measurement unless it receives an external request
   * @param mode  power modes to choose from
   * @n           eHighPerformance_14bit         /<High-Performance Mode,14-bit resolution>/
   * @n           eContLowPwr4_14bit             /<Continuous measurement,Low-Power Mode 4(14-bit resolution)>/
   * @n           eContLowPwr3_14bit             /<Continuous measurement,Low-Power Mode 3(14-bit resolution)>/
   * @n           eContLowPwr2_14bit             /<Continuous measurement,Low-Power Mode 2(14-bit resolution)/
   * @n           eContLowPwr1_12bit             /<Continuous measurement,Low-Power Mode 1(12-bit resolution)>/
   * @n           eSingleLowPwr4_14bit           /<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution)>/
   * @n           eSingleLowPwr3_14bit           /<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution)>/
   * @n           eSingleLowPwr2_14bit           /<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution)>/
   * @n           eSingleLowPwr1_12bit           /<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution)>/
   * @n           eHighPerformanceLowNoise_14bit /<High-Performance Mode,Low-noise enabled,14-bit resolution>/
   * @n           eContLowPwrLowNoise4_14bit     /<Continuous measurement,Low-Power Mode 4(14-bit resolution,Low-noise enabled)>/
   * @n           eContLowPwrLowNoise3_14bit     /<Continuous measurement,Low-Power Mode 3(14-bit resolution,Low-noise enabled)>/
   * @n           eContLowPwrLowNoise2_14bit     /<Continuous measurement,Low-Power Mode 2(14-bit resolution,Low-noise enabled)>/
   * @n           eContLowPwrLowNoise1_12bit     /<Continuous measurement,Low-Power Mode 1(12-bit resolution,Low-noise enabled)>/
   * @n           eSingleLowPwrLowNoise4_14bit   /<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution),Low-noise enabled>/
   * @n           eSingleLowPwrLowNoise3_14bit   /<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution),Low-noise enabled>/
   * @n           eSingleLowPwrLowNoise2_14bit   /<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution),Low-noise enabled>/
   * @n           eSingleLowPwrLowNoise1_12bit   /<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution),Low-noise enabled>/
   */
  void setPowerMode(ePowerMode_t mode);
  
  /**
   * @fn setDataRate
   * @brief Chip data collection rate setting
   * @param rate  Accelerometer frequency,0-1600hz selection
   * @n           eRate_0hz            /<Measurement off>/
   * @n           eRate_1hz6           /<1.6hz, use only under low-power mode>/
   * @n           eRate_12hz5          /<12.5hz>/
   * @n           eRate_25hz        
   * @n           eRate_50hz        
   * @n           eRate_100hz       
   * @n           eRate_200hz       
   * @n           eRate_400hz         /<Use only under High-Performance mode>/
   * @n           eRate_800hz         /<Use only under High-Performance mode>/
   * @n           eRate_1k6hz         /<Use only under High-Performance mode>/
   * @n           eSetSwTrig          /<The software triggers a single measurement>/
   */
  void setDataRate(eRate_t rate);
  
  /**
   * @fn setFreeFallDur
   * @brief Set the free fall time, or the number of free-fall samples. The free-fall events will not occur unless the samples are enough
   * @param dur Number of freefall samples, range: 0~31
   * @n time = dur * (1/rate)(unit:s)
   * @n |                           An example of a linear relationship between an argument and time                             |
   * @n |------------------------------------------------------------------------------------------------------------------------|
   * @n |                |                     |                          |                          |                           |
   * @n |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
   * @n |------------------------------------------------------------------------------------------------------------------------|
   * @n |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
   * @n |------------------------------------------------------------------------------------------------------------------------|
   */
  void setFreeFallDur(uint8_t dur);
  
  /**
   * @fn setInt1Event
   * @brief Select the interrupt event generated on the interrupt 1 pin
   * @param event  Interrupt event
   * @n            eDoubleTap    = 0x08,/<Double tap event>/
   * @n            eFreeFall     = 0x10,/<Freefall event>/
   * @n            eWakeUp       = 0x20,/<Wake up event>/
   * @n            eSingleTap    = 0x40,/<Single tap event>/
   * @n            e6D           = 0x80,/<An event that changes the status of facing up/down/left/right/forward/back>/
   */
  void setInt1Event(eInt1Event_t event);
  
  /**
   * @fn setInt2Event
   * @fn setInt2Event
   * @brief Select the interrupt event generated on the interrupt 2 pin
   * @param event Interrupt event
   * @n           eSleepChange = 0x40,/<Sleep change status routed to INT2 pad>/
   * @n           eSleepState  = 0x80,/<Enable routing of SLEEP_STATE on INT2 pad>/
   */
  void setInt2Event(eInt2Event_t event);
  
  /**
   * @fn setWakeUpDur
   * @brief Set the wake-up duration, when using the detection mode of eDetectAct in setActMode() function, it will be a period of time to collect 
   * @n data at a normal rate after the chip is awakened. Then the chip will continue to hibernate, collecting data at a frequency of 12.5hz.
   * @param dur duration, range: 0~3
   * @n time = dur * (1/rate)(unit:s)
   * @n  |                        An example of a linear relationship between an argument and time                                |
   * @n  |------------------------------------------------------------------------------------------------------------------------|
   * @n  |                |                     |                          |                          |                           |
   * @n  |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
   * @n  |------------------------------------------------------------------------------------------------------------------------|
   * @n  |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
   * @n  |------------------------------------------------------------------------------------------------------------------------|
   */
  void setWakeUpDur(uint8_t dur);

  /**
   * @fn setWakeUpThreshold
   * @brief Set the wake-up threshold, when the acceleration in a certain direction is greater than this value, a wake-up event will be triggered
   * @param th threshold ,unit:mg, the value is within the measurement range
   */
  void setWakeUpThreshold(float th);
  
  /**
   * @fn setActMode
   * @brief Set the mode of motion detection, the first mode will not detect whether the module is moving; the second,once set, will measure data
   * @n at a lower frequency to save consumption, and return to normal after detecting motion; the third can only detect whether the module is 
   * @n in sleep state.
   * @param mode Motion detection mode
   * @n          eNoDetection         /<No detection>/
   * @n          eDetectAct           /<Detect movement,the chip automatically goes to 12.5 Hz rate in the low-power mode>/
   * @n          eDetectStatMotion    /<Detect Motion, the chip detects acceleration below a fixed threshold but does not change either rate or operating mode>/
   */
  void setActMode(eActDetect_t mode);
  
  /**
   * @fn setRange
   * @brief Set the range
   * @param range Measurement range
   * @n           e2_g     = 2, /<±2g>/
   * @n           e4_g     = 4, /<±4g>/
   * @n           e8_g     = 8, /<±8g>/
   * @n           e16_g    = 16, /< ±16g>/
   */
  void setRange(eRange_t range);
  
  /**
   * @fn enableTapDetectionOnZ
   * @brief enable detect tap events in the Z direction
   * @param enable ture(Enable tap detection)\false(Disable tap detection)
   */
  void enableTapDetectionOnZ(bool enable);
  
  /**
   * @fn enableTapDetectionOnY
   * @brief enable detect tap events in the Y direction
   * @param enable ture(Enable tap detection)\false(Disable tap detection)
   */
  void enableTapDetectionOnY(bool enable);

  /**
   * @fn enableTapDetectionOnX
   * @brief enable detect tap events in the X direction
   * @param enable ture(Enable tap detection)\false(Disable tap detection)
   */
  void enableTapDetectionOnX(bool enable);

  /**
   * @fn setTapThresholdOnX
   * @brief Set the tap threshold in the X direction
   * @param th Threshold(mg),Can only be used in the range of 0~2g
   */
  void setTapThresholdOnX(float th);
  
  /**
   * @fn setTapThresholdOnY
   * @brief Set the tap threshold in the Y direction
   * @param th Threshold(mg),Can only be used in the range of 0~2g
   */
  void setTapThresholdOnY(float th);

  /**
   * @fn setTapThresholdOnZ
   * @brief Set the tap threshold in the Z direction
   * @param th Threshold(mg),Can only be used in the range of 0~2g
   */
  void setTapThresholdOnZ(float th);
  
  /**
   * @fn setTapDur
   * @brief Duration of maximum time gap for double-tap recognition. When double-tap 
   * @n recognition is enabled, this register expresses the maximum time between two 
   * @n successive detected taps to determine a double-tap event.
   * @param dur duration, range: 0~15
   * @n time = dur * (1/rate)(unit:s)
   * @n  |                      An example of a linear relationship between an argument and time                                  |
   * @n  |------------------------------------------------------------------------------------------------------------------------|
   * @n  |                |                     |                          |                          |                           |
   * @n  |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
   * @n  |------------------------------------------------------------------------------------------------------------------------|
   * @n  |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
   * @n  |------------------------------------------------------------------------------------------------------------------------|
   */
  void setTapDur(uint8_t dur);
  
  /**
   * @fn setTapMode
   * @brief Set the tap detection mode, detect single tap or both single tap and double tap.
   * @param mode Tap detection mode
   * @n           eOnlySingle   /<Detect only single tap>/
   * @n           eBothSingleDouble /<Detect both single tap and double tap>/
   */
  void setTapMode(eTapMode_t mode);

  /**
   * @fn set6DThreshold
   * @brief Set Thresholds for 4D/6D, when the threshold of rotation exceeds the specified angle, a direction change event will occur.
   * @param degree   eDegrees80   /<80°>/
   * @n              eDegrees70   /<70°>/
   * @n              eDegrees60   /<60°>/
   * @n              eDegrees50   /<50°>/
   */
  void set6DThreshold(e6DTh_t degree);

  /**
   * @fn readAccX
   * @brief Read the acceleration in the x direction
   * @return  Acceleration data from x(mg), the measurement range is ±2g, ±4g, ±8g or ±16g, set by the setRange() function
   */
  int16_t readAccX();
  
  /**
   * @fn readAccY
   * @brief Read the acceleration in the y direction
   * @return  Acceleration data from y(mg), the measurement range is ±2g, ±4g, ±8g or ±16g, set by the setRange() function
   */
  int16_t readAccY();
  
  /**
   * @fn readAccZ
   * @brief Read the acceleration in the z direction
   * @return  Acceleration data from z(mg), the measurement range is ±2g, ±4g, ±8g or ±16g, set by the setRange() function
   */
  int16_t readAccZ();
  
  /**
   * @fn actDetected
   * @brief Detect whether a motion is generated
   * @return true(Motion generated)/false(No motion)
   */
  bool actDetected();
  
  /**
   * @fn freeFallDetected
   * @brief Detect whether a freefall occurs
   * @return true(Freefall detected)/false(No freefall detected)
   */
  bool freeFallDetected();
  
  /**
   * @fn oriChangeDetected
   * @brief Detect whether the direction of the chip changes when the chip is facing up/down/left/right/forward/back (ie 6D)
   * @return true(a change in position is detected)/false(no event detected)
   */
  bool oriChangeDetected();
  
  /**
   * @fn getOrientation
   * @brief Only in 6D (facing up/down/left/right/forward/backward) state can the function get the orientation of the sensor
   * @n     relative to the positive z-axis.
   * @return    eXDown  /<X is now down>/
   * @n         eXUp    /<X is now up>/
   * @n         eYDown  /<Y is now down>/
   * @n         eYUp    /<Y is now up>/
   * @n         eZDown  /<Z is now down>/
   * @n         eZUp    /<Z is now up>/
   */
  eOrient_t getOrientation();
  
  /**
   * @fn tapDetect
   * @brief Tap detection, can detect it is double tap or single tap
   * @return   eSTap       /<Single Tap>/
   * @n        eDTap       /<double Tap>/
   * @n        eNoTap,     /<No tap>         
   */
  eTap_t tapDetect();
  
  /**
   * @fn getTapDirection
   * @brief Tap direction source detection
   * @return   eDirXUp   /<Tap is detected in the positive direction of X>/
   * @n        eDirXDown /<Tap is detected in the negative direction of X>/
   * @n        eDirYUp   /<Tap is detected in the positive direction of Y>/
   * @n        eDirYDown /<Tap is detected in the negative direction of Y>/
   * @n        eDirZUp   /<Tap is detected in the positive direction of Z>/
   * @n        eDirZDown /<Tap is detected in the negative direction of Z>/
   */
  eTapDir_t getTapDirection();
  
  /**
   * @fn getWakeUpDir
   * @brief Wake up motion direction detection
   * @return   eDirX    /<Chip woken up by motion in X direction>/
   * @n        eDirY    /<Chip woken up by motion in Y direction>/
   * @n        eDirZ    /<Chip woken up by motion in Z direction>/
   * @n        eDirError,/<Direction detection error>/
   */
  eWakeUpDir_t getWakeUpDir();
  
  /**
   * @fn demandData
   * @brief In Single data conversion on demand mode
   */
  void demandData();
protected:

  /**
   * @fn readReg
   * @brief read data from sensor chip register
   * @param reg chip register 
   * @param pBuf  buf for store data to read 
   * @param size  number of data to read
   * @return Number of successfully read data
   */
  virtual uint8_t readReg(uint8_t reg,uint8_t * pBuf ,size_t size) = 0;
  
  /**
   * @fn writeReg
   * @brief Write data to sensor register 
   * @param reg register
   * @param pBuf  buf for store data to write 
   * @param size  The number of the data in pBuf
   * @return Number of successfully sent data
   */
  virtual uint8_t  writeReg(uint8_t reg,const void *pBuf,size_t size)= 0; 
  float   _range     = e2_g;
  int32_t _range1    = 0;
private:

  /**
   * @fn setTapQuiet
   * @brief set quiet time after a tap detection: this register represents
   * @n the time after the first detected tap in which there must not be any overthreshold event.
   * @param quiet quiet time
   */
  void setTapQuiet(uint8_t quiet);
  
  /**
   * @fn setTapShock
   * @brief Set the maximum time of an over-threshold signal detection to be recognized as a tap event.
   * @param shock  shock time
   */
  void setTapShock(uint8_t shock);
  
  /**
   * @fn set6dFeedData
   * @brief Set 6d filtered data source
   * @param data 0: rate/2 low pass filtered data sent to 6D interrupt function (default)
   * @n          1: LPF2 output data sent to 6D interrupt function)
   */
  void set6dFeedData(uint8_t data);
  
  /**
   * @fn setFfThreshold
   * @brief Set Free-fall threshold
   * @param th threshold
   */
  void setFfThreshold(uint8_t th);
  
  /**
   * @fn setActSleepDur
   * @brief Set duration to go in sleep mode.
   * @param dur  duration
   */
  void setActSleepDur(uint8_t dur);

  /**
   * @fn lockInterrupt
   * @brief lock interrupt Switches between latched ('1'-logic) and pulsed ('0'-logic) mode for 
   * @n function source signals and interrupts routed to pins (wakeup, single/double-tap).
   * @param enable  true lock interrupt/false pulsed interrupt
   */
  void lockInterrupt(bool enable);
};

class DFRobot_IIS2DLPC_I2C : public DFRobot_LIS2DW12{
public:
  /**
   * @fn DFRobot_IIS2DLPC_I2C
   * @brief Constructor 
   * @param pWire I2c controller
   * @param addr  I2C address(0x19/0x18)
   */
  DFRobot_IIS2DLPC_I2C(TwoWire * pWire = &Wire,uint8_t addr = IIS2DLPC_I2C_ADDR);

  /**
   * @fn begin
   * @brief init function
   * @return true(Initialization succeeded)/fasle(Initialization failed)
   */
  bool begin(void);

protected:

  /**
   * @fn readReg
   * @brief read data from sensor chip register
   * @param reg chip register 
   * @param pBuf  buf for store data to read 
   * @param size  number of data to read
   * @return Number of successfully read data
   */
  uint8_t readReg(uint8_t reg,uint8_t * pBuf ,size_t size);
  
  /**
   * @fn writeReg
   * @brief Write data to sensor register 
   * @param reg register
   * @param pBuf  buf for store data to write 
   * @param size  The number of the data in pBuf
   * @return Number of successfully sent data
   */
  uint8_t  writeReg(uint8_t reg,const void *pBuf,size_t size); 
private:
    uint8_t _deviceAddr;
    TwoWire *_pWire;
};

class DFRobot_IIS2DLPC_SPI : public DFRobot_LIS2DW12{

public:
  /**
   * @fn DFRobot_IIS2DLPC_SPI
   * @brief Constructor 
   * @param cs : Chip selection pinChip selection pin
   * @param spi :SPI controller
   */
  DFRobot_IIS2DLPC_SPI(uint8_t cs = 3,SPIClass *spi=&SPI);
  
  /**
   * @fn begin
   * @brief init function
   * @return true(Initialization succeeded)/fasle(Initialization failed)
   */
  bool begin(void);
protected:

  /**
   * @fn readReg
   * @brief read data from sensor chip register
   * @param reg chip register 
   * @param pBuf  buf for store data to read 
   * @param size  number of data to read
   * @return Number of successfully read data
   */
  uint8_t readReg(uint8_t reg,uint8_t * pBuf ,size_t size);
  
  /**
   * @fn writeReg
   * @brief Write data to sensor register 
   * @param reg register
   * @param pBuf  buf for store data to write 
   * @param size  The number of the data in pBuf
   * @return Number of successfully sent data
   */
  uint8_t  writeReg(uint8_t reg,const void *pBuf,size_t size); 
private:
    SPIClass *_pSpi;
    uint8_t _cs = 0;
};

class DFRobot_LIS2DW12_I2C : public DFRobot_LIS2DW12{
public:
  /**
   * @fn DFRobot_LIS2DW12_I2C
   * @brief Constructor 
   * @param pWire I2c controller
   * @param addr  I2C address(0x19/0x18)
   */
  DFRobot_LIS2DW12_I2C(TwoWire * pWire = &Wire,uint8_t addr = LIS2DW12_I2C_ADDR);
  
  /**
   * @fn begin
   * @brief init function
   * @return true(Initialization succeeded)/fasle(Initialization succeeded)
   */
  bool begin(void);

protected:
  
  /**
   * @fn begin
   * @brief read data from sensor chip register
   * @param reg chip register 
   * @param pBuf  buf for store data to read 
   * @param size  number of data to read
   * @return Number of successfully read data
   */
  uint8_t readReg(uint8_t reg,uint8_t * pBuf ,size_t size);

  /**
   * @fn writeReg
   * @brief Write data to sensor register 
   * @param reg register
   * @param pBuf  buf for store data to write 
   * @param size  The number of the data in pBuf
   * @return Number of successfully sent data
   */
  uint8_t  writeReg(uint8_t reg,const void *pBuf,size_t size); 
private:
    uint8_t _deviceAddr;
    TwoWire *_pWire;
};

class DFRobot_LIS2DW12_SPI : public DFRobot_LIS2DW12{

public:
  /**
   * @fn DFRobot_LIS2DW12_SPI
   * @brief Constructor 
   * @param cs : Chip selection pinChip selection pin
   * @param spi :SPI controller
   */
  DFRobot_LIS2DW12_SPI(uint8_t cs = 3,SPIClass *spi=&SPI);
  
  /**
   * @fn begin
   * @brief init function
   * @return true(Initialization succeeded)/fasle(Initialization succeeded)
   */
  bool begin(void);
protected:

  /**
   * @fn readReg
   * @brief read data from sensor chip register
   * @param reg chip register 
   * @param pBuf  buf for store data to read 
   * @param size  number of data to read
   * @return Number of successfully read data
   */
  uint8_t readReg(uint8_t reg,uint8_t * pBuf ,size_t size);
  
  /**
   * @fn writeReg
   * @brief Write data to sensor register 
   * @param reg register
   * @param pBuf  buf for store data to write 
   * @param size  The number of the data in pBuf
   * @return Number of successfully sent data
   */
  uint8_t  writeReg(uint8_t reg,const void *pBuf,size_t size); 
private:
    SPIClass *_pSpi;
    uint8_t _cs = 0;
};
#endif
