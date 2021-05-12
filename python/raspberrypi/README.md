# DFRobot_LIS
The H3LIS200DL is a low-power high performance 3-axis linear accelerometer 
belonging to the “nano” family, with digital I2C/SPI 
serial interface standard output. <br>
The device features ultra-low-power operational 
modes that allow advanced power saving and 
smart sleep-to-wakeup functions.<br>
The H3LIS200DL has dynamically user selectable full scales of ±100g/±200g and is 
capable of measuring accelerations with output 
data rates from 0.5 Hz to 1 kHz.<br>
The H3LIS200DL is available in a small thin 
plastic land grid array package (LGA) and is 
guaranteed to operate over an extended 
temperature range from -40 °C to +85 °C.<br>

The LIS331HH is an ultra low-power high 
performance high full-scale three axes linear 
accelerometer belonging to the “nano” family, with 
digital I2C/SPI serial interface standard output. 
The device features ultra low-power operational 
modes that allow advanced power saving and 
smart sleep to wake-up functions. 
The LIS331HH has dynamically user selectable 
full scales of ±6g/±12g/±24g and it is capable of 
measuring accelerations with output data rates 
from 0.5 Hz to 1 kHz. The self-test capability 
allows the user to check the funct

The IIS2DLPC is an ultra-low-power high-performance 
three-axis linear accelerometer with digital I²C/SPI 
output interface which leverages on the robust and 
mature manufacturing processes already used for the 
production of micromachined accelerometers.<br>
The IIS2DLPC has user-selectable full scales of 
±2g/±4g/±8g/±16g and is capable of measuring 
accelerations with output data rates from 1.6 Hz to 
1600 Hz.<br>

The LIS2DW12 is an ultra-low-power high-performance three-axis linear
accelerometer belonging to the “femto” family which leverages on the robust and
mature manufacturing processes already used for the production of micromachined
accelerometers.
The LIS2DW12 has user-selectable full scales of ±2g/±4g/±8g/±16g and is capable of
measuring accelerations with output data rates from 1.6 Hz to 1600 Hz.

CHIP                | Work Well    | Work Wrong  | Remarks
------------------ | :----------: | :----------| -----
H3LIS200DL       |      √       |              |             
LIS331HH      |      √       |              |   
LIS2DW12       |      √       |              |             
IIS2DLPC      |      √       |              |              

## DFRobot_LIS Library for RaspberryPi
---------------------------------------------------------

Provide an RaspberryPi library to get Three-axis acceleration by reading data from H3LIS200DL and LIS331HH.

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Method_H3LIS200DL_LIS331HH](#Method_H3LIS200DL_LIS331HH)
* [Method_LIS2DW12_IIS2DLPC](#Method_LIS2DW12_IIS2DLPC)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

Provide an RaspberryPi library to get Three-axis acceleration by reading data from H3LIS200DL and LIS331HH.

## Installation

Download the DFRobot_LIS file to the Raspberry Pi file directory, then run the following command line to use this sensor:
```
cd DFRobot_LIS/python/raspberry/examples/LIS331HH/get_acceleration
python get_acceleration.py
```


## Method_H3LIS200DL_LIS331HH
```python
  '''
    @brief Initialize the function
    @return return True(Succeed)/False(Failed)
  '''
  def begin(self):
  
  '''
    @brief get chip id
    @return 8 bit serial number
  '''
  def get_id(self):

  '''
    @brief Set the measurement range
    @param range Range(g)
                 H3LIS200DL_100G = 100 #±100G
                 H3LIS200DL_200G = 200 #±200G
                 LIS331HH_6G     #±6G
                 LIS331HH_12G    #±12G
                 LIS331HH_24G    #±24G
  '''
  def set_range(self,range_r):

  '''
    @brief Set data measurement rate
    @param rate rate(HZ)
                 POWERDOWN_0HZ   
                 LOWPOWER_HALFHZ 
                 LOWPOWER_1HZ    
                 LOWPOWER_2HZ 
                 LOWPOWER_5HZ
                 LOWPOWER_10HZ 
                 NORMAL_50HZ
                 NORMAL_100HZ 
                 NORMAL_400HZ 
                 NORMAL_1000HZ 
  '''
  def set_acquire_rate(self, rate):

  '''
    @brief Set the threshold of interrupt source 1 interrupt
    @param threshold Threshold(g)
  '''
  def set_int1_th(self,threshold):

  '''
    @brief Set interrupt source 2 interrupt generation threshold
    @param threshold Threshold(g)
  '''
  def set_int2_th(self,threshold):
  
  '''
    @brief Enable interrupt
    @source Interrupt pin selection
             INT_1 = 0,/<int pad 1 >/
             INT_2,/<int pad 2>/
    @param event Interrupt event selection
             X_LOWERTHAN_TH     = 0x1<The acceleration in the x direction is less than the threshold>
             X_HIGHERTHAN_TH  = 0x2<The acceleration in the x direction is greater than the threshold>
             Y_LOWERTHAN_TH     = 0x4<The acceleration in the y direction is less than the threshold>
             Y_HIGHERTHAN_TH  = 0x8<The acceleration in the y direction is greater than the threshold>
             Z_LOWERTHAN_TH     = 0x10<The acceleration in the z direction is less than the threshold
             Z_HIGHERTHAN_TH  = 0x20<The acceleration in the z direction is greater than the threshold>
             EVENT_ERROR      = 0 <No event>
  '''
  def enable_int_event(self,source,event):

  '''
    @brief Check whether the interrupt event'event' is generated in interrupt 1
    @param event Interrupt event
             X_LOWERTHAN_TH     = 0x1<The acceleration in the x direction is less than the threshold>
             X_HIGHERTHAN_TH  = 0x2<The acceleration in the x direction is greater than the threshold>
             Y_LOWERTHAN_TH     = 0x4<The acceleration in the y direction is less than the threshold>
             Y_HIGHERTHAN_TH  = 0x8<The acceleration in the y direction is greater than the threshold>
             Z_LOWERTHAN_TH     = 0x10<The acceleration in the z direction is less than the threshold
             Z_HIGHERTHAN_TH  = 0x20<The acceleration in the z direction is greater than the threshold>
             EVENT_ERROR      = 0 <No event>
    @return True ：The event is generated.
            False：The event is not generated.
  '''
  def get_int1_event(self,event):
         
  '''
    @brief Check whether the interrupt event'event' is generated in interrupt 2
    @param event Interrupt event
             X_LOWERTHAN_TH     = 0x1<The acceleration in the x direction is less than the threshold>
             X_HIGHERTHAN_TH  = 0x2<The acceleration in the x direction is greater than the threshold>
             Y_LOWERTHAN_TH     = 0x4<The acceleration in the y direction is less than the threshold>
             Y_HIGHERTHAN_TH  = 0x8<The acceleration in the y direction is greater than the threshold>
             Z_LOWERTHAN_TH     = 0x10<The acceleration in the z direction is less than the threshold
             Z_HIGHERTHAN_TH  = 0x20<The acceleration in the z direction is greater than the threshold>
             EVENT_ERROR      = 0 <No event>
    @return True ：The event is generated.
            False：The event is not generated.
  '''
  def get_int2_event(self,source):

  '''
    @brief Enable sleep wake function
    @param enable True(enable)/False(disable)
  '''
  def enable_sleep(self, enable):
  
  '''
    @brief Set data filtering mode
    @param mode Four modes
                CUTOFF_MODE1 = 0
                CUTOFF_MODE2 = 1
                CUTOFF_MODE3 = 2
                CUTOFF_MODE4 = 3
     eg：Select eCutOffMode1 in 50HZ, and the filtered frequency is 1HZ
                            High-pass filter cut-off frequency configuration
    |--------------------------------------------------------------------------------------------------------|
    |                |    ft [Hz]      |        ft [Hz]       |       ft [Hz]        |        ft [Hz]        |
    |   mode         |Data rate = 50 Hz|   Data rate = 100 Hz |  Data rate = 400 Hz  |   Data rate = 1000 Hz |
    |--------------------------------------------------------------------------------------------------------|
    |  CUTOFF_MODE1  |     1           |         2            |            8         |             20        |
    |--------------------------------------------------------------------------------------------------------|
    |  CUTOFF_MODE2  |    0.5          |         1            |            4         |             10        |
    |--------------------------------------------------------------------------------------------------------|
    |  CUTOFF_MODE3  |    0.25         |         0.5          |            2         |             5         |
    |--------------------------------------------------------------------------------------------------------|
    |  CUTOFF_MODE4  |    0.125        |         0.25         |            1         |             2.5       |
    |--------------------------------------------------------------------------------------------------------|
  '''
  def set_filter_mode(self,mode):

  '''
    @brief Get the acceleration in the three directions of xyz
    @return Three-axis acceleration 
  '''
  def read_acce_xyz(self):

```

## Method_LIS2DW12_IIS2DLPC
```python
  '''
     @brief Initialize the function
     @return True(Iniatialization succeed)/Fasle(Iniatialization failed)
  '''
  def begin(self):
  
  '''
     @brief Get chip id
     @return 8 bit serial number
  '''
  def get_id(self):

  '''
     @brief Software reset to restore the value of all registers to the default value
  '''
  def soft_reset(self):
    
  '''
     @brief Set the measurement range
     @param range Range(g)
                 RANGE_2G     #±2g
                 RANGE_4G     #±4g
                 RANGE_8G     #±8g
                 RANGE_16G    #±16g
  '''
  def set_range(self,range_r):
    
  '''
     @brief Choose whether to continuously let the chip collect data
     @param enable  true(continuous update)/false( output registers not updated until MSB and LSB read)
  '''
  def contin_refresh(self,enable):
  
  '''
    @brief Set the filter processing mode
    @param path path of filtering
                LPF          #Low pass filter
                HPF          #High pass filter
  '''
  def set_filter_path(self,path):

  '''
    @brief Set the  bandwidth of the data
    @param bw bandwidth
                RATE_DIV_2   #RATE/2 (up to RATE = 800 Hz, 400 Hz when RATE = 1600 Hz)
                RATE_DIV_4   #RATE/4 (High Power/Low power)
                RATE_DIV_10  #RATE/10 (HP/LP)
                RATE_DIV_20  #RATE/20 (HP/LP)
  '''
  def set_filter_bandwidth(self,bw):
    
  '''
     @brief Set power mode
     @param mode 16 power modes to choose from
               HIGH_PERFORMANCE_14BIT          #High-Performance Mode
               CONT_LOWPWR4_14BIT              #Continuous measurement,Low-Power Mode 4(14-bit resolution)
               CONT_LOWPWR3_14BIT              #Continuous measurement,Low-Power Mode 3(14-bit resolution)
               CONT_LOWPWR2_14BIT              #Continuous measurement,Low-Power Mode 2(14-bit resolution)
               CONT_LOWPWR1_12BIT              #Continuous measurement,Low-Power Mode 1(12-bit resolution)
               SING_LELOWPWR4_14BIT            #Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution)
               SING_LELOWPWR3_14BIT            #Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution
               SING_LELOWPWR2_14BIT            #Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution)
               SING_LELOWPWR1_12BIT            #Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution)
               HIGHP_ERFORMANCELOW_NOISE_14BIT #High-Performance Mode,Low-noise enabled
               CONT_LOWPWRLOWNOISE4_14BIT      #Continuous measurement,Low-Power Mode 4(14-bit resolution,Low-noise enabled)
               CONT_LOWPWRLOWNOISE3_14BIT      #Continuous measurement,Low-Power Mode 3(14-bit resolution,Low-noise enabled)
               CONT_LOWPWRLOWNOISE2_14BIT      #Continuous measurement,Low-Power Mode 2(14-bit resolution,Low-noise enabled)
               CONT_LOWPWRLOWNOISE1_12BIT      #Continuous measurement,Low-Power Mode 1(14-bit resolution,Low-noise enabled)
               SINGLE_LOWPWRLOWNOISE4_14BIT    #Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution),Low-noise enabled
               SINGLE_LOWPWRLOWNOISE3_14BIT    #Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution),Low-noise enabled
               SINGLE_LOWPWRLOWNOISE2_14BIT    #Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution),Low-noise enabled
               SINGLE_LOWPWRLOWNOISE1_12BIT    #Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution),Low-noise enabled
  '''
  def set_power_mode(self,mode):
    
  '''
     @brief Set data measurement rate
     @param rate rate
                 RATE_OFF          #Measurement off
                 RATE_1HZ6         #1.6hz, use only under low-power mode
                 RATE_12HZ5        #12.5hz
                 RATE_25HZ         
                 RATE_50HZ         
                 RATE_100HZ        
                 RATE_200HZ        
                 RATE_400HZ        #Use only under High-Performance mode
                 RATE_800HZ        #Use only under High-Performance mode
                 RATE_1600HZ       #Use only under High-Performance mode
                 SETSWTRIG         #The software triggers a single measurement
  '''
  def set_data_rate(self, rate):
    
  '''
     @brief Set the free fall time, or the numbers of free-fall samples. In a measurement, it will not be determined as a free fall event unless the samples are enough. 
     @param dur duration, range: 0~31
     @n time = dur * (1/rate)(unit:s)
     |                   An example of a linear relationship between an argument and time                                     |
     |------------------------------------------------------------------------------------------------------------------------|
     |                |                     |                          |                          |                           |
     |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
     |------------------------------------------------------------------------------------------------------------------------|
     |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
     |------------------------------------------------------------------------------------------------------------------------|
  '''
  def set_free_fall_dur(self,dur):

  
  '''
    @brief Set the interrupt source of the int1 pin
    @param event  Several interrupt events, after setting, when an event is generated, a level transition will be generated on the int1 pin
              DOUBLE_TAP    #Double tap event
              FREEFALL      #Freefall event
              WAKEUP        #Wake-up event
              SINGLE_TAP    #Single tap event
              IA6D          #An event that changes the status of facing up/down/left/right/forward/back
    
  '''
  def set_int1_event(self,event):
    
  '''
     @brief Set the wake-up duration, when using the detection mode of eDetectAct in setActMode() function, it will be a period of time to collect
     @n data at a normal rate after the chip is awakened. Then the chip will continue to hibernate, collecting data at a frequency of 12.5hz.
     @param dur  duration, range: 0~3
     @n time = dur * (1/rate)(unit:s)
     |                                  An example of a linear relationship between an argument and time                      |
     |------------------------------------------------------------------------------------------------------------------------|
     |                |                     |                          |                          |                           |
     |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
     |------------------------------------------------------------------------------------------------------------------------|
     |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
     |------------------------------------------------------------------------------------------------------------------------|
  '''
  def set_wakeup_dur(self,dur):
  
  '''
    @brief Set the mode of motion detection, the first mode will not detect whether the module is moving; the second, once set, will measure data
    @n at a lower frequency to save consumption, and return to normal after detecting motion; the third can only detect whether the module is in
    @n sleep state. 
    @param mode Motion detection mode
                NO_DETECTION         #No detection
                DETECT_ACT           #Detect movement,the chip automatically goes to 12.5 Hz rate in the low-power mode
                DETECT_STATMOTION    #Detect Motion, the chip detects acceleration below a fixed threshold but does not change either
                                      rate or operating mode
  '''
  def set_act_mode(self,mode):

  '''
    @brief Set the wake-up threshold, when the acceleration in a certain direction is greater than this value, a wake-up event will be triggered
    @param th threshold ,unit:mg, the value is within the measurement range
  '''
  def set_wakeup_threshold(self,th):
    
  '''
    @brief Set to detect tap events in the Z direction
    @param enable Ture(Enable tap detection\False(Disable tap detection)
  '''
  def enable_tap_detection_on_z(self, enable):
  
  '''
    @brief Set to detect tap events in the Y direction
    @param enable Ture(Enable tap detection\False(Disable tap detection)
  '''
  def enable_tap_detection_on_y(self, enable):

    
  '''
    @brief Set to detect tap events in the X direction
    @param enable Ture(Enable tap detection)\False(Disable tap detection)
  '''
  def enable_tap_detection_on_x(self, enable):

  '''
    @brief Set the tap threshold in the X direction
    @param th Threshold(g),Can only be used in the range of ±2g
  '''
  def set_tap_threshold_on_x(self,th):
  
  '''
    @brief Set the tap threshold in the Y direction
    @param th Threshold(g),Can only be used in the range of ±2g
  '''
  def set_tap_threshold_on_y(self,th):
    
  '''
    @brief Set the tap threshold in the Z direction
    @param th Threshold(g),Can only be used in the range of ±2g
  '''
  def set_tap_threshold_on_z(self,th):
    
  '''
     @brief Duration of maximum time gap for double-tap recognition. When double-tap 
     @n recognition is enabled, this register expresses the maximum time between two 
     @n successive detected taps to determine a double-tap event.
     @param dur  duration, range: 0~15
     @n time = dur * (1/rate)(unit:s)
     |                      An example of a linear relationship between an argument and time                                  |
     |------------------------------------------------------------------------------------------------------------------------|
     |                |                     |                          |                          |                           |
     |  Data rate     |       25 Hz         |         100 Hz           |          400 Hz          |         = 800 Hz          |
     |------------------------------------------------------------------------------------------------------------------------|
     |   time         |dur*(1s/25)= dur*40ms|  dur*(1s/100)= dur*10ms  |  dur*(1s/400)= dur*2.5ms |  dur*(1s/800)= dur*1.25ms |
     |------------------------------------------------------------------------------------------------------------------------|
  '''
  def set_tap_dur(self,dur):
  
  '''
    @brief Set the tap detection mode, detect single tap or detect both single tap and double tap
    @param mode  Tap detection mode
                     ONLY_SINGLE        #Detect single tap
                     BOTH_SINGLE_DOUBLE #Detect both single tap and double tap
  '''
  def set_tap_mode(self,mode):
  
  '''
    @brief Set Thresholds for 4D/6D，When the threshold of rotation exceeds the specified angle, a direction change event will occur.
    @param degree   DEGREES_80   #80°
                    DEGREES_70   #70°
                    DEGREES_60   #60°
                    DEGREES_50   #50°
  '''
  def set_6d_threshold(self,degree):
    
  '''
    @brief Select the interrupt event generated on the int2 pin
    @param event  Several interrupt events, after setting, when an event is generated, a level transition will be generated on the int2 pin
                  SLEEP_CHANGE  #Enable routing of SLEEP_STATE on INT2 pad
                  SLEEP_STATE   #0x80 Sleep change status routed to INT2 pad
  '''
  def set_int2_event(self,event):
  
  '''
    @brief Read the acceleration in the x direction
    @return Acceleration data from x(mg), the measurement range is ±2g,±4g,±8g or ±16g, set by the setRange() function
  '''
  def read_acc_x(self):

  '''
    @brief Read the acceleration in the y direction
    @return  Acceleration data from y(mg), the measurement range is ±2g,±4g,±8g or ±16g, set by the setRange() function
  '''
  def read_acc_y(self):

  '''
    @brief Read the acceleration in the z direction
    @return Acceleration data from z(mg), the measurement range is ±2g,±4g,±8g or ±16g, set by the setRange() function
  '''
  def read_acc_z(self):
  
  '''
    @brief Detect whether a motion is generated
    @return True(Motion generated)/False(No motion)
  '''
  def act_detected(self):
      
  '''
    @brief Freefall detection
    @return True(Freefall detected)/False(No freefall detected)
  '''
  def free_fall_detected(self):
    
  '''
    @brief Detect whether the direction of the chip changes when the chip is facing up/down/left/right/forward/back (ie 6D)
    @return True(a change in position is detected)/False(no event detected)
  '''
  def ori_change_detected(self):
      
  '''
     @brief Only in 6D (facing up/down/left/right/forward/backward) state can the function get the orientation of the sensor relative to the     
     @n positive z-axis.
     @return      X_DOWN   #X is now down
               X_UP     #X is now up
               Y_DOWN   #Y is now down
               Y_UP     #Y is now up
               Z_DOWN   #Z is now down
               Z_UP     #Z is now up
  '''
  def get_oriention(self):
     
  '''
    @brief Tap detection, can detect it is double tap or single tap
    @return   S_TAP       #single tap
              D_TAP       #double tap
              NO_TAP,     #No tap generated
  '''
  def tap_detect(self):

  '''
    @brief Source detection of tap direction
    @return     DIR_X_UP   #Tap is detected in the positive direction of X
                DIR_X_DOWN #Tap is detected in the negative direction of X
                DIR_Y_UP   #在Tap is detected in the positive direction of Y
                DIR_Y_DOWN #Tap is detected in the negative direction of Y
                DIR_Z_UP   #Tap is detected in the positive direction of Z
                DIR_Z_DOWN #Tap is detected in the negative direction of Z
  '''
  def get_tap_direction(self):
   
  '''
    @brief Wake-up motion direction detection.
    @return    DIR_X  #The chip is woken up by the motion in X direction
               DIR_Y  #The chip is woken up by the motion in Y direction
               DIR_Z  #The chip is woken up by the motion in Z direction
               eDirError,
  '''
  def get_wake_up_dir(self):

  '''
    @brief In Single data conversion on demand mode, request a measurement 
  '''
  def demand_data(self):
```
## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Raspberry Pi              |      √         |            |             | 



## History

- data 2021-1-26
- version V1.0


## Credits

Written by(li.feng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))
