/*!
 * @file  DFRobot_WT61PC.h
 * @brief  Define the basic structure of DFRobot_WT61PC sensor class 
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  huyujie(yujie.hu@dfrobot.com)
 * @version  V1.0
 * @date  2023-07-10
 * @url  https://github.com/DFRobot/DFRobot_WT61PC
 */
#ifndef __DFRobot_WT61PC_H__
#define __DFRobot_WT61PC_H__

#include <Arduino.h>

// #define ENABLE_DBG   //!< open this macro and you can see the details of the program
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif


class DFRobot_WT61PC
{
public:
#define  FREQUENCY_0_1HZ   0X01
#define  FREQUENCY_0_5HZ   0X02
#define  FREQUENCY_1HZ     0X03
#define  FREQUENCY_2HZ     0X04
#define  FREQUENCY_5HZ     0X05
#define  FREQUENCY_10HZ    0X06
#define  FREQUENCY_20HZ    0X07
#define  FREQUENCY_50HZ    0X08
#define  FREQUENCY_100HZ   0X09
#define  FREQUENCY_125HZ   0X0A
#define  FREQUENCY_200HZ   0X0B

  /**
   * @struct sSensorData_t
   * @brief A structure that stores three kinds of measurement data
   */
  typedef struct 
  {
    float X;
    float Y;
    float Z;
  }sSensorData_t;

  sSensorData_t Acc;     // Store the acceleration data of x, y, and z axes, unit(m/s²)
  sSensorData_t Gyro;    // Store the angular velocity data of x, y, and z axes， unit(°/s)
  sSensorData_t Angle;   // Store the angle data of x, y and z axes, unit (°)

  /**
   * @fn DFRobot_WT61PC
   * @brief Constructor 
   * @param Stream Software serial port interface 
   * @return None
   */
  DFRobot_WT61PC(Stream *s);

  /**
   * @fn available
   * @brief  Check if there is data to read 
   * @return Return true, data avaiable to read; false, no data to read 
   */
  bool available(void);

  /**
   * @fn modifyFrequency
   * @brief Revise the data output frequncy of sensor
   * @param frequency - FREQUENCY_0_1HZ for 0.1Hz, FREQUENCY_0_5HZ for 0.5Hz, FREQUENCY_1HZ for 1Hz, 
   * @n                 FREQUENCY_2HZ for 2Hz, FREQUENCY_5HZ for 5Hz, FREQUENCY_10HZ for 10Hz, 
   * @n                 FREQUENCY_20HZ for 20Hz, FREQUENCY_50HZ for 50Hz, FREQUENCY_100HZ for 100Hz, 
   * @n                 FREQUENCY_125HZ for 125Hz, FREQUENCY_200HZ for 200Hz.
   * @return None
   */
  void modifyFrequency(uint8_t frequency);

private:

#define HEADERACC 0x51     // Acceleration packet header 
#define HEADERGYRO 0x52    // Angular velocity packet header 
#define HEADERANGLE 0x53   // Angle packet header 
#define HEADER55 0x55

#define TIMEOUT 5000       // timeout 

#define WT61PC_XL 2
#define WT61PC_XH 3
#define WT61PC_YL 4
#define WT61PC_YH 5
#define WT61PC_ZL 6
#define WT61PC_ZH 7

  Stream *_s;
  uint8_t receivedAccData[11];     //Store the acceleration original data packet of x, y, and z
  uint8_t receivedGyroData[11];    //Store the angular velocity original data packet of x, y, and z 
  uint8_t receivedAngleData[11];   //Store the angle original data packet of x, y and z 

  uint8_t Cmd[5] = {0xff, 0xaa, 0x03, FREQUENCY_10HZ, 0x00};   //Revise the command of sensor output data frequency 

private:
  /**
   * @fn readN
   * @brief  Read data on serial port 
   * @param  buf Store the read data 
   * @param  len Read the length of byte 
   * @return  Return the number of byte actually read 
   */
  size_t readN(uint8_t *buf, size_t len);

  /**
   * @fn recvData
   * @brief  Check if the read data is valid  
   * @param  buf Store the read data 
   * @param  header HEADERACC for Acc, HEADERGYRO for Gyro, HEADERANGLE for Angle
   * @return  Return true, valid; false, invalid 
   */
  bool recvData(uint8_t *buf, uint8_t header);

  /**
   * @fn getCS
   * @brief  Calculate checksum 
   * @param  buf Data packet to be checked 
   * @return  Return checksum 
   */
  uint8_t getCS (uint8_t *buf);

  /**
   * @fn getAcc
   * @brief  Calculate acceleration of x, y and z, and place the result into Acc struct  
   * @param  buf Original data packet 
   * @return None
   */
  void getAcc(uint8_t *buf);

  /**
   * @fn getGyro
   * @brief  Calculate the angular velocity of x, y and z, and place the result into Gyro struct 
   * @param  buf Original data packet 
   * @return None
   */
  void getGyro(uint8_t *buf);

  /**
   * @fn getAngle
   * @brief  Calculate the angle of x, y and z, place the result into Angle struct 
   * @param  buf Original data packet 
   * @return None
   */
  void getAngle(uint8_t *buf);
};

#endif
