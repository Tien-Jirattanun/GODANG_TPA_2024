
/*!
 * @file getData.ino
 * @brief Set the frequency of data output by the sensor, read the acceleration, 
 * @n angular velocity, and angle of X, Y, and Z axes.
 * @n Experimental phenomenon: when the sensor starts, it outputs data at the set 
 * @n frequency and the data will be displayed on serial monitor.
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [huyujie](yujie.hu@dfrobot.com)
 * @version V1.0
 * @date 2023-07-12
 * @url https://github.com/DFRobot/DFRobot_WT61PC
 */
#include <DFRobot_WT61PC.h>


#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/10, /*tx =*/11);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

DFRobot_WT61PC sensor(&FPSerial);

void setup()
{
  //Use Serial as debugging serial port 
  Serial.begin(115200);

#if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  FPSerial.begin(9600);
#endif

  /**
   * @brief Revise the data output frequncy of sensor
   * @param frequency - FREQUENCY_0_1HZ for 0.1Hz, FREQUENCY_0_5HZ for 0.5Hz, FREQUENCY_1HZ for 1Hz, 
   * @n                 FREQUENCY_2HZ for 2Hz, FREQUENCY_5HZ for 5Hz, FREQUENCY_10HZ for 10Hz, 
   * @n                 FREQUENCY_20HZ for 20Hz, FREQUENCY_50HZ for 50Hz, FREQUENCY_100HZ for 100Hz, 
   * @n                 FREQUENCY_125HZ for 125Hz, FREQUENCY_200HZ for 200Hz.
   */
  sensor.modifyFrequency(FREQUENCY_10HZ);
}


void loop()
{
  if (sensor.available()) {
    Serial.print("Acc\t"); Serial.print(sensor.Acc.X); Serial.print("\t");
    Serial.print(sensor.Acc.Y); Serial.print("\t"); Serial.println(sensor.Acc.Z); //acceleration information of X,Y,Z
    Serial.print("Gyro\t"); Serial.print(sensor.Gyro.X); Serial.print("\t");
    Serial.print(sensor.Gyro.Y); Serial.print("\t"); Serial.println(sensor.Gyro.Z); //angular velocity information of X,Y,Z
    Serial.print("Angle\t"); Serial.print(sensor.Angle.X); Serial.print("\t");
    Serial.print(sensor.Angle.Y); Serial.print("\t"); Serial.println(sensor.Angle.Z); //angle information of X, Y, Z 
    Serial.println();
  }
}
