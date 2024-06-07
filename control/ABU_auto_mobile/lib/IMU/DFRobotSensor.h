#ifndef DFROBOT_SENSOR_H
#define DFROBOT_SENSOR_H

#include <DFRobot_WT61PC.h>

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO)) // Using a soft serial port
#include <SoftwareSerial.h>
#endif

struct SensorData
{
    float px, py, pz;
    float ax_filtered, ay_filtered, az_filtered;
    float gx_filtered, gy_filtered, gz_filtered;
    float angleX, angleY, angleZ;
};

class DFRobotSensor
{
public:
    DFRobotSensor();

    void begin();
    void update();
    void Reset();
    SensorData getSensorData() const;

private:
    DFRobot_WT61PC sensor;
    unsigned long previousTime;
    float vx, vy, vz;
    float px, py, pz;
    float delta_time;
    float roll, pitch;

    // Low-pass filter variables
    const float alpha = 0.1; // Smoothing factor for low-pass filter
    float ax_filtered, ay_filtered, az_filtered;
    float gx_filtered, gy_filtered, gz_filtered;

    // Translation vector
    float tx, ty, tz;

    // Angle variables
    float set_zero;
    float angle_z;

    bool isStationary(float ax, float ay, float az);
    bool isXStationary(float ax);
    bool isYStationary(float ay);
};

#endif // DFROBOT_SENSOR_H
