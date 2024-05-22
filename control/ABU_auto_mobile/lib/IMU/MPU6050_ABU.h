// MPU6050_ABU.h
#ifndef MPU6050_ABU_H
#define MPU6050_ABU_H

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include "LowPass.h"
#include <math.h>

class MPU6050_ABU {
private:
    MPU6050 sensor;
    LowPass<2> accelXFilter, accelYFilter, accelZFilter; // Filters for each accelerometer axis
    LowPass<2> gyroXFilter, gyroYFilter, gyroZFilter;   // Filters for each gyroscope axis

    int16_t ax, ay, az; // Raw accelerometer data
    int16_t gx, gy, gz; // Raw gyroscope data

    float fax, fay, faz; // Filtered accelerometer data
    float fgx, fgy, fgz; // Filtered gyroscope data

public:
    // Constructor that accepts cutoff frequency and sampling frequency
    MPU6050_ABU(float f0, float fs);

    void begin();
    void update();

    int16_t getRawAx() const;
    int16_t getRawAy() const;
    int16_t getRawAz() const;
    int16_t getRawGx() const;
    int16_t getRawGy() const;
    int16_t getRawGz() const;

    float getFilteredAx() const;
    float getFilteredAy() const;
    float getFilteredAz() const;
    float getFilteredGx() const;
    float getFilteredGy() const;
    float getFilteredGz() const;
};

#endif
