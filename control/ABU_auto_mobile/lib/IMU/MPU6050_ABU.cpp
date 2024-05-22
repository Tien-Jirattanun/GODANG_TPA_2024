// MPU6050_ABU.cpp
#include "MPU6050_ABU.h"

MPU6050_ABU::MPU6050_ABU(float f0, float fs) :
    accelXFilter(f0, fs, false), accelYFilter(f0, fs, false), accelZFilter(f0, fs, false),
    gyroXFilter(f0, fs, false), gyroYFilter(f0, fs, false), gyroZFilter(f0, fs, false) {
    // Filters initialized with provided cutoff and sampling frequencies
}

void MPU6050_ABU::begin() {
    Wire.begin();
    sensor.initialize();
}

void MPU6050_ABU::update() {
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    fax = accelXFilter.filt(ax);
    fay = accelYFilter.filt(ay);
    faz = accelZFilter.filt(az);
    fgx = gyroXFilter.filt(gx);
    fgy = gyroYFilter.filt(gy);
    fgz = gyroZFilter.filt(gz);
}

int16_t MPU6050_ABU::getRawAx() const { return ax; }
int16_t MPU6050_ABU::getRawAy() const { return ay; }
int16_t MPU6050_ABU::getRawAz() const { return az; }
int16_t MPU6050_ABU::getRawGx() const { return gx; }
int16_t MPU6050_ABU::getRawGy() const { return gy; }
int16_t MPU6050_ABU::getRawGz() const { return gz; }

float MPU6050_ABU::getFilteredAx() const { return fax; }
float MPU6050_ABU::getFilteredAy() const { return fay; }
float MPU6050_ABU::getFilteredAz() const { return faz; }
float MPU6050_ABU::getFilteredGx() const { return fgx; }
float MPU6050_ABU::getFilteredGy() const { return fgy; }
float MPU6050_ABU::getFilteredGz() const { return fgz; }
