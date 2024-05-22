#include "Arduino.h"
#include "Motor.h"
// #include "LowPass.h"

Motor::Motor(int encoderPin, int INA_PIN, int INB_PIN, double kp, double ki, double kd)
  : encoder(encoderPin), _INA_PIN(INA_PIN), _INB_PIN(INB_PIN), _kp(kp), _ki(ki), _kd(kd) {
  pinMode(_INA_PIN, OUTPUT);
  pinMode(_INB_PIN, OUTPUT);
}

void Motor::setSpeed(int speed) {
  if (speed > 0) {
    analogWrite(_INA_PIN, speed);
    digitalWrite(_INB_PIN, 0);
  } else if (speed < 0) {
    digitalWrite(_INA_PIN, 0);
    analogWrite(_INB_PIN, -speed);
  } else {
    digitalWrite(_INA_PIN, 0);
    digitalWrite(_INB_PIN, 0);
  }
}

void Motor::stop() {
  digitalWrite(_INA_PIN, LOW);
  digitalWrite(_INB_PIN, LOW);
}

float Motor::computeRADS(float setPoint_RADS, float deltaTime) {
  if (setPoint_RADS != 0) {
    int pos = encoder.getCount();
    float velocity = (pos - _prevCount) / deltaTime;
    _v = (velocity / 980) * (2 * M_PI);                                      
    _vFilt = 0.96906992 * _vFilt + 0.01546504 * _v + 0.01546504 * _vPrev;
    // _vFilt = lp.filt(_v);

    // _ffw = 621.79 * setPoint_RADS + 2508.9;  //edit here
    // setSpeed(_ffw + _u);

    _error = setPoint_RADS - _vFilt;
    _integral += _error * deltaTime;
    float P = _kp * _error;
    float I = _ki * _integral;
    float D = _kd * (_error - _lastError) / deltaTime;

    if (_error == 0) {
      I = 0;
    }
    I = std::min(std::max(I, -255.0f), 255.0f);
    _u = P + I + D;

    _u = std::max(std::min(_u, 255.0f), -255.0f);

    setSpeed(_u);
    // Serial.print(_error);
    // Serial.print(" ");
    // Serial.print(P);
    // Serial.print(" ");
    // Serial.print(_u);
    // Serial.print(" ");
    // Serial.print(setPoint_RADS);
    // Serial.print(" ");
    // Serial.print(_vFilt);
    // Serial.print(" ");

    _vPrev = _v;
    _prevCount = pos;
    _lastError = _error;

  } else if (setPoint_RADS == 0) {
    stop();
  }

  return _vFilt;
}

void Motor::FindSpeedFromPWM(float pwm, float deltaTime) {
  int pos = encoder.getCount();
  float velocity = (pos - _prevCount) / deltaTime;
  _v = velocity / 980 * 2 * M_PI;
  _vFilt = 0.96906992 * _vFilt + 0.01546504 * _v + 0.01546504 * _vPrev;
  // _vFilt = lp.filt(_v);
  setSpeed(pwm);

  Serial.print(_v);
  Serial.print(" ");
  Serial.print(_vFilt);
  Serial.print(" ");

  _vPrev = _v;
  _prevCount = pos;
}