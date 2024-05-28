/*!
 * @file  DFRobot_WT61PC.cpp
 * @brief  Realize the basic structure of DFRobot_WT61PC sensor class 
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence  The MIT License (MIT)
 * @author  huyujie(yujie.hu@dfrobot.com)
 * @version  V1.0
 * @date  2023-07-10
 * @url  https://github.com/DFRobot/DFRobot_WT61PC
 */
#include "DFRobot_WT61PC.h"

DFRobot_WT61PC::DFRobot_WT61PC(Stream *s)
{
  _s = s;
}

size_t DFRobot_WT61PC::readN(uint8_t *buf, size_t len)
{
  size_t offset = 0, left = len;
  long curr = millis();
  while (left) {
    if (_s -> available()) {
      buf[offset++] = _s->read();
      left--;
    }
    if (millis() - curr > TIMEOUT) {
      break;
    }
  }
  return offset;
}

bool DFRobot_WT61PC::recvData(uint8_t *buf, uint8_t header)
{
  long curr = millis();
  bool ret = false;
  uint8_t ch;
  while (!ret) {
    if (millis() - curr > TIMEOUT) {
      break;
    }

    if (readN(&ch, 1) != 1) {
      continue;
    }

    if (ch == HEADER55) {
      buf[0] = ch;
      if (readN(&ch, 1) == 1) {
        if (ch == header) {
          buf[1] = ch;
          if (readN(&buf[2], 9) == 9) {
            if (getCS(buf) == buf[10]) {
              ret = true;
            }
          }
        }
      }
    }
  }
  return ret;
}

uint8_t DFRobot_WT61PC::getCS(uint8_t *buf)
{
  uint8_t cs = 0;
  for (int i = 0; i < 10; i++) {
    cs = cs + buf[i];
  }
  return cs;
}

bool DFRobot_WT61PC::available(void)
{
  bool ret = false;
  if (recvData(receivedAccData , HEADERACC) && recvData(receivedGyroData , HEADERGYRO) && recvData(receivedAngleData , HEADERANGLE)) {
    ret = true;
    getAcc(receivedAccData);
    getGyro(receivedGyroData);
    getAngle(receivedAngleData);
  }
  return ret;
}

void DFRobot_WT61PC::getAcc(uint8_t *buf)
{
  Acc.X = ((buf[WT61PC_XH] << 8) | buf[WT61PC_XL]) / 32768.000 * 16.000 * 9.8;
  Acc.Y = ((buf[WT61PC_YH] << 8) | buf[WT61PC_YL]) / 32768.000 * 16.000 * 9.8;
  Acc.Z = ((buf[WT61PC_ZH] << 8) | buf[WT61PC_ZL]) / 32768.000 * 16.000 * 9.8;
}

void DFRobot_WT61PC::getGyro(uint8_t *buf)
{
  Gyro.X = ((buf[WT61PC_XH] << 8) | buf[WT61PC_XL]) / 32768.000 * 2000.000;
  Gyro.Y = ((buf[WT61PC_YH] << 8) | buf[WT61PC_YL]) / 32768.000 * 2000.000;
  Gyro.Z = ((buf[WT61PC_ZH] << 8) | buf[WT61PC_ZL]) / 32768.000 * 2000.000;
}

void DFRobot_WT61PC::getAngle(uint8_t *buf)
{
  Angle.X = ((buf[WT61PC_XH] << 8) | buf[WT61PC_XL]) / 32768.000 * 180.000;
  Angle.Y = ((buf[WT61PC_YH] << 8) | buf[WT61PC_YL]) / 32768.000 * 180.000;
  Angle.Z = ((buf[WT61PC_ZH] << 8) | buf[WT61PC_ZL]) / 32768.000 * 180.000;
}


void DFRobot_WT61PC::modifyFrequency(uint8_t frequency)
{
  Cmd[3] = frequency;
  _s->write(Cmd, 5);

}
