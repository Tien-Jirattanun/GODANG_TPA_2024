# DFRobot_WT61PC
* [English Version](./README.md)

- 该模块集成了高精度陀螺仪、加速度计、高性能微处理器和先进的动力学求解和卡尔曼滤波算法，旨在快速求解当前模块姿态的实时运动。
- 传感器采用串行通信，数据输出频率可修改。
- 传感器测量的数据包括加速度、角速度和x、y、z角。

![产品实物图](./resources/images/WT61PC.png)


## 产品链接 (https://www.dfrobot.com.cn/goods-3015.html)

  SKU: SEN0386


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

* 通过UART接口获取传感器数据(加速度：3 维，角速度：3 维，角度：3 维)


## 库安装

要使用这个库, 首先下载库文件, 将其粘贴到\Arduino\libraries目录中, 然后打开示例文件夹并在文件夹中运行演示。


## 方法

```C++

  /**
   * @fn DFRobot_WT61PC
   * @brief 构造函数
   * @param Stream 软件串口接口
   * @return 无
   */
  DFRobot_WT61PC(Stream *s);

  /**
   * @fn available
   * @brief 检查是否有可读取的数据
   * @return 如果有可读取的数据则返回true；否则返回false
   */
  bool available(void);

  /**
   * @fn modifyFrequency
   * @brief 修改传感器的数据输出频率
   * @param frequency - FREQUENCY_0_1HZ 表示0.1Hz，FREQUENCY_0_5HZ 表示0.5Hz，FREQUENCY_1HZ 表示1Hz，
   * @n                 FREQUENCY_2HZ 表示2Hz，FREQUENCY_5HZ 表示5Hz，FREQUENCY_10HZ 表示10Hz，
   * @n                 FREQUENCY_20HZ 表示20Hz，FREQUENCY_50HZ 表示50Hz，FREQUENCY_100HZ 表示100Hz，
   * @n                 FREQUENCY_125HZ 表示125Hz，FREQUENCY_200HZ 表示200Hz。
   * @return 无
   */
  void modifyFrequency(uint8_t frequency);

```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
Arduino MEGA2560   |      √       |              |             | 
Arduino Leonardo   |      √       |              |             | 


## 历史

- 2023/07/12 - 1.0.0 版本


## 创作者

Written by huyujie(yujie.hu@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

