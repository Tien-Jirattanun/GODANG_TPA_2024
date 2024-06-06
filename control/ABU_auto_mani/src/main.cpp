#include <Arduino.h>
#include <micro_ros_platformio.h>

// Micro ros include

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>

// Control include

#include "ball_gripper.h"
#include <RPi_Pico_TimerInterrupt.h>
#include <pio_encoder.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <SPI.h>

// microros define

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32 com_msg;
std_msgs__msg__Int32MultiArray sen_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Control define
#define LED_PIN 25
#define limit_switch 04
#define INA 10
#define INB 11
#define stepPin 16
#define dirPin 17
#define servoPin 5

#define IR1 26
#define IR3 6

#define Back1 27
#define Back2 28

#define start 1
#define retry 0

BallGripper bg(limit_switch, INA, INB, stepPin, dirPin, servoPin);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_300MS, TCS34725_GAIN_1X);

int maniState = 0;
int lastState = -1;

int colortemp;

/*------------------------------------------------------*/
// error handler

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
    }                                                                                                                  \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/*------------------------------------------------------*/
// working loop

// ros void call back

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {

    sen_msg.data.data[0] = digitalRead(IR1);
    sen_msg.data.data[1] = digitalRead(IR3);
    sen_msg.data.data[2] = digitalRead(Back1);
    sen_msg.data.data[3] = digitalRead(Back2);
    sen_msg.data.data[4] = colortemp;

    RCSOFTCHECK(rcl_publish(&publisher, &sen_msg, NULL));
  }
}

void subscription_callback(const void* msgin)
{
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;

  maniState = msg->data;

  // if (counter++ % 100 == 0 && (vx != 0 || vy != 0 || wz != 0))
  // {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // }s
}

void setup()
{
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(IR1, INPUT);
  pinMode(Back1, INPUT);
  pinMode(Back2, INPUT);
  pinMode(IR3, INPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_mani_auto_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                         "mani_com_data"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                                      "mani_sensor_data"));

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &com_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  sen_msg.data.capacity = 5;
  sen_msg.data.size = 5;
  sen_msg.data.data = (int32_t*)malloc(sen_msg.data.capacity * sizeof(int32_t));

  sen_msg.data.data[0] = 0.0f;
  sen_msg.data.data[1] = 0.0f;
  sen_msg.data.data[2] = 0.0f;
  sen_msg.data.data[3] = 0.0f;
  sen_msg.data.data[4] = 0.0f;
}

void setup1()
{
  bg.setup();
  pinMode(start, INPUT_PULLUP);
  pinMode(retry, INPUT_PULLUP);
  pinMode(Back1, INPUT);
  pinMode(Back2, INPUT);

  Wire.setSDA(20);
  Wire.setSCL(21);

  Wire.begin();
  tcs.begin();

}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

void loop1()
{
  uint16_t r,g,b,c;
  tcs.getRawData(&r,&g,&b,&c);
  colortemp = tcs.calculateColorTemperature_dn40(r,g,b,c);

  if (maniState != lastState)
  {
    switch (maniState)
    {
      case 1:
        bg.preparing();
        break;
      case 2:
        bg.grab();
        bg.lift();
        bg.release();
        break;
      case 3:
        bg.shoot();
        break;
      case 4:
        bg.stepper_cw();
        break;
      case 5:
        bg.stepper_ccw();
        break;
      default:
        break;
    }
    lastState = maniState;
  }
}
