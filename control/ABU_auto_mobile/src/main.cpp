#include <Arduino.h>
#include <micro_ros_platformio.h>

// Micro ros include

#include <stdio.h>
#include <iostream>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

// Control include

#include "Motor.h"
#include "Kinematics.h"
#include <RPi_Pico_TimerInterrupt.h>
#include <pio_encoder.h>

// microros define

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray vel_msg;
std_msgs__msg__Float32MultiArray pos_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float vel[4] = { 0, 0, 0, 0 };

// control define

#define dt_us 1000	// 1000 Hz
float deltaT = dt_us / 1.0e6;
RPI_PICO_Timer Timer(0);
bool TimerStatus = false;

Motor FL(2, 10, 11, 200, 20, 0);
Motor FR(4, 12, 13, 200, 20, 0);
Motor BL(6, 14, 15, 200, 20, 0);
Motor BR(8, 20, 21, 200, 20, 0);

float wheelDiameter = 0.127;
float lx = 0.388 / 2;
float ly = 0.375 / 2;
Kinematics kinematics(wheelDiameter, lx, ly);

Kinematics::Position currentPosition{ 0.0, 0.0, 0.0 };

float vx, vy, wz;

float radps_fl, radps_fr, radps_bl, radps_br;

struct TransformStep
{
	float vx;
	float vy;
	float wz;
	unsigned long duration;
};

// variable
long start_time, T;
int currentStep = 0;
int stepsCount = 0;
bool newStepsAvailable = false;
TransformStep steps[10];

#define LED_PIN 25

// --------------------------------

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
		delay(2000);
	}
}

// -------------------------

// ros void call back

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		pos_msg.data.data[0] = currentPosition.x;
		pos_msg.data.data[1] = currentPosition.y;
		pos_msg.data.data[2] = currentPosition.theta;

		RCSOFTCHECK(rcl_publish(&publisher, &pos_msg, NULL));
	}
}

void subscription_callback(const void* msgin)
{
	const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
	vel[0] = msg->data.data[0];
	vel[1] = msg->data.data[1];
	vel[2] = msg->data.data[2];
}

// timer interrupt call back

bool TimerHandler(struct repeating_timer* t)
{
	(void)t;

	vx = vel[0];
	vy = vel[1];
	wz = vel[2];

	Kinematics::RadPS wheelSpeeds = kinematics.Inverse_Kinematics(vx, vy, wz);
	radps_fl = FL.computeRADS(wheelSpeeds.radps_fl, deltaT);
	radps_fr = FR.computeRADS(wheelSpeeds.radps_fr, deltaT);
	radps_bl = BL.computeRADS(wheelSpeeds.radps_bl, deltaT);
	radps_br = BR.computeRADS(wheelSpeeds.radps_br, deltaT);
	currentPosition =
			kinematics.Forward_Kinematics_Position(radps_fl, radps_fr, radps_bl, radps_br, currentPosition, deltaT);

	return true;
}

void setup()
{
	Serial.begin(921600);
	set_microros_serial_transports(Serial);
	delay(100);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	delay(2000);

	allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_mobile_auto_node", "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(&subscriber, &node,
																				 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "vel_data"));

	// create publisher
	RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
																			"pos_data"));

	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

	vel_msg.data.capacity = 3;
	vel_msg.data.size = 3;
	vel_msg.data.data = (float_t*)malloc(vel_msg.data.capacity * sizeof(float_t));

	vel_msg.layout.dim.capacity = 3;
	vel_msg.layout.dim.size = 3;
	vel_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(vel_msg.layout.dim.capacity *
																																				sizeof(std_msgs__msg__MultiArrayDimension));

	for (size_t i = 0; i < vel_msg.layout.dim.capacity; i++)
	{
		vel_msg.layout.dim.data[i].label.capacity = 3;
		vel_msg.layout.dim.data[i].label.size = 3;
		vel_msg.layout.dim.data[i].label.data = (char*)malloc(vel_msg.layout.dim.data[i].label.capacity * sizeof(char));
	}

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &vel_msg, &subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer))

	pos_msg.data.capacity = 3;
	pos_msg.data.size = 3;
	pos_msg.data.data = (float_t*)malloc(pos_msg.data.capacity * sizeof(float_t));

	vel_msg.data.data[0] = 0.0f;
	vel_msg.data.data[1] = 0.0f;
	vel_msg.data.data[2] = 0.0f;
	vel_msg.data.data[3] = 0.0f;
}

void loop()
{
	RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

void setup1()
{
	analogWriteFreq(1000);
	analogWriteRange(255);

	FR.encoder.begin();
	FL.encoder.begin();
	BR.encoder.begin();
	BL.encoder.begin();

	// Timer Interrupt Setup
	TimerStatus = Timer.attachInterruptInterval(dt_us, TimerHandler);
}

void loop1()
{
}