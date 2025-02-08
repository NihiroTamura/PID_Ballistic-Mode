//===============================================
#define CONTROL_PERIOD_MS 1
#define PUB_PERIOD_MS 10
#define LED 13
// Topic names
#define SUB_TOPICNAME "/board1/aout"
#define PUB_TOPICNAME "/board1/ain"
// AD/DA conv channels
#define ANALOG_IN_CH 18
#define ANALOG_OUT_CH 12
const int ain_channels[ANALOG_IN_CH] = {14,15,16,17,18,19,20,21,22,23,24,25,26,27,38,39,40,41};
const int aout_channels[ANALOG_OUT_CH] = {0,1,2,3,4,5,6,7,8,9,28,29};
//===============================================

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include "TeensyThreads.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

std_msgs__msg__UInt16MultiArray msg_pub;
std_msgs__msg__UInt16MultiArray msg_sub;

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

volatile uint16_t aout[12];
volatile uint16_t ain[18];

Threads::Mutex adc_lock;

void error_loop(){
  while(1){
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
  }
}


void thread_callback() {
  while(1) {
    int t0 = millis();

    //====================================
    // write codes from here
    //====================================
    if(1) {
      Threads::Scope scope(adc_lock);

      for(int i = 0; i < ANALOG_IN_CH; i++){
        ain[i] = analogRead(ain_channels[i]);
      }

      //
      //
      //
    
      for(int i = 0; i < ANALOG_OUT_CH; i++){
        analogWrite(aout_channels[i], aout[i]);
      }
    }
    //====================================
    // to here
    //====================================
    int t1 = millis();
    //Serial.println(t1-t0);
    if( t1-t0 <= CONTROL_PERIOD_MS ){
      // spare cpu slices to other threads for the remaining time
      threads.delay(CONTROL_PERIOD_MS - (t1-t0));
    } else {
      // the designated period is violated
      error_loop();
    }
  }
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    for (size_t i = 0; i < ANALOG_IN_CH; i++) {
      msg_pub.data.data[i] = ain[i];
    }
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
  }
}


void subscription_callback(const void * msgin)
{
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    aout[i] = msg->data.data[i];
  }
  
}


void setup() {
  set_microros_transports();
  
  // configure LED pin
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // configure PWM/AD pins
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    pinMode(aout_channels[i], OUTPUT);
  }
  for (size_t i = 0; i < ANALOG_IN_CH; i++) {
    pinMode(ain_channels[i], INPUT);
  }

  // allocate message variables
  msg_pub.data.capacity = ANALOG_IN_CH;
  msg_pub.data.size = 0;
  msg_pub.data.data = (uint16_t*)malloc(msg_pub.data.capacity * sizeof(uint16_t));
  msg_pub.layout.dim.capacity = 1; // 1-dimentional array: vector
  msg_pub.layout.dim.size = 0;
  msg_pub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_pub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  for(size_t i = 0; i < msg_pub.layout.dim.capacity; i++){
    msg_pub.layout.dim.data[i].label.capacity = 20;
    msg_pub.layout.dim.data[i].label.size = 0;
    msg_pub.layout.dim.data[i].label.data = (char*) malloc(msg_pub.layout.dim.data[i].label.capacity * sizeof(char));
  }
  
  msg_sub.data.capacity = ANALOG_OUT_CH;
  msg_sub.data.size = 0;
  msg_sub.data.data = (uint16_t*)malloc(msg_sub.data.capacity * sizeof(uint16_t));
  msg_sub.layout.dim.capacity = 1; // 1-dimentional array: vector
  msg_sub.layout.dim.size = 0;
  msg_sub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_sub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  for(size_t i = 0; i < msg_sub.layout.dim.capacity; i++){
    msg_sub.layout.dim.data[i].label.capacity = 20;
    msg_sub.layout.dim.data[i].label.size = 0;
    msg_sub.layout.dim.data[i].label.data = (char*) malloc(msg_sub.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // initialize message variables
  msg_pub.data.size = ANALOG_IN_CH;
  msg_pub.layout.dim.size = 1;
  msg_pub.layout.dim.data[0].label.size = strlen(msg_pub.layout.dim.data[0].label.data);
  msg_pub.layout.dim.data[0].size = ANALOG_IN_CH;
  msg_pub.layout.dim.data[0].stride = ANALOG_IN_CH;
  for (size_t i = 0; i < ANALOG_IN_CH; i++) {
    ain[i] = 0;
    msg_pub.data.data[i] = ain[i];
  }
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    msg_sub.data.data[i] = 255;
    aout[i] = msg_sub.data.data[i];
    
  }

  delay(2000);

  // create default allocator
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    SUB_TOPICNAME));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    PUB_TOPICNAME));

  // create timer,
  const unsigned int timer_timeout = PUB_PERIOD_MS;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // turn off LED
  digitalWrite(LED, LOW);

  // run the control thread
  threads.addThread(thread_callback);
}

void loop() {

  // spin the ros tasks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
