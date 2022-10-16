#include <ros.h>
#include <std_msgs/Int64.msg>

#define encoderA_pin PB_12
#define encoderB_pin PB_13

ros::NodeHandle nh;

std_msgs::Int64 encoder_msg;
ros::Publisher encoder("encoder", &encoder_msg);

long long encoderCount = 0;

void setup() {
  // intialize node
  nh.initNode();
  nh.advertise(encoder);

  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderA_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA_pin), IRQ_encoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_pin), IRQ_encoder_B, CHANGE);
}

void loop() {
  encoder_msg.data = encoderCount;
  encoder.publish(&encoder_msg);
  nh.spinOnce();
}

void ArmsInstances::IRQ_encoder_A() {
  encoderCount += (digitalRead(encoderA_pin) == digitalRead(encoderB_pin)) ? 1 : -1;
}

void ArmsInstances::IRQ_encoder_B() {
  encoderCount += (digitalRead(encoderA_pin) == digitalRead(encoder_B_pin)) ? -1 : 1;
}