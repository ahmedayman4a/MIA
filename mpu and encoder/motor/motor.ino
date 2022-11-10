#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "time_helper.h"
#include "encoder.h"
#include "motor_pid.h"

#define encoderA_pin PA_15
#define encoderB_pin PB_3

#define motor_pwm_pin PA_6
#define motor_dir_pin PA_7
#define max_rpm 500
#define set_point 150

ros::NodeHandle nh;

std_msgs::Int64 encoder_msg;
std_msgs::String output_msg;
ros::Publisher encoder("encoder", &encoder_msg);
ros::Publisher output("Output", &output_msg);

long encoderCount = 0;

// void rpm_recieved(const std_msgs::Int32 &rpm_msg) {
//   int rpm = rpm_msg.data;
//   set_motor_speed(rpm);
// }

//ros::Subscriber<std_msgs::Int32> sub("controlled_rpm", &rpm_recieved);
TimeHelper t = TimeHelper();
void setup() {
  // initialize serial communication
  Serial.setRx(PB_11);
  Serial.setTx(PB_10);
  Serial.begin(115200);

  // intialize node
  (nh.getHardware())->setPort(&Serial);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.advertise(encoder);
  nh.advertise(output);

  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_dir_pin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA_pin), IRQ_encoder_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_pin), IRQ_encoder_B, CHANGE);
}

void loop() {
  encoder_msg.data = encoderCount;
  encoder.publish(&encoder_msg);
  delay(5);

  double dt = t.get_dt();

  Encoder encoderManager = Encoder(540.0);

  double rpm = encoderManager.count_to_rpm(encoderCount, dt, nh);
  String output_str;
  output_str = "dt : ";
  output_str += String(dt, 5);
  //if (dt > 0) {
  MotorPID motor_pid = MotorPID(set_point, 50, 0, 0);
  double controlled_rpm = motor_pid.get_controlled_rpm(rpm, dt);
  set_motor_speed((int)controlled_rpm);
  double error = set_point - rpm;

  output_str += ";;;;Encoder count : ";
  output_str += String(encoderCount);
  output_str += ";;;;Controlled : ";
  output_str += String(controlled_rpm);
  output_str += ";;;;Actual rpm : ";
  output_str += String(rpm, 5);
  output_str += ";;;;Set point : ";
  output_str += String(set_point);
  output_str += ";;;;Error : ";
  output_str += String(error);
  //}
  output_msg.data = output_str.c_str();
  output.publish(&output_msg);


  nh.spinOnce();
}

void IRQ_encoder_A() {
  encoderCount += (digitalRead(encoderA_pin) == digitalRead(encoderB_pin)) ? 1 : -1;
}

void IRQ_encoder_B() {
  encoderCount += (digitalRead(encoderA_pin) == digitalRead(encoderB_pin)) ? -1 : 1;
}

void set_motor_speed(int rpm) {
  bool opposite_dir = rpm < 0;
  rpm = abs(rpm);
  int dutyCycle = map(rpm, 0, max_rpm, 0, 65535);
  analogWrite(motor_pwm_pin, dutyCycle);
  analogWrite(motor_dir_pin, opposite_dir);
}

void set_motor_speed(int dutyCycle) {
  bool opposite_dir = dutyCycle < 0;
  dutyCycle = abs(dutyCycle);
  //int dutyCycle = map(rpm, 0, max_rpm, 0, 65535);
  dutyCycle = constrain(dutyCycle,0,65535);
  analogWrite(motor_pwm_pin, dutyCycle);
  analogWrite(motor_dir_pin, opposite_dir);
}

float low_pass(float v_new, float v_filtered_old, float dt, int cutoff = 25) {
  float alpha = dt / (dt + 1 / (2 * PI * cutoff));
  float v_filtered_new = v_new * alpha + (1 - alpha) * v_filtered_old;
  return v_filtered_new;
}