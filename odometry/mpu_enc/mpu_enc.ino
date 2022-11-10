#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <Wire.h>

// ================================================================
// ===                      Pins Definitions                    ===
// ================================================================

#define PIN_SDA PB_7
#define PIN_SCL PB_6

#define PIN_RX PA_10
#define PIN_TX PA_9

#define PIN_ENC_X_A PB_12
#define PIN_ENC_X_B PB_13

#define PIN_ENC_Y_A PB_12
#define PIN_ENC_Y_B PB_13

// ================================================================
// ===                  Robot Specs Definitions                 ===
// ================================================================

#define WHEEL_RADIUS 0.07  // in meters
#define RESOLUTION 5400.0  // encoders resolution

// ================================================================
// ===                      Misc Definitions                    ===
// ================================================================

#define BAUD_RATE 115200

#define RAD_TO_DEG 180.0 / M_PI
#define DEG_TO_RAD M_PI / 180.0

#define SAMPLE_TIME 5  // in milli seconds




MPU6050 mpu;

// ================================================================
// ===                ROS setup/publishing vars                 ===
// ================================================================

ros::NodeHandle nh;
geometry_msgs::Pose pose_msg;
ros::Publisher pose_pub("pose", &pose_msg);
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// ================================================================
// ===                MPU control/status vars                   ===
// ================================================================
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

uint8_t fifoBuffer[64];  // FIFO storage buffer

// ================================================================
// ===                  orientation/motion vars                 ===
// ================================================================
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 gyro;     // [x, y, z]            gyro sensor measurements
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float theta;
float world_x = 0;
float world_y = 0;
long encoder_x_count = 0;
long encoder_y_count = 0;
long previous_x_count = 0;
long previous_y_count = 0;

// ================================================================
// ===                          Time vars                       ===
// ================================================================

uint32_t previous_time = 0;



void setup() {
  // intialize node
  nh.initNode();
  nh.advertise(pose_pub);
  nh.advertise(imu_pub);

  // initialize serial communication
  Serial.setRx(PIN_RX);
  Serial.setTx(PIN_TX);
  Serial.begin(BAUD_RATE);

  // Set port and BAUD rate for ROS
  (nh.getHardware())->setPort(&Serial);
  (nh.getHardware())->setBaud(BAUD_RATE);

  //
  //  Encoder Setup
  //
  pinMode(PIN_ENC_X_A, INPUT_PULLUP);
  pinMode(PIN_ENC_X_B, INPUT_PULLUP);

  pinMode(PIN_ENC_Y_A, INPUT_PULLUP);
  pinMode(PIN_ENC_Y_B, INPUT_PULLUP);

  attachInterrupt(PIN_ENC_X_A, ISR_encoder_x_a, CHANGE);
  attachInterrupt(PIN_ENC_X_B, ISR_encoder_x_b, CHANGE);

  attachInterrupt(PIN_ENC_Y_A, ISR_encoder_y_a, CHANGE);
  attachInterrupt(PIN_ENC_Y_B, ISR_encoder_y_b, CHANGE);


  //
  //  IMU Setup
  //

  // set SDA and SCL pins
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  // join I2C bus
  Wire.begin();

  analogWriteFrequency(2000);
  analogWriteResolution(16);

  // initialize device
  nh.logdebug("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  nh.logdebug("Testing device connections...");
  nh.logdebug(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  nh.logdebug("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    nh.logdebug("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    nh.loginfo("IMU DMP ready!");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    nh.logerror(String("DMP Initialization failed (code " + String(devStatus) + ")").c_str());
  }
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  updateImuReadings();
  publishImuReadings();

  theta = ypr[0];  // set orientation theta to yaw angle

  u_int32_t current_time = millis();
  int dt_millis = (current_time - previous_time);
  if (dt_millis >= SAMPLE_TIME) {
    updatePosition();
    publishPosition();
    previous_time = current_time;
  }

  nh.spinOnce();
}

void readFifoBuffer() {
  // Clear the buffer so as we can get fresh values
  // The sensor is running a lot faster than our sample period
  mpu.resetFIFO();

  // get current FIFO count
  uint16_t fifoCount = mpu.getFIFOCount();

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}

// ================================================================
// ===                  Encoder counter callbacks               ===
// ================================================================

void ISR_encoder_x_a() {
  encoder_x_count += (digitalRead(PIN_ENC_X_A) == digitalRead(PIN_ENC_X_B)) ? 1 : -1;
}

void ISR_encoder_x_b() {
  encoder_x_count += (digitalRead(PIN_ENC_X_A) == digitalRead(PIN_ENC_X_B)) ? -1 : 1;
}

void ISR_encoder_y_a() {
  encoder_y_count += (digitalRead(PIN_ENC_Y_A) == digitalRead(PIN_ENC_Y_B)) ? 1 : -1;
}

void ISR_encoder_y_b() {
  encoder_y_count += (digitalRead(PIN_ENC_Y_A) == digitalRead(PIN_ENC_Y_B)) ? -1 : 1;
}

// ================================================================
// ===                   Localization functions                 ===
// ================================================================

void updateImuReadings() {
  readFifoBuffer();

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  mpu.dmpGetGyro(&gyro, fifoBuffer);
}

void publishImuReadings() {

  imu_msg.orientation.w = q.w;
  imu_msg.orientation.x = q.x;
  imu_msg.orientation.y = q.y;
  imu_msg.orientation.z = q.z;

  imu_msg.linear_acceleration.x = aaWorld.x;
  imu_msg.linear_acceleration.y = aaWorld.y;
  imu_msg.linear_acceleration.z = aaWorld.z;

  imu_msg.angular_velocity.x = gyro.x;
  imu_msg.angular_velocity.y = gyro.y;
  imu_msg.angular_velocity.z = gyro.z;

  imu_pub.publish(&imu_msg);
}

void updatePosition() {
  // use the same encoder count during the whole function call
  long current_x_count = encoder_x_count;
  long current_y_count = encoder_y_count;

  long delta_x_count = current_x_count - previous_x_count;
  long delta_y_count = current_y_count - previous_y_count;

  float rotations_x = (float)delta_x_count / RESOLUTION;
  float rotations_y = (float)delta_y_count / RESOLUTION;

  float x_local = rotations_x * 2 * M_PI * WHEEL_RADIUS;
  float y_local = rotations_y * 2 * M_PI * WHEEL_RADIUS;

  float delta_x = x_local * cos(theta) + y_local * sin(theta);
  float delta_y = y_local * cos(theta) - x_local * sin(theta);

  world_x += delta_x;
  world_y += delta_y;

  previous_x_count = current_x_count;
  previous_y_count = current_y_count;
}

void publishPosition() {

  pose_msg.orientation.w = q.w;
  pose_msg.orientation.x = q.x;
  pose_msg.orientation.y = q.y;
  pose_msg.orientation.z = q.z;

  pose_msg.position.x = world_x;
  pose_msg.position.y = world_y;
  pose_msg.position.z = 0;

  pose_pub.publish(&pose_msg);
}