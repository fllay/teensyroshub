#include <Arduino.h>
#include "TeensyThreads.h"
#include "Kinematics.h"
#include "waveshareMotor.h"

#include "libs/ros.h"
#include "libs/std_msgs/String.h"
#include "libs/std_msgs/Empty.h"

#include "libs/sensor_msgs/Imu.h"
#include "libs/sensor_msgs/MagneticField.h"

#include "libs/std_msgs/Int32.h"
#include "libs/geometry_msgs/Twist.h"
#include "lino_velocities.h"


#define LINO_BASE SKID_STEER      // 2WD and Tracked robot w/ 2 motors
#define MAX_RPM 100               // motor's maximum RPM
#define WHEEL_DIAMETER 0.10       // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.302  // distance between left and right wheels 0.800
#define FR_WHEELS_DISTANCE 0.175  // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

#define HWSERIAL1 Serial5
#define HWSERIAL2 Serial8
#define HWSERIAL3 Serial2
#define HWSERIAL4 Serial7

#define COMMAND_RATE 30

int led_pin = 13;
int led_state = 0;

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);


waveshareMotor md;

ros::NodeHandle nh;

int currentFrontLeftWheelRPM;
int currentFrontRightWheelRPM;
int currentRearLeftWheelRPM;
int currentRearRightWheelRPM;

std_msgs::Int32 rpmLeft;
std_msgs::Int32 rpmRight;
lino_msgs::Velocities raw_vel_msg;

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

Threads::Mutex mutex;

ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher rpmLeft_pub("rpmLeft", &rpmLeft);
ros::Publisher rpmRight_pub("rpmRight", &rpmRight);

void commandCallback(const geometry_msgs::Twist& cmd_msg) {
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

void messageCb(const std_msgs::Empty& toggle_msg) {
  digitalWrite(13, HIGH - digitalRead(13));  // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb);



/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected) {
  static bool wait_flag = false;

  if (isConnected) {
    if (wait_flag == false) {
      delay(10);

      wait_flag = true;
    }
  } else {
    wait_flag = false;
  }
}


void setFrontLeftRPM(int rpm) {
  currentFrontLeftWheelRPM = -1 * md.getM1Speed();
  md.setM1Speed(rpm);
}

void setFrontRightRPM(int rpm) {
  currentFrontRightWheelRPM = md.getM2Speed();
  md.setM2Speed(rpm);
}

void setRearLeftRPM(int rpm) {
  currentRearLeftWheelRPM = -1 * md.getM3Speed();
  md.setM3Speed(rpm);
}

void setRearRightRPM(int rpm) {
  currentRearRightWheelRPM = md.getM4Speed();

  md.setM4Speed(rpm);
}


void command() {
  static unsigned long prev_control_time = 0;
  while (1) {

    while (mutex.try_lock() == 0) {
      threads.yield();
    }

    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {

      Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
      setFrontLeftRPM(req_rpm.motor1);
      setFrontRightRPM(req_rpm.motor2);
      setRearLeftRPM(req_rpm.motor3);
      setRearRightRPM(req_rpm.motor4);
      digitalWrite(led_pin, (led_state) ? HIGH : LOW);
      led_state = !led_state;
      prev_control_time = millis();
    }


    // finished getting data from the sample, so we can unlock
    // the mutex now.
    mutex.unlock();


    //threads.delay(100);
    nh.spinOnce();
    waitForSerialLink(nh.connected());
    threads.yield();
  }
}


void odom() {
  static unsigned long prev_odom_time = 0;
  Kinematics::velocities current_vel;
  while (1) {
    if ((millis() - prev_odom_time) >= (1000 / COMMAND_RATE)) {
      int current_rpm1 = currentFrontLeftWheelRPM;   //rightWheel.getRPM();
      int current_rpm2 = currentFrontRightWheelRPM;  //leftWheel.getRPM();
      int current_rpm3 = currentRearLeftWheelRPM;
      int current_rpm4 = currentRearRightWheelRPM;
      rpmLeft.data = current_rpm1;
      rpmRight.data = current_rpm2;

      rpmLeft_pub.publish(&rpmLeft);
      rpmRight_pub.publish(&rpmRight);

      current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
      //current_vel = kinematics.getVelocities(50, 50, 0, 0);

      //pass velocities to publisher object
      raw_vel_msg.linear_x = current_vel.linear_x;
      raw_vel_msg.linear_y = current_vel.linear_y;
      raw_vel_msg.angular_z = current_vel.angular_z;

      //publish raw_vel_msg
      raw_vel_pub.publish(&raw_vel_msg);
      prev_odom_time = millis();
    }

    nh.spinOnce();
    waitForSerialLink(nh.connected());
    threads.yield();
  }
}

void stop() {
  md.setM1Speed(0);
  md.setM2Speed(0);
  md.setM3Speed(0);
  md.setM4Speed(0);
}


void setup() {

  HWSERIAL1.begin(115200);
  HWSERIAL2.begin(115200);
  HWSERIAL3.begin(115200);
  HWSERIAL4.begin(115200);
  stop();
  pinMode(led_pin, OUTPUT);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_sub);
  nh.subscribe(sub);

  nh.advertise(raw_vel_pub);
  nh.advertise(rpmLeft_pub);
  nh.advertise(rpmRight_pub);

  threads.addThread(command);
  threads.addThread(odom);
}

void loop() {
  nh.spinOnce();
  delay(500);
}