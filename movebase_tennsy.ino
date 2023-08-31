const int LED = 13;
#include <Arduino.h>
#include "TeensyThreads.h"


#include "libs/ros.h"
#include "libs/std_msgs/String.h"
#include "libs/std_msgs/Empty.h"

#include "libs/sensor_msgs/Imu.h"
#include "libs/sensor_msgs/MagneticField.h"

#include "libs/std_msgs/Int32.h"
#include "libs/geometry_msgs/Twist.h"
#include "lino_velocities.h"

ros::NodeHandle nh;

std_msgs::Int32 rpmLeft;
std_msgs::Int32 rpmRight;
lino_msgs::Velocities raw_vel_msg;

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

Threads::Mutex mutex;

void commandCallback(const geometry_msgs::Twist& cmd_msg) {
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;


}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher rpmLeft_pub("rpmLeft", &rpmLeft);
ros::Publisher rpmRight_pub("rpmRight", &rpmRight);


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


#define HWSERIAL Serial7

void S7() {
  while (1) {

    while (mutex.try_lock() == 0) {
      threads.yield();
    }


    // finished getting data from the sample, so we can unlock
    // the mutex now.
    mutex.unlock();

    HWSERIAL.write(2);
    threads.delay(100);
    threads.yield();
  }
}


void blink() {
  while (1) {
    rpmRight.data = 10.0;
    rpmRight_pub.publish(&rpmRight);
    rpmLeft.data = 11.0;
    rpmLeft_pub.publish(&rpmLeft);
    digitalWrite(LED, HIGH);  // turn the LED on (HIGH is the voltage level)
    threads.delay(1000);      // wait for a second
    digitalWrite(LED, LOW);   // turn the LED off by making the voltage LOW
    threads.delay(1000);
    nh.spinOnce();
    waitForSerialLink(nh.connected());
    threads.yield();
  }
}


void setup() {
  HWSERIAL.begin(9600);
  pinMode(LED, OUTPUT);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_sub);
  nh.subscribe(sub);

  nh.advertise(raw_vel_pub);
  nh.advertise(rpmLeft_pub);
  nh.advertise(rpmRight_pub);

  threads.addThread(S7);
  threads.addThread(blink);
}

void loop() {
  nh.spinOnce();
  delay(500);
}