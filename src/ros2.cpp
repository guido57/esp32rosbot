#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <tf2_msgs/msg/tf_message.h>

#include "ros2.h"
#include "Ld19.h"
#include "battery.h"
#include "wifimonitor.h"
#include "debuglog.h"
#include "charging.h"

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t motor_timer;
rcl_timer_t lidar_timer;
rcl_timer_t odom_timer;
rcl_timer_t battery_timer;
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
rcl_publisher_t tf_publisher;
rcl_clock_t * my_clock = (rcl_clock_t *)malloc(sizeof(rcl_clock_t));

extern IPAddress ros2_agent_ipa;  // declared in credentials.h
extern int ros2_agent_port;       // declared in credentials.h

// declared and initialized in main.cpp setup()
extern Battery *battery;

geometry_msgs__msg__Twist cmd_vel_msg;

unsigned long last_cmd_vel_msg = 0L;
void sub_cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist *msg_conv = (const geometry_msgs__msg__Twist *)msg_in;
  cmd_vel_msg = * msg_conv;
  // // linear speed limit
  // if(cmd_vel_msg.linear.x > 0.05)
  //   cmd_vel_msg.linear.x = 0.05;
  // else if(cmd_vel_msg.linear.x < -0.05)
  //   cmd_vel_msg.linear.x = -0.05;
  // // angular speed limit
  // if(cmd_vel_msg.angular.z > 0.5)
  //   cmd_vel_msg.angular.z = 0.5;
  // else if(cmd_vel_msg.angular.z < -0.5)
  //   cmd_vel_msg.angular.z = -0.5;
  
  last_cmd_vel_msg = millis();
  //printf("Received message: %f\r\n", msg_conv->linear.x);
}

// declared in motors.cpp
extern void motor_timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// declared in odometry.cpp
extern void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time);


// declared in main.cpp
extern Charging myCharging;         // the object to manage navigation to charger and charging 


WiFiMonitor * wifimonitor;

rcl_ret_t InitROS()
{
  wifimonitor = new WiFiMonitor();
  wifimonitor->Connect();
  
  struct my_micro_ros_agent_locator {
      IPAddress address;
      int port;
    } static locator;
  locator.address = ros2_agent_ipa;
  locator.port = ros2_agent_port;

  rcl_ret_t ret;
  CHECK_AND_REPORT_NO_ROS(
    rmw_uros_set_custom_transport(false, (void *) &locator, arduino_wifi_transport_open, arduino_wifi_transport_close, arduino_wifi_transport_write, arduino_wifi_transport_read),
    "rmw_uros_set_custom_transport"
  );

  printf("rcl_get_default_allocator...\r\n");
  allocator = rcl_get_default_allocator();

  CHECK_AND_REPORT_NO_ROS(rclc_support_init(&support, 0, NULL, &allocator), "rclc_support_init");

  CHECK_AND_REPORT_NO_ROS(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support), "rclc_node_init_default");

  CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"), "rclc_publisher_init_default /odom");

  CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(&lidar_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan"), "rclc_publisher_init_default /scan");
  
  // publish a TFMessage
  CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(&tf_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),"/tf"), "rclc_publisher_init_default /tf");
  
  const unsigned int timer_scan_timeout = 20;
  CHECK_AND_REPORT_NO_ROS(rclc_timer_init_default(&lidar_timer, &support, RCL_MS_TO_NS(timer_scan_timeout),*lidar_timer_callback), "rclc_timer_init_default scan_timer_callback");

  CHECK_AND_REPORT_NO_ROS(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"), "rclc_subscription_init");

  const unsigned int timer_timeout = 20;
  CHECK_AND_REPORT_NO_ROS(rclc_timer_init_default(&motor_timer, &support, RCL_MS_TO_NS(timer_timeout), *motor_timer_callback), "rclc_timer_init_default");

  const unsigned int odom_timer_timeout = 20;
  CHECK_AND_REPORT_NO_ROS(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(odom_timer_timeout), *odom_timer_callback), "rclc_timer_init_default odom_timer");

  const unsigned int battery_wifi_timer_timeout = 1000;
  CHECK_AND_REPORT_NO_ROS(rclc_timer_init_default(
    &battery_timer, 
    &support, 
    RCL_MS_TO_NS(battery_wifi_timer_timeout),       
    [](rcl_timer_t *timer, int64_t last_call_time) 
    {
      battery->timer_callback(timer, last_call_time);
      wifimonitor->publishStatus();
    })
    , "rclc_timer_init_default battery_timer");

  battery->init_publisher(node);  
  wifimonitor->begin(&node);
  
  CHECK_AND_REPORT_NO_ROS(rcl_clock_init(RCL_ROS_TIME, my_clock,&allocator),"rcl_clock_init");
  
  // Initialize message
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);

  // create executor and add timers
  CHECK_AND_REPORT_NO_ROS(rclc_executor_init(&executor, &support.context, 5, &allocator),"rclc_executor_init");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_timer(&executor, &motor_timer),"rclc_executor_add_timer motor");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_timer(&executor, &lidar_timer),"rclc_executor_add_timer scan");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_timer(&executor, &odom_timer),"rclc_executor_add_timer odom");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_timer(&executor, &battery_timer),"rclc_executor_add_timer battery");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, sub_cmd_vel_callback, ON_NEW_DATA),"rclc_executor_add_subscription");

  // synch timing of this microros session
  CHECK_AND_REPORT_NO_ROS(rmw_uros_sync_session(1000),"rmw_uros_sync_session"); 

  return RCL_RET_OK;
}
