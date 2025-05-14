#include <Arduino.h>
#include <atomic>
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
#include "nav.h"

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

SemaphoreHandle_t xPublishSemaphore;

unsigned long last_cmd_vel_msg = 0L;
void sub_cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist *msg_conv = (const geometry_msgs__msg__Twist *)msg_in;
  cmd_vel_msg = * msg_conv;
  
  last_cmd_vel_msg = millis();
  //printf("Received message: %f\r\n", msg_conv->linear.x);
}

extern WiFiMonitor * wifimonitor;   // declared in main.cpp

rcl_ret_t InitROS()
{
    
  rcl_ret_t ret;
  
  struct my_micro_ros_agent_locator {
      IPAddress address;
      int port;
    } static locator;
  locator.address = ros2_agent_ipa;
  locator.port = ros2_agent_port;

  CHECK_AND_REPORT_NO_ROS(
    rmw_uros_set_custom_transport(false, (void *) &locator, arduino_wifi_transport_open, arduino_wifi_transport_close, arduino_wifi_transport_write, arduino_wifi_transport_read),
    "rmw_uros_set_custom_transport"
  );

  rmw_ret_t rmw_ret;
  
  LOG_DEBUG("Pinging microros agent...");
  rmw_ret = rmw_uros_ping_agent(100, 1);
  if (rmw_ret != RMW_RET_OK) {
    LOG_ERROR("Failed to ping agent. Error %d", rmw_ret);
    return rmw_ret;
  }
  
  printf("rcl_get_default_allocator...\r\n");
  allocator = rcl_get_default_allocator();

  CHECK_AND_REPORT_NO_ROS(rclc_support_init(&support, 0, NULL, &allocator), "rclc_support_init");

  CHECK_AND_REPORT_NO_ROS(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support), "rclc_node_init_default");

  CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"), "rclc_publisher_init_default /odom");

  CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(&lidar_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan"), "rclc_publisher_init_default /scan");
  
  // publish a TFMessage
  CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(&tf_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),"/tf"), "rclc_publisher_init_default /tf");
  
  CHECK_AND_REPORT_NO_ROS(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"), "rclc_subscription_init");

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
  wifimonitor->init_publisher(&node);
  
  CHECK_AND_REPORT_NO_ROS(rcl_clock_init(RCL_ROS_TIME, my_clock,&allocator),"rcl_clock_init");
  
  // Initialize message
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);

  // create executor and add timers
  CHECK_AND_REPORT_NO_ROS(rclc_executor_init(&executor, &support.context, 5, &allocator),"rclc_executor_init");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_timer(&executor, &battery_timer),"rclc_executor_add_timer battery");
  CHECK_AND_REPORT_NO_ROS(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, sub_cmd_vel_callback, ON_NEW_DATA),"rclc_executor_add_subscription");

  // synch timing of this microros session
  CHECK_AND_REPORT_NO_ROS(rmw_uros_sync_session(1000),"rmw_uros_sync_session"); 
  

  // init_publish_mutex()
  xPublishSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xPublishSemaphore);  // Initially give the semaphore so it's available

  return RCL_RET_OK;
}

// This is a workaround to avoid the need of a mutex in the rcl_publish function
// It is not thread safe, but we are not using it in a thread
// It is not a good practice, but it works for now
rcl_ret_t my_rcl_publish(rcl_caller_t caller, const rcl_publisher_t * publisher, const void * ros_message, rmw_publisher_allocation_t * allocation){

  static std::atomic<bool> in_progress(false);
  TaskHandle_t this_task = xTaskGetCurrentTaskHandle();
  const char* task_name = pcTaskGetName(this_task);

  // Detect concurrent access (informational only)
  if (in_progress.exchange(true)) {
    //printf("[my_rcl_publish] CONCURRENT call from task: %s (caller ID: %d)\n", task_name, caller);
  }

  if (xPublishSemaphore == NULL) {
    printf("[my_rcl_publish] ERROR: Semaphore not initialized!\n");
    in_progress = false;
    return RCL_RET_ERROR;
  }

  TickType_t tick_start = xTaskGetTickCount();

  if (xSemaphoreTake(xPublishSemaphore, portMAX_DELAY) == pdTRUE) {
    TickType_t tick_acquired = xTaskGetTickCount();
    uint32_t waited_ms = (tick_acquired - tick_start) * portTICK_PERIOD_MS;

    if (waited_ms > 0) {
      // printf("[my_rcl_publish] Task '%s' waited %lu ms for semaphore\n", task_name, waited_ms);
    }

    rcl_ret_t ret = rcl_publish(publisher, ros_message, allocation);

    xSemaphoreGive(xPublishSemaphore);
    in_progress = false;

    return ret;

  } else {
    printf("[my_rcl_publish] ERROR: Failed to acquire semaphore by task '%s'\n", task_name);
    in_progress = false;
    return RCL_RET_ERROR;
  }
  
}


