#ifndef _ROS2_H
#define _ROS2_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include "debuglog.h"


#define CHECK_AND_REPORT(statement, error_msg)  \
    LOG_DEBUG("%s ...", error_msg);                       \
    if ((ret = (statement)) != RCL_RET_OK) {    \
        LOG_ERROR(error_msg " error=%d\r\n", ret); \
        return ret; \
    }

#define CHECK_AND_REPORT_NO_ROS(statement, error_msg)  \
    printf("%s ...\r\n", error_msg);                       \
    if ((ret = (statement)) != RCL_RET_OK) {    \
        printf(error_msg " error=%d\r\n", ret); \
        return ret; \
    }

// ROS2 entities
// extern rcl_subscription_t cmd_vel_subscriber;
// extern rclc_support_t support;
// extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_timer_t motor_timer;
extern rcl_timer_t lidar_timer;
extern std_msgs__msg__Int32 sub_msg;
extern rcl_publisher_t odom_publisher;
extern rcl_publisher_t lidar_publisher;
extern rcl_publisher_t tf_publisher;
extern rcl_clock_t * my_clock; // = (rcl_clock_t *)malloc(sizeof(rcl_clock_t));

rcl_ret_t InitROS(
  // void (*lidar_timer_callback)(rcl_timer_t *timer, int64_t last_call_time),
  // void (*motor_timer_callback)(rcl_timer_t *timer, int64_t last_call_time)
);

#endif