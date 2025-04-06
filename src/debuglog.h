#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include "esp_log.h"
#include <cstdarg> // Required for variadic function (printf-style logs)

#define TAG "ESP32"

// Log level definitions
enum LogLevel { DEBUG, INFO, WARN, ERROR };

class DebugLogger {
private:
    rcl_publisher_t debug_pub;
    std_msgs__msg__String log_msg;
    // rcl_allocator_t allocator;
    // rclc_support_t support;
    // rcl_node_t node;
    LogLevel min_log_level;

public:
    DebugLogger();
    void log(LogLevel level, const char* functionName, const char *format, ...);
    void begin(rcl_node_t * node, LogLevel log_level = DEBUG);
    
private:
    const char* levelToString(LogLevel level);
};

extern DebugLogger debugLogger; // defined in debuglog.cpp

// Macros for logging at different levels
#define LOG_ERROR(msg, ...) debugLogger.log(ERROR, __FUNCTION__, msg, ##__VA_ARGS__)
#define LOG_DEBUG(msg, ...) debugLogger.log(DEBUG, __FUNCTION__, msg, ##__VA_ARGS__)
#define LOG_INFO(msg, ...)  debugLogger.log(INFO, __FUNCTION__, msg, ##__VA_ARGS__)
#define LOG_WARN(msg, ...)  debugLogger.log(WARN, __FUNCTION__, msg, ##__VA_ARGS__)

#endif // DEBUGLOG_H
