#include "debuglog.h"

DebugLogger debugLogger;
DebugLogger::DebugLogger() {}

void DebugLogger::begin(rcl_node_t *node, LogLevel log_level)
{
    
    min_log_level = log_level;
    rclc_publisher_init_default(
        &debug_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/debug_logs"
    );
}

void DebugLogger::log(LogLevel level, const char* functionName, const char *format, ...) {
    if (level < min_log_level) return; // Filtering by log level

    char log_buffer[1024];

    // Get timestamp
    uint32_t timestamp = esp_log_timestamp();

    // Prepare the formatted message
    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    va_end(args);

    // Prefix with timestamp and level
    char final_log[1200];
    
    switch (level) {
        case ERROR: snprintf(final_log, sizeof(final_log), "[ERROR] ❌ [%u] [%s] %s", timestamp, functionName, log_buffer);break;
        case WARN : snprintf(final_log, sizeof(final_log), "[WARN]  ⚠ [%u] [%s] %s", timestamp, functionName, log_buffer);break;
        case INFO : snprintf(final_log, sizeof(final_log), "[INFO]  ℹ [%u] [%s] %s", timestamp, functionName, log_buffer);break;
        case DEBUG: snprintf(final_log, sizeof(final_log), "[DEBUG] ⚙ [%u] [%s] %s", timestamp, functionName, log_buffer);break;
    }

    // Log to Serial
    //printf("%s\r\n", final_log);
    
    // Publish to /debug_logs in ROS2
    log_msg.data.data = final_log;
    log_msg.data.size = strlen(final_log);
    log_msg.data.capacity = log_msg.data.size + 1;
    rcl_ret_t ret = rcl_publish(&debug_pub, &log_msg, NULL);
    if(ret |= RCL_RET_OK)
        printf("DebugLogger::log: rcl_publish returned %d\r\n",ret);
}

const char* DebugLogger::levelToString(LogLevel level) {
    switch (level) {
        case DEBUG: return "DEBUG";
        case INFO:  return "INFO";
        case WARN:  return "WARN";
        case ERROR: return "ERROR";
        default:    return "UNKNOWN";
    }
}
