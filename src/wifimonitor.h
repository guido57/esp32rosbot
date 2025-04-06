// wifimonitor.h
#ifndef WIFIMONITOR_H
#define WIFIMONITOR_H

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/string.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>


class WiFiMonitor {
public:
    struct Hotspot {
        String ssid;
        String bssidstr;
        uint8_t bssid[6];
        int32_t strength; // Signal strength in dBm
    };
    WiFiMonitor();
    void begin(rcl_node_t * node);
    void publishStatus();
    void Connect();
    void Connect(Hotspot hotspot);
    String BssidToString(uint8_t * bssid);

private:
    const char* agent_ip;
    uint16_t agent_port;
    const char* topic_name;

    rcl_publisher_t wifi_publisher;
    std_msgs__msg__String wifi_msg;

    char wifi_status[200];
    void checkAndReconnectWiFi();
    

    int publish_count;
    
    
    Hotspot findStrongestHotspot() ;
};

#endif // WIFIMONITOR_H
