// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#if !defined(ESP32) 
#error This program is only available for ESP32 Dev module or ESP32 D1 mini 
#endif

#include "ros2.h"
#include "Ld19.h"
#include "battery.h"
#include "wifimonitor.h"
#include "debuglog.h"
#include "nav.h"

#define LED_PIN 2

extern void motors_init();   // declared in motors.cpp
extern void motors_loop();   // declared in motors.cpp

Nav myNav;                   // the object to manage navigation to charger and charging 
Ld19 lidar;                  // the lidar object         
Battery *battery;            // the battery monitor
WiFiMonitor * wifimonitor;   // the WiFi monitor
extern Breadcrumbs *breadcrumbs; // defined in nav.cpp


// ROS Shared flags
bool ROS_initialized = false;
bool ROS_connected = false;

void setup() {
  Serial.begin(921600);
  printf("setup ...\r\n");
  
  lidar.begin();
  delay(1000);
    
  motors_init();
  delay(2000);

  // set the WiFi LED pin to output
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize Battery object
  battery = new Battery(Wire);

  // Initialize WiFiMonitor
  wifimonitor = new WiFiMonitor();
  wifimonitor->Connect();
    
  // Initialize OTA and its handlers
  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA update complete");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  
  // Initialize Navigation
  myNav.init();

  LOG_INFO("🚀 End of setup!");
}

// Declare variables to store elapsed times and counters
unsigned long current_time = 0UL;
unsigned long loop_time = 0UL;
unsigned long ota_accum = 0, lidar_accum = 0, navigate_accum = 0, motors_accum = 0;
unsigned long ping_accum = 0, spin_accum = 0, freq_accum = 0;
unsigned long measurement_start = 0;
int measurement_count = 0;
uint32_t heap_min = INT32_MAX;
unsigned long led_flash_period_ms = 200;
unsigned long last_led_flash_period_ms = 0UL;

void loop() {
  
  freq_accum +=  1000.0/ ((float)(millis()-loop_time));
  loop_time = millis();
  current_time = millis();
  lidar_loop();                            // Runs even without ROS
  lidar_accum += millis()-current_time;

  current_time = millis();
  myNav.navigateToChargingStation();  // Runs even without ROS
  navigate_accum += millis() - current_time;

  current_time = millis();    
  motors_loop();                           // Runs even without ROS
  motors_accum += millis() - current_time;
  
  // microros loop
  if(ROS_initialized && WiFi.status() == WL_CONNECTED){
    unsigned long start_pinging = millis();
    rmw_ret_t rmw_ret = rmw_uros_ping_agent(100,1);  
    ping_accum += millis() - start_pinging;
    if(rmw_ret != RMW_RET_OK){
      LOG_ERROR("Failed to ping agent. Error %d", rmw_ret);
      ROS_connected = false;
    }else{
      ROS_connected = true;
      current_time = millis();  
      rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      spin_accum += millis() - current_time;
      if (ret != RCL_RET_OK) {
        LOG_WARN("rclc_executor_spin_some error=%d %s", ret, rcutils_get_error_string().str);
        rcl_reset_error();
      }
    }
  }
 
  // The very first time WiFi is connected, initialize microros
  if(!ROS_initialized && WiFi.status() == WL_CONNECTED){
    rcl_ret_t ret = InitROS();
    if (ret != RCL_RET_OK) {
      LOG_ERROR("InitROS failed: %d", ret);
    } else {
      debugLogger.initPublisher(&node, INFO);  // Logs  DEBUG, INFO, WARN, and ERROR
      LOG_INFO("✅ InitROS succeeded.");       // Value     0     1     2          3
      ROS_initialized = true;
    }
  }

  if(WiFi.status() == WL_CONNECTED){
    // blink the blue LED
    if(WiFi.RSSI() < -65){
      // blink the blue LED
      if(millis() - last_led_flash_period_ms > led_flash_period_ms){
        last_led_flash_period_ms = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
    }else{
      // turn on the blue LED
      digitalWrite(LED_PIN, HIGH);
    }
  }else{
        // turn off the blue LED
        digitalWrite(LED_PIN, LOW);
        // try to reconnect to WiFi if disconnected
        wifimonitor->checkAndReconnectWiFi();
  }

  // run OTA
  current_time = millis();
  ArduinoOTA.handle();
  ota_accum += millis() - current_time;
 
  heap_min = min(esp_get_free_heap_size(), heap_min);

  // Increment measurement count and check if 3 seconds have passed
  measurement_count++;
  if (millis() - measurement_start >= 3000) {
    // Calculate averages
    float lidar_avg    = lidar_accum    / (float)measurement_count;
    float navigate_avg = navigate_accum / (float)measurement_count;
    float motors_avg   = motors_accum   / (float)measurement_count;
    float ping_avg     = ping_accum     / (float)measurement_count;
    float spin_avg     = spin_accum     / (float)measurement_count;
    float freq_avg     = freq_accum     / (float)measurement_count;
    float ota_avg      = ota_accum      / (float)measurement_count;

    // Publish the averages
    LOG_INFO("freq:%.1fHz OTA:%.1fms lidar:%.0fms nav:%.0fms motors:%.0fms Ping:%.1fms spin:%.0fms heap:%d bc=%d no_way=%d %s",
             freq_avg, ota_avg, lidar_avg, navigate_avg, motors_avg, ping_avg, spin_avg, 
             heap_min, breadcrumbs->breadcrumbs.size(), myNav.no_way, myNav.getStateString().c_str());
      
    // Reset accumulators and counters
    freq_accum = 0;
    ota_accum = 0;
    lidar_accum = 0;
    navigate_accum = 0;
    motors_accum = 0;
    ping_accum = 0;
    spin_accum = 0;
    measurement_count = 0;
    measurement_start = millis();
    heap_min = INT32_MAX;  
  }
}