// The micro_ros_platformio library provides the functions to communicate with ROS2
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#if !defined(ESP32) 
#error This program is only available for ESP32 Dev module or ESP32 D1 mini 
#endif

#include "ros2.h"
#include "Ld19.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "battery.h"
#include "wifimonitor.h"
#include "debuglog.h"
#include "charging.h"
#include "astar.h"

#define LED_PIN 2

// Lidar object
Ld19 lidar;                  // declared in Ld19.cpp
extern void motors_init();   // declared in motors.cpp

Charging myCharging;         // the object to manage navigation to charger and charging 

Battery *battery;            // the battery monitor

AStar astar;
PoseInt start = {0, 0, 0.0f};
PoseInt goal = {15, 18, 0.0f};
std::vector<Pose> obstacles;  // Define obstacles if needed
int mapWidthCm = 400, mapHeightCm = 400; // 40 m x 40 m
bool pathFound = false;

void setup() {
  Serial.begin(921600);
  printf("setup ...\r\n");
  
  lidar.begin();
  delay(1000);
    
  motors_init();
  delay(2000);

  // Initialize Battery object
  battery = new Battery(Wire);

  // init WiFi and ROS2
  rcl_ret_t ret;
  if(RCL_RET_OK != (ret = InitROS())){
    delay(500);
    esp_restart();
  }
  // to begin the debugger we need a valid ROS2 node
  debugLogger.begin(&node, DEBUG);  // Logs DEBUG, INFO, WARN, and ERROR

  // Initialize OTA
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

  myCharging.init();

  // astar.initialize(start, goal, obstacles, mapWidthCm, mapHeightCm);

  // create one obstacle
  obstacles.push_back({5.0, 5.0, 0.0});
  obstacles.push_back({5.0, 6.0, 0.0});
  obstacles.push_back({6.0, 6.0, 0.0});
  obstacles.push_back({6.0, 5.0, 0.0});

  // turn on the blue LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  LOG_INFO("🚀 End of setup!");
}

unsigned long last_run_test = 0L;
unsigned long run_test_duration = 1000L;
unsigned long last_run_nav_to_charging = 0L;
unsigned long nav_to_charging_interval = 100; // milliseconds

int step = 0;

void loop() {
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  if(ret != RCL_RET_OK){
    LOG_WARN("rclc_executor_spin_some error=%d %s",ret, rcutils_get_error_string().str);
    rcl_reset_error();
  }

  // if (!astar.isComplete()) {
  //   bool success = astar.loop();
  //   // bool success = astar.loop();
  //   if (!success) {
  //       Serial.println("A* Failed to find a path.");
  //       delay(5000);  // Wait before restarting
  //       astar.initialize(start, goal, obstacles, mapWidthCm, mapHeightCm);
  //   }
  // } else if (!pathFound) {
  //   pathFound = true;
  //   std::vector<PoseInt> path = astar.getPath();

  //   if (path.empty()) {
  //       LOG_DEBUG("No path found.");
  //   } else {
  //       LOG_DEBUG("Path found:");
  //       for (const auto& p : path) {
  //           LOG_DEBUG("tile %d %d", p.x, p.y);
  //       }
  //   }
  //   delay(5000);  // Wait before restarting the algorithm
  //   pathFound = false;
  //   LOG_DEBUG("astar started");
  //   astar.initialize(start, goal, obstacles, mapWidthCm, mapHeightCm);
  // }

  if( millis()> last_run_nav_to_charging + nav_to_charging_interval){
     myCharging.navigateToChargingStation();
     last_run_nav_to_charging = millis();
  }
  
  

  ArduinoOTA.handle();
}