// battery.h
#ifndef BATTERY_H
#define BATTERY_H

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>

class Battery {
public:
  Battery(TwoWire &i2c, uint8_t address = 0x40);
  rcl_ret_t init_publisher(rcl_node_t &node);
  void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
  float getCurrentmA();
  float getBusVoltage();
  float getShuntVoltage();
  
private:
  Adafruit_INA219 ina219;
  TwoWire &i2c_bus;
  uint8_t sensor_address;

  rcl_publisher_t battery_publisher;
  sensor_msgs__msg__BatteryState battery_state_msg;
};

#endif
