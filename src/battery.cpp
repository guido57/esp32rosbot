// battery.cpp
#include "battery.h"
#include <Arduino.h>
#include "ros2.h"
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <vector>
#include <debuglog.h>

Battery::Battery(TwoWire &i2c, uint8_t address) : i2c_bus(i2c), sensor_address(address) {
  if (!ina219.begin(&i2c_bus)) {
    printf("Failed to find INA219 chip!\r\n");
    while (1);
  }
  ina219.setCalibration_32V_2A();
  printf("INA219 Initialized\r\n");

  // Initialize BatteryState message
  battery_state_msg.header.frame_id.data = const_cast<char*>("battery_state");
  battery_state_msg.header.frame_id.size = strlen(battery_state_msg.header.frame_id.data);
  battery_state_msg.header.frame_id.capacity = battery_state_msg.header.frame_id.size + 1;
  
  battery_state_msg.voltage = 0.0;
  battery_state_msg.temperature = NAN;
  battery_state_msg.current = 0.0;
  battery_state_msg.charge = NAN;
  battery_state_msg.capacity = 2.2;
  battery_state_msg.design_capacity = 2.2;
  battery_state_msg.percentage = 0.0;

  battery_state_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_state_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_state_msg.present = true;
  battery_state_msg.serial_number.data =  const_cast<char*>("");
  battery_state_msg.serial_number.size = strlen(battery_state_msg.serial_number.data);
  battery_state_msg.serial_number.capacity = battery_state_msg.serial_number.size + 1;
  battery_state_msg.location.data = const_cast<char*>("on the rover");
  battery_state_msg.location.size = strlen(battery_state_msg.location.data);
  battery_state_msg.location.capacity = battery_state_msg.location.size + 1;
    
 float zerof = 0.0;
 battery_state_msg.cell_temperature.data = &zerof;
 battery_state_msg.cell_temperature.size = 0;
 battery_state_msg.cell_temperature.capacity = 0;

 battery_state_msg.cell_voltage.data = &zerof;
 battery_state_msg.cell_voltage.size = 0;
 battery_state_msg.cell_voltage.capacity = 0;

}

rcl_ret_t Battery::init_publisher(rcl_node_t &node) {

    rcl_ret_t ret;
    CHECK_AND_REPORT_NO_ROS(rclc_publisher_init_default(
      &battery_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state"), "rclc_publisher_init_default battery_state");
    return ret;
}

void Battery::timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  // Read values from INA219
  float bus_voltage = ina219.getBusVoltage_V();
  float shunt_voltage = ina219.getShuntVoltage_mV() / 1000.0;
  float current_mA = -ina219.getCurrent_mA()/5.0;

  // Calculate battery voltage
  float battery_voltage = bus_voltage + shunt_voltage;

  //printf("voltage=%f current_mA=%f\r\n",battery_voltage,current_mA);  

  // Update BatteryState message
  battery_state_msg.voltage = battery_voltage;
  battery_state_msg.current = getCurrentmA() / 1000.0; // Convert mA to A
  LOG_DEBUG("CurrentmA=%.3f", getCurrentmA());
  battery_state_msg.percentage = min(100.0, battery_voltage/8.4);
  battery_state_msg.header.stamp.sec = millis() / 1000;
  battery_state_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

  // Publish the message
  rcl_publish(&battery_publisher, &battery_state_msg, NULL);
}

float Battery::getCurrentmA(){

  return ina219.getCurrent_mA() * 0.1;
}

float Battery::getBusVoltage(){
  return ina219.getBusVoltage_V();
}

float Battery::getShuntVoltage(){
  return ina219.getShuntVoltage_mV() / 1000.0;
}
  