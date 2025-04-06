#include "debuglog.h"
#include "odometry.h"
#include "ros2.h"
#include "MotorController.h"
#include <rosidl_runtime_c/string_functions.h> // For rosidl_runtime_c__String__assign
#include <geometry_msgs/msg/transform_stamped.h>
#include <rcl/time.h>
#include <rmw_microros/rmw_microros.h>
#include <tf2_msgs/msg/tf_message.h>

// Default parameters
float wheel_radius = 0.038;     // in meters
float wheel_base = 0.148;       // in meters

// Odometry variables
volatile int enc_r_total = 0;
volatile int enc_l_total = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // Mutex for critical section

int enc_r_position = 0;
int enc_l_position = 0;
int enc_r_errors = 0;
int enc_l_errors = 0;
float odomX = 0.0, odomY = 0.0, odomTheta = 0.0;

// Encoder states
uint8_t encr_state = 0;
uint8_t encl_state = 0;

nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t odom_publisher;

extern MotorController leftWheel;
extern MotorController rightWheel;

Pose getOdomPose(){
  return {odomX, odomY, odomTheta };
}

void setOdomPose(Pose pose){
    odomX = pose.x;
    odomY = pose.y;
    odomTheta = pose.theta;
}

void update_odometry() {
  float left_distance = (2 * PI * wheel_radius * enc_l_position) / leftWheel.tick;;
  float right_distance = (2 * PI * wheel_radius * enc_r_position) / rightWheel.tick;
  float distance = (left_distance + right_distance) / 2.0;
  float delta_theta = (right_distance - left_distance) / wheel_base;

  odomTheta += delta_theta;
  odomX += distance * cos(odomTheta);
  odomY += distance * sin(odomTheta);
  LOG_DEBUG("update_odometry: x=%f y=%f theta=%f enc_r_pos=%d enc_r_tot=%d enc1_errors=%d enc_l_pos=%d enc_l_tot=%d enc2_errors=%d",
        odomX,odomY,odomTheta,enc_r_position, enc_r_total, enc_r_errors,  enc_l_position, enc_l_total, enc_l_errors );
  
  // Reset encoder positions for next calculation
  enc_r_position = 0;
  enc_l_position = 0;
}

// Function to populate and publish the odom -> base_link transform as a TFMessage
rcl_ret_t publish_odom_to_base_link(double x, double y, double theta) {
    geometry_msgs__msg__TransformStamped transform;

    // Get the current time
    int64_t mil = rmw_uros_epoch_millis();
    int64_t nanos = rmw_uros_epoch_nanos();

    transform.header.stamp.sec = mil / 1000;
    transform.header.stamp.nanosec = nanos % 1000000000;

    // Set the parent and child frames
    transform.header.frame_id.data = const_cast<char*>("odom");
    transform.header.frame_id.size = strlen(transform.header.frame_id.data);
    transform.header.frame_id.capacity = transform.header.frame_id.size + 1;

    transform.child_frame_id.data = const_cast<char*>("base_link");
    transform.child_frame_id.size = strlen(transform.child_frame_id.data);
    transform.child_frame_id.capacity = transform.child_frame_id.size + 1;

    // Set the translation
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;

    // Calculate the quaternion for the yaw angle
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(theta / 2.0);
    transform.transform.rotation.w = cos(theta / 2.0);

    // Create a TFMessage with a single transform
    geometry_msgs__msg__TransformStamped transforms_array[1];
    transforms_array[0] = transform;

    tf2_msgs__msg__TFMessage tf_message;
    tf_message.transforms.data = transforms_array;
    tf_message.transforms.size = 1;
    tf_message.transforms.capacity = 1;

    // Publish the TFMessage
    rcl_ret_t ret = rcl_publish(&tf_publisher, &tf_message, NULL);

    return ret;
}

rcl_ret_t publish_odometry() {

  odom_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
  odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos() %1000000000L;
  odom_msg.header.frame_id.data = const_cast<char*>("odom");
  odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  
  //rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");
  odom_msg.child_frame_id.data = const_cast<char*>("base_link");
  odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  odom_msg.pose.pose.position.x = odomX;
  odom_msg.pose.pose.position.y = odomY;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.z = sin(odomTheta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(odomTheta / 2.0);

  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  rcl_ret_t ret = RCL_RET_OK;
  
  // Publish the odom -> base_link transform
  //printf("rcl_publish odom -> base_link transform\r\n");  
  if( RCL_RET_OK != (ret = publish_odom_to_base_link(odomX, odomY, odomTheta))){
    LOG_ERROR("rcl_publish odom to base_link error=%d",ret);  
    return ret;
  }
    
  // //printf("rcl_publish /odom\r\n");  
  if( RCL_RET_OK != (ret = rcl_publish(&odom_publisher, &odom_msg, NULL))){
    LOG_ERROR("rcl_publish /odom error=%d",ret);  
    return ret;
  }
  
  return ret;
}

void IRAM_ATTR encoderr_interrupt() {

  portENTER_CRITICAL_ISR(&mux);  // Disable interrupts

  uint8_t new_state = (digitalRead(ENCR_A_PIN) << 1) | digitalRead(ENCR_B_PIN);
  //Serial.printf("enc1_state=%d new_state=%d\r\n", enc1_state,new_state);
  if (new_state == FORWARD_TRANSITIONS[encr_state]) {
    enc_r_position++;
    enc_r_total ++;
  } else if (new_state == BACKWARD_TRANSITIONS[encr_state]) {
    enc_r_position--;
    enc_r_total --;
  } else {
    enc_r_errors++;
  }
  encr_state = new_state;

  portEXIT_CRITICAL_ISR(&mux);   // Re-enable interrupts
}

void IRAM_ATTR encoderl_interrupt() {

  portENTER_CRITICAL_ISR(&mux);  // Disable interrupts

  uint8_t new_state = (digitalRead(ENCL_A_PIN) << 1) | digitalRead(ENCL_B_PIN);
  if (new_state == FORWARD_TRANSITIONS[encl_state]) {
    enc_l_position++;
    enc_l_total++;
  } else if (new_state == BACKWARD_TRANSITIONS[encl_state]) {
    enc_l_position--;
    enc_l_total--;
  } else {
    enc_l_errors++;
  }
  encl_state = new_state;

  portEXIT_CRITICAL_ISR(&mux);   // Re-enable interrupts
}


void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
    
      // update_odometry();
      // publish_odometry();
    }

}

