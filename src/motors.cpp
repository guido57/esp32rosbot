#include "MotorController.h"
#include "odometry.h"

// Left Motor PWM pins 
# define PWM_LEFT_1_PIN 4
# define PWM_LEFT_2_PIN 12
// Right Motor PWM pins 
# define PWM_RIGHT_1_PIN 27
# define PWM_RIGHT_2_PIN 32

//creating objects for right wheel and left wheel
//encoder value per revolution of left wheel and right wheel
int tickPerRevolution_LW = 3692; //3692;
int tickPerRevolution_RW = 3692; // 3692;

extern float wheel_radius; // = 0.03625;   // in meters
extern float wheel_base; // = 0.136;       // in meters

// total ticks counters from odometry
extern volatile int enc_r_total;
extern volatile int enc_l_total;

float currentmsL, currentmsR;

//pid constants of left wheel
float kp_l = 600.0; // 1000;  // it was 2.0
float ki_l = 1500.0; // 6000;  // it was 5.0
float kd_l = 0.0; //100; // it was 0.1
//pid constants of right wheel
float kp_r = 600.0; //1000; // it was 2.0
float ki_r = 1500.0; //6000; // it was 5.0
float kd_r = 0.0; //100; // it was 0.1

//pwm parameters setup
const int freq = 30000;
const int resolution = 8;

MotorController leftWheel(PWM_LEFT_1_PIN, PWM_LEFT_2_PIN, tickPerRevolution_LW);
MotorController rightWheel(PWM_RIGHT_1_PIN, PWM_RIGHT_2_PIN, tickPerRevolution_RW);

// declared in ros2.cpp
extern geometry_msgs__msg__Twist cmd_vel_msg;

float linearVelocity, angularVelocity;
float vL, vR;
float actuating_signal_LW, actuating_signal_RW;

void stop_motors(){
    leftWheel.stop(0,1);
    rightWheel.stop(2,3);
    leftWheel.eintegral = 0.0;
    rightWheel.eintegral = 0.0;
    actuating_signal_LW = 0;
    actuating_signal_RW = 0;
}

void attachInterrupts(){

  attachInterrupt(ENCR_A_PIN, encoderr_interrupt, CHANGE);
  attachInterrupt(ENCR_B_PIN, encoderr_interrupt, CHANGE);
  attachInterrupt(ENCL_A_PIN, encoderl_interrupt, CHANGE);
  attachInterrupt(ENCL_B_PIN, encoderl_interrupt, CHANGE);

}

void detachInterrupts(){

  detachInterrupt(ENCR_A_PIN);
  detachInterrupt(ENCR_B_PIN);
  detachInterrupt(ENCL_A_PIN);
  detachInterrupt(ENCL_B_PIN);

}

void motors_control(){
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity  = cmd_vel_msg.linear.x;
  angularVelocity = cmd_vel_msg.angular.z;
  //printf("angularVelocity=%.3f cmd_vel_msg.angular.z=%.3f\r\n",angularVelocity,cmd_vel_msg.angular.z );
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  vL = linearVelocity - angularVelocity * wheel_base / 2;
  vR = linearVelocity + angularVelocity * wheel_base / 2;
  //current wheel rpm is calculated
  detachInterrupts();
      
  float currentRpmL = leftWheel.getRpm(& enc_l_total);
  float currentRpmR = rightWheel.getRpm(& enc_r_total);
  attachInterrupts();
  
  if( !isnan(currentRpmL) && !isnan(currentRpmR)){
  
    // current wheel speed (m/s) is calculated
    currentmsL = currentRpmL * 2 * PI * wheel_radius / 60.0;
    currentmsR = currentRpmR * 2 * PI * wheel_radius / 60.0;

    //pid controlled is used for generating the pwm signal
    float aslw =  leftWheel.pid(vL, currentmsL);
    float asrw = rightWheel.pid(vR, currentmsR);
    if( !isnan(aslw) && !isnan(asrw)){
      actuating_signal_LW = aslw;
      actuating_signal_RW = asrw;
    }else{
        LOG_WARN("PID returned NAN\r\n");
        return;
    }
    // printf("lv=%.3f av=%.3f vL=%.2f vR=%.3f encL=%d encR=%d msL=%.3f msR=%.3f LW=%.0f RW=%.0f corrL=%d  corrR=%d            \r",
    //       linearVelocity, angularVelocity, vL, vR,enc_l_total,enc_r_total, currentmsL, currentmsR, actuating_signal_LW, actuating_signal_RW,
    //       leftWheel.corruptions, rightWheel.corruptions    );
  }
  if (vL == 0 && vR == 0) { 
    stop_motors();
  } else {
    leftWheel.moveBase(actuating_signal_LW, 150, 0, 1);
    rightWheel.moveBase(actuating_signal_RW,150, 2, 3);
  }
}

void motors_init(){

// Set Encoders pins
  pinMode(ENCR_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCR_B_PIN, INPUT_PULLDOWN);
  pinMode(ENCL_A_PIN, INPUT_PULLDOWN);
  pinMode(ENCL_B_PIN, INPUT_PULLDOWN);
  // Attach Encoders Interrupts
  attachInterrupts();

  //initializing the pid constants
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);

  //initializing pwm signal parameters
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);
  ledcAttachPin(PWM_LEFT_1_PIN, 0);
  ledcAttachPin(PWM_LEFT_2_PIN, 1);
  ledcAttachPin(PWM_RIGHT_1_PIN, 2);
  ledcAttachPin(PWM_RIGHT_2_PIN, 3);
}

// this is called every 20 milliseconds
extern unsigned long last_cmd_vel_msg;
unsigned long motor_timeout = 2000L;         // if no cmd_vel is received in 2 second, stop the motors
void motor_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
      if(millis() > last_cmd_vel_msg + motor_timeout){
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;
        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
      }

      // run the motors
      motors_control();
    }
}
