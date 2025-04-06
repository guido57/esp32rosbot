// Ld19.cpp
#include "Ld19.h"
#include "ros2.h"
#include <rmw_microros/rmw_microros.h>
#include <vector>
#include <numeric>

rcl_publisher_t lidar_publisher;

extern Ld19 lidar; // declared in main.cpp

Ld19::Ld19() : pointAlign(false), last_angle_prev_scan(-1.0),packet_ndx (POINT_PER_PACK) {}

void Ld19::begin() {
    Serial2.setRxBufferSize(12000);
    Serial2.begin(230400, SERIAL_8N1, 16, 17);
    
    // Init the two vectors
    ranges.assign(400, 0.0f);
    qualities.assign(400, 0.0f);
}

uint8_t Ld19::CalCRC8(uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

int Ld19::GetNextPacket(){
    unsigned long timeout = millis() + 600;
    bool isBuilding = false;
    int bufferIndex = 0;
    
    while (millis()<timeout) {

        uint8_t incomingByte;
        int ser_ret = Serial2.read(&incomingByte,1);
        if(ser_ret == 0)
            continue;    

        if (!isBuilding && incomingByte == HEADER) {
            if (Serial2.peek() == 0x2C) {
                packetBuffer[bufferIndex++] = incomingByte;
                isBuilding = true;
                continue;
            }
        } else if (isBuilding) {
            packetBuffer[bufferIndex++] = incomingByte;

            if (bufferIndex == 2 && incomingByte != 0x2C) {
                isBuilding = false;
                bufferIndex = 0;
                continue;
            }

            if (bufferIndex >= sizeof(packetBuffer)) {
                isBuilding = false;
                uint8_t crc = CalCRC8(packetBuffer, bufferIndex - 1); // Exclude the CRC byte itself
                if (crc == packetBuffer[bufferIndex - 1]) {
                    //Serial.println("CRC check passed!");
                    // a packet is ready in packetBuffer
                    return 1;
                 } else {
                    LOG_ERROR("CRC check failed! packet_ndx=%d", packet_ndx);
                 }
            }
        }
    }
    LOG_ERROR("GetNextPacket timeout. Ser2.av=%d", Serial2.available());
    return 0;
}

int Ld19::uartRx(){
  
    unsigned long timeout = millis() + 3000;
    int num_points = 0;
    bool start_collecting = false;
    float last_angle = last_angle_prev_scan;
    float last_distance, last_quality;
    float delta_angle_interp = 360.0/BUF_SIZE;
    
    while (millis()<timeout) {
        if(packet_ndx == POINT_PER_PACK ){
            // the previous packet was completely examined -> let's get a new one
            int ret = GetNextPacket();
            packet_ndx = 0;
            if(ret == 0)
                break;;
        }

        // Process the packet just stored in packetBuffer
        // Cast the byte array to a structured LiDAR packet for easier access to its fields.
        const LiDARFrameTypeDef* frame = reinterpret_cast<const LiDARFrameTypeDef*>(packetBuffer);
        float startAngle = frame->start_angle * 0.01;
        float endAngle = frame->end_angle * 0.01;
        // Calculate step size, considering wrapping from 360 to 0 degrees
        float step = ((endAngle < startAngle) ? (360 - startAngle + endAngle) : (endAngle - startAngle)) / (POINT_PER_PACK - 1);

        // Loop through each measurement point and get its distance and intensity.
        for (; packet_ndx < POINT_PER_PACK; packet_ndx++) {
            float angle = startAngle + step * packet_ndx;
            if (angle >= 360) angle -= 360; // Wrap around if the angle exceeds 360 degrees
            uint16_t distance = frame->point[packet_ndx].distance;
            uint8_t quality = frame->point[packet_ndx].intensity;
            //if(num_points == 0)
                // printf("num_points=%d packet_ndx=%d last_angle=%.1f angle=%.1f dist=%d quality=%d free_heap=%d min_free_heap=%d Ser2.av=%d\r\n",
                //     num_points, packet_ndx, last_angle, angle, distance, quality, esp_get_free_heap_size(), esp_get_minimum_free_heap_size(), Serial2.available());

            // here you are angle distance and quality
            num_points ++;

            if(angle < last_angle){
                last_angle -= 360.0;
                start_collecting = true;   // start collecting angles
            }
                                    
            if(start_collecting){
                int ndx = angle / delta_angle_interp; // rounding   
                if(ndx <0 || ndx >= BUF_SIZE)
                    LOG_ERROR("ndx is out of bounds ----> ndx=%d <--------------",ndx);

                qualities[BUF_SIZE-1-ndx] = quality;    
                ranges[BUF_SIZE-1-ndx] = distance / 1000.0;    // mm to m
                
                if(ndx == BUF_SIZE -1){
                    // scan completed!
                    // printf("%d points returned. num_points=%d angle=%f last_angle=%f Ser2.av=%d\r\n",
                    //     ndx+1, num_points, angle,last_angle, Serial2.available());
                    
                    last_angle_prev_scan = angle - 360.0; // it should become a little negative value (e.g. -0.7) to be used at the next scan
                    return ndx+1;        
                }
                ndx++;
            }            

            last_angle = angle;
            
        } // end for           
    } // end while

    // got timeout 
    LOG_ERROR("uartRX timeout");
    return 0;
}

void Ld19::processFrame(int pos_min, int pos_max, int total_points) {
    
    // Populate the LaserScan message with the range [pos_min,pos_max]
    
    // Get the current time
    int64_t mil = rmw_uros_epoch_millis();
    int64_t nanos = rmw_uros_epoch_nanos();
    
    // Prepare LaserScan message
    scan_msg.header.frame_id.data = const_cast<char*> ("laser_frame");
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1; // Assuming 10 Hz scan rate
    scan_msg.range_min = 0.05;
    scan_msg.range_max = 10.0;

    scan_msg.angle_min = 0;
    scan_msg.angle_increment = (2 * PI) / ((float)BUF_SIZE);
    scan_msg.angle_max = scan_msg.angle_increment *(BUF_SIZE - 1);
    
    scan_msg.header.stamp.sec = mil / 1000;
    scan_msg.header.stamp.nanosec = nanos % 1000000000;
    //scan_msg.ranges.data = ranges;
    scan_msg.ranges.data = ranges.data();
    scan_msg.ranges.size = BUF_SIZE;
    scan_msg.ranges.capacity = BUF_SIZE;
    //scan_msg.intensities.data = qualities;
    scan_msg.intensities.data = qualities.data();
    scan_msg.intensities.size = BUF_SIZE;
    scan_msg.intensities.capacity = BUF_SIZE;
    // Serial.printf("angle_min=%f angle_max=%f angle_increment=%f \r\n", 
    //     scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment);   
}

// declared in motors.cpp but printed by lidar_loop()
extern float linearVelocity, angularVelocity;
extern float vL, vR;
extern float currentRpmL, currentRpmR;
extern float currentmsL, currentmsR;
extern float actuating_signal_LW, actuating_signal_RW;
extern int enc_r_total;
extern int enc_l_total;
extern float x,y,theta;
extern int enc_r_errors;
extern int enc_l_errors;


extern void update_odometry();
extern void publish_odometry(); 

unsigned long total_loop_time = 0L;
float loop_period = 0.0;
int consecutive_errors = 0;

// the most critical function of this project!

void lidar_loop(){

    // read the laser scan
    unsigned long uart_elapsed = millis();
    int count;
    count = lidar.uartRx();
    uart_elapsed = millis() - uart_elapsed;
    
    unsigned long process_elapsed = millis();
    process_elapsed = millis()-process_elapsed;

    // publish /scan
    unsigned long publish_elapsed = millis();
    int nc = 50;
    lidar.processFrame(0,count-1,count);
    rcl_ret_t ret_pub = rcl_publish(&lidar_publisher, &lidar.scan_msg, NULL);
    
    if(ret_pub != RCL_RET_OK){
        LOG_ERROR("rcl_publish returned %d", ret_pub);
        consecutive_errors ++;
        // if(consecutive_errors > 10)
        //     esp_restart();
    }else
        consecutive_errors = 0;

    publish_elapsed = millis() - publish_elapsed;

    // publish /odom 
    update_odometry(); // read the pose 
    unsigned long pub_odom_elapsed = millis();
    publish_odometry(); 
    pub_odom_elapsed = millis() - pub_odom_elapsed,

    // calculate loop period  
    total_loop_time = millis()-total_loop_time;
    float total_loop_time_f = (float) total_loop_time;
    loop_period = loop_period*0.9 + total_loop_time_f*0.1;

    uint64_t mil = rmw_uros_epoch_millis();
    uint64_t nanos = rmw_uros_epoch_nanos();
    
    LOG_DEBUG("lv=%.3f av=%.3f msL=%.3f msR=%.3f encL=%d encR=%d errL=%d errR=%d LW=%.0f RW=%.0f ||lidar=%dpts uartRx=%lums Proc=%lums Pub=%lums PubOdom=%lums Loop=%lums Freq=%.1fHz Ser2.av=%d",
        linearVelocity, angularVelocity, currentmsL, currentmsR, enc_l_total, enc_r_total,enc_l_errors, enc_r_errors,  actuating_signal_LW, actuating_signal_RW,
        count, uart_elapsed, process_elapsed, publish_elapsed, pub_odom_elapsed, total_loop_time, 1000.0/loop_period, Serial2.available()
    );
    total_loop_time = millis();
}


void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL )  {

    unsigned long lidar_timer_cb_elapsed = millis(); 
    lidar_loop();
    // printf("\r\n%lu lidar_timer_callback took %lu millis. Serial2aval=%d\r\n", 
    //       millis(), millis()-lidar_timer_cb_elapsed, Serial2.available());
  }
}

