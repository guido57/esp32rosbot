// Charging.cpp
#include <Arduino.h>


#include <cmath>
#include <map>
#include <functional>
#include <unordered_set>
#include <geometry_msgs/msg/twist.h>
#include "segments.h"
#include "cartesian.h"
#include "Ld19.h"
#include "charging.h"
#include "battery.h"
#include "odometry.h"
#include "astar.h"

// Constants
const float CROSS_X_THRESHOLD = 0.5;   // Cross the x-axis at x > 0.5 meters
const float APPROACH_Y_THRESHOLD = 0.02;  // Distance threshold to switch to alignment phase
const float APPROACH_X_THRESHOLD = 0.10;  // Distance threshold to stop
const float ANGLE_THRESHOLD = 0.02;    // Angle threshold for alignment
const float Kp_linear = 0.05;           // Proportional gain for linear velocity
const float Kp_angular = 0.1;          // Proportional gain for angular velocity
const float BREADCRUMB_REACHED_THRESHOLD = 0.05; // breadcrumb is reached within 5 cm

Segments *lidarSegments;
extern Ld19 lidar; // declared in Ld19.cpp

// declared in motors.cpp
extern geometry_msgs__msg__Twist cmd_vel_msg;
extern unsigned long last_cmd_vel_msg;
extern unsigned long motor_timeout;

// declare in main.cpp
extern Battery *battery;            // the battery monitor

Charging::Charging() : state(IDLE) {


}

void Charging::init() {
    lidarSegments = new Segments();
    lidarSegments->begin();
    //angularResolution = 2.0 * PI * sizeof(lidar.ranges[0]) / sizeof(lidar.ranges);
    angularResolution = 2.0 * PI / lidar.ranges.size();
    x_filter = new EMAFilter(0.1);   // Adjust alpha as needed
    y_filter = new EMAFilter(0.1);
    theta_filter = new EMAFilter(0.1);
    fillBreadcrumbs();
    
}

// Fill breadcrumbs with a simulated path
void Charging::fillBreadcrumbs(){
    breadcrumbs.push_back({0.0,0.0,0.0});
    breadcrumbs.push_back({0.4,0.0,0.0});
    breadcrumbs.push_back({0.4,0.3,0.0});
    breadcrumbs.push_back({0.4,0.6,0.0});
    breadcrumbs.push_back({0.4,0.8,0.0});
    breadcrumbs.push_back({0.4,1.1,0.0});
    breadcrumbs.push_back({0.4,1.3,-1.57});
    breadcrumbs.push_back({1.0,1.3,-1.57});
    breadcrumbs.push_back({1.8,1.3,-1.57});
    breadcrumbs.push_back({2.5,1.3,-1.57});
    breadcrumbs.push_back({1.8,1.3,-1.57});
    breadcrumbs.push_back({1.0,1.3,-1.57});
    //// breadcrumbs.push_back({0.4,1.1,1.57});
    breadcrumbs.push_back({0.4,1.3,-1.57});
    breadcrumbs.push_back({0.4,0.6,1.57});
     breadcrumbs.push_back({0.4,0.0,1.57});
    // breadcrumbs.push_back({0.4,-0.4,1.57});
    // breadcrumbs.push_back({0.4,0.0,1.57});
}

// Calculate: maximum correlation and midwheels pose by lidar scan
bool Charging::getCurrentPose() {
    Pose lidarPose = {0.0, 0.0, 0.0}; 
    Segments::LineSegment maxCorrSegm;
    float maxCorr = 0.0;

    lidar.uartRx();
    
    unsigned long process_start = millis();
    std::vector <Segments::LineSegment> segments = lidarSegments->findSegments(lidar.ranges, angularResolution);
    // Process the segments
    lidarSegments->detectPatternAll(segments, lidar.ranges, lidar.qualities, 8, 0.20, 0.35);    
    
    // look for the maximum correlation
    for(Segments::LineSegment segm_ : segments){
    
        if(segm_.patternLength == 0)
            continue;
    
        if(segm_.pts == 0)
            continue;

        // printf("PatternLen: %d  LidarDataLen: %d  Max correlation: %.3f  EnergyPerc=%2f uh=%.0f u=%.0f ul=%.0f pts=%.0f\r\n",
        //     segm_.patternLength, segm_.segmentLength, segm_.maxCorrelation, segm_.energyPerc, segm_.uh,segm_.u,segm_.ul,segm_.pts);

        if(segm_.maxCorrelation > maxCorr){
            maxCorrSegm = segm_;
            maxCorr = segm_.maxCorrelation;
        }
    }   
    if(maxCorr < 0.7){
        return false;
    }
        
    Pose patternTargetPose;
    // printf("segm.startIndex=%d segm.endIndex=%d\r\n", segm.startIndex, segm.endIndex);
    if(maxCorrSegm.endIndex > maxCorrSegm.startIndex)
        patternTargetPose = calculateMidpoint(lidar.ranges[maxCorrSegm.startIndex], angularResolution * maxCorrSegm.startIndex, 
            lidar.ranges[maxCorrSegm.endIndex],   angularResolution * maxCorrSegm.endIndex     );
    else
        patternTargetPose = calculateMidpoint(lidar.ranges[maxCorrSegm.startIndex], angularResolution * maxCorrSegm.startIndex, 
            lidar.ranges[maxCorrSegm.endIndex], angularResolution * (lidar.ranges.size() +maxCorrSegm.endIndex)    );
    
    LOG_INFO("Segm:[%d]=%.3f [%d]=%.3f Len=%.3f Corr=%.3f pts=%.0f x_target=%.3f y_target=%.3f th_target=%.3f",
    maxCorrSegm.startIndex,lidar.ranges[maxCorrSegm.startIndex], maxCorrSegm.endIndex, lidar.ranges[ maxCorrSegm.endIndex], maxCorrSegm.length,maxCorrSegm.maxCorrelation, maxCorrSegm.pts,
    patternTargetPose.x, patternTargetPose.y, patternTargetPose.theta );
    
    lidarPose = calculateRelativePose(patternTargetPose);
    
    LOG_DEBUG("Target Pose is x=%.3f y=%.3f theta=%.3f", patternTargetPose.x, patternTargetPose.y, patternTargetPose.theta);
    LOG_DEBUG("Lidar  Pose is x=%.3f y=%.3f theta=%.3f", lidarPose.x, lidarPose.y, lidarPose.theta);
    midWheelsPose = getWheelsMidpointPose(lidarPose);
    LOG_DEBUG("Wheels Pose is x=%.3f y=%.3f theta=%.3f", midWheelsPose.x,midWheelsPose.y, midWheelsPose.theta);
    midWheelsPose.x = x_filter->update(midWheelsPose.x);
    midWheelsPose.y = y_filter->update(midWheelsPose.y);
    midWheelsPose.theta = theta_filter->updateTheta(midWheelsPose.theta);
    LOG_DEBUG("Wheels Pose is x=%.3f y=%.3f theta=%.3f", midWheelsPose.x,midWheelsPose.y, midWheelsPose.theta);
    return true;
}

void Charging::recordBreadcrumb() {
    Pose currentPose = midWheelsPose;
    if (breadcrumbs.empty() || calculateDistance(currentPose, breadcrumbs.back()) > 0.2) {
        breadcrumbs.push_back(currentPose);
        LOG_DEBUG("Now breadcrumbs has %d elements",breadcrumbs.size());
    }
}
 
// Custom comparator for priority queue of Node pointers
struct CompareNodePointers {
    bool operator()(const Charging::Node* a, const Charging::Node* b) const {
        return *a > *b; // Compare based on f-value
    }
};

// Function to calculate distance between two poses
float distance(const Pose& a, const Pose& b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Heuristic function (e.g., Euclidean distance)
int heuristic(const Pose& current, const Pose& goal) {
    return static_cast<int>(distance(current, goal) * 10); // Multiply by 10 for integer cost
}
  
  // Function to check if a pose is valid (within bounds and not an obstacle)
  bool Charging::isValid(const Pose& pose, const std::vector<Obstacle>& obstacles, float mapWidth, float mapHeight) {
      
    // Calculate half widths for checking against center origin
    float halfWidth = mapWidth / 2.0f;
    float halfHeight = mapHeight / 2.0f;


    if (pose.x < -halfWidth || pose.x > halfWidth || pose.y < -halfHeight || pose.y > halfHeight) {
        return false; // Out of bounds
    }

    for (const auto& obstacle : obstacles) {
        if (distance(pose, {obstacle.x, obstacle.y}) < 0.2) { // Adjust 0.2 to obstacle size/clearance
            return false; // Collision with obstacle
        }
    }
    return true;
}

void checkStackUsage() {
    Serial.print("Stack high water mark: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));  // Works in ESP32 Arduino
}
void checkMemory() {
    Serial.print("Free heap: ");
    Serial.println(esp_get_free_heap_size());
}

std::string poseToKey(float x, float y) {
    return std::to_string(x) + "_" + std::to_string(y);
}

// A* pathfinding function.
// Returns a vector of poses from start to goal.
// If no path is found, an empty vector is returned.    
std::vector<PoseInt> Charging::findPath(const Pose& start, const Pose& goal, const std::vector<float>& obstacles, float mapWidth, float mapHeight) {

    unsigned long started_ms, timeout_ms = 3000;
    
    // Initialize A* search
    AStar astar;
    astar.initialize(start, goal, obstacles, mapWidth, mapHeight);
    started_ms = millis();
    while(astar.state == AStar::ASTAR_STARTED && millis() < started_ms + timeout_ms){
        astar.step();
    }
    if(astar.state == AStar::ASTAR_FAILED){
        LOG_ERROR("A* failed to find a path from (%.3f,%.3f) to (%.3f,%.3f)",start.x, start.y, goal.x, goal.y);
        best_path = {};
    }else if(astar.state == AStar::ASTAR_COMPLETE){
        LOG_DEBUG("A* succeeded to find a path from (%.3f,%.3f) to (%.3f,%.3f) with size=%d state=ASTAR_COMPLETE", start.x, start.y, goal.x, goal.y, astar.getPath().size());
        best_path = astar.getPath();
    }
    
    astar.drawMap();

    // free memory before leaving
    astar.visitedSet.clear();
    while (!astar.openSet.empty()) astar.openSet.pop();
    for (NodeInt* node : astar.allNodes) delete node;
        astar.allNodes.clear();
        
    return best_path; // Return the path
}
  
void Charging::navigateToChargingStation() {
    float current;
    float voltage = battery->getBusVoltage() + battery->getShuntVoltage();
            
    float av_min = 0.0;
    float av_max = 0.0;
    bool targetVisible;
    
    // Charging station is at (0, 0) with theta=0
    float x_goal = 0.0, y_goal = 0.0;
    float error_x, error_y, distance_to_goal, desired_theta, error_theta;
    Velocity vel;
    Pose current_breadcrumb, next_breadcrumb;
    float next_breadcrumb_angle , next_breadcrumb_dist;
    int next_breadcrumb_angle_ndx;
    bool next_breadcrumb_clear;
    obstacle_avoid_ret ret;
    bool step_ret;
    unsigned long started_ms, timeout_ms = 300;
    
    float mapWidth    = 2.0;  // Total map width  in meters
    float mapHeight   = 2.0;  // Total map height in meters

    targetVisible = getCurrentPose();
    if( targetVisible == false ){
        //LOG_INFO("Target NOT visible. lastTarget=%lu targetTimeOut=%lu", lastTarget, targetTimeOut);
        if(millis() > lastTarget + targetTimeOut ){
            targetVisible5 = false; // after 5 seconds declare the target not visible
            //LOG_INFO("Timeout: no target after %ld milliseconds.", targetTimeOut);
        }
    }else{ 
        lastTarget = millis();
        targetVisible5 = true;
        //LOG_INFO("Target is visible.");
    }

    switch (state) {
        case IDLE:
            //if(voltage < 7.4 ) {
                if(targetVisible5){
                    state = CROSS_X_AXIS;
                    PoseA = midWheelsPose;
                    velocityState = RobotState::RotatingToPosition;
                }else if( !breadcrumbs.empty()){
                    state = BREADCRUMB_FOLLOWING;
                    PoseA = midWheelsPose;
                    velocityState = RobotState::RotatingToPosition;
                }
            //}
            break;
        
        case BREADCRUMB_FOLLOWING:
        
            if(targetVisible5){
                state = IDLE;
                break;
            }

            // read the actual position from Odometry
            midWheelsPose = getOdomPose();

            if(best_path.size() == 0 && breadcrumbs.size() == 0){
                LOG_INFO("The last breadcrumb(%.3f,%.3f,%.3f) is reached! Stop.", 
                    breadcrumbs.back().x, breadcrumbs.back().y, breadcrumbs.back().theta);
                state = IDLE;
                stop();
            }
            
            // erase the reached path Pose, if any
            if(best_path.size() > 0){
                Pose PoseB = {0.01f * best_path[0].x, 0.01f *  best_path[0].y, 0.0}; // convert to meters        
                if(distance(midWheelsPose, {PoseB.x, PoseB.y, PoseB.theta}) < BREADCRUMB_REACHED_THRESHOLD){
                    LOG_DEBUG("Reached the waypoint (%.3f,%.3f,%.3f). best_path.size()=%d", PoseB.x, PoseB.y, PoseB.theta, best_path.size());
                    best_path.erase(best_path.begin());
                }
                // if the path is empty (i.e. the breadcrumb is reached), remove the breadcrumb as well
                if(best_path.size() == 0){
                    LOG_DEBUG("Reached the breadcrumb (%.3f,%.3f,%.3f). %d breadcrumbs remaining.", 
                        breadcrumbs.back().x, breadcrumbs.back().y, breadcrumbs.back().theta, breadcrumbs.size()-1);
                    breadcrumbs.pop_back();
                }
            }
            
            // move toward the first Pose in the best_path, if any
            if(best_path.size() > 0){
                Pose PoseB = {0.01f * best_path[0].x, 0.01f *  best_path[0].y, 0.0}; // convert to meters        
                vel = computeVelocitySimple(midWheelsPose,PoseB,0.01,1.0,0.01,1.0,0.15,0.22);
                LOG_DEBUG("computed: vel.linear=%.3f vel.angular=%.3f to go from (%.3f,%.3f,%.3f) to (%d,%d,%.3f)", 
                    vel.linear,vel.angular,
                    midWheelsPose.x,midWheelsPose.y,midWheelsPose.theta,
                    best_path[0].x,best_path[0].y,best_path[0].theta);
                move(vel.linear, vel.angular);
            }
            
            // if best_path is empty or the renew_path_timeout is over, get the path to the next breadcrumb
            if(breadcrumbs.size() > 0 && (best_path.size() == 0 || millis() > last_renew_path_ms + renew_path_timeout_ms)) {
                    
                Pose bc = breadcrumbs.back();
                started_ms = millis();
                LOG_DEBUG("Check if the current breadcrumb (%.3f,%.3f) is reachable. Free heap before findPath=%d", bc.x, bc.y, esp_get_free_heap_size());
                best_path = findPath(midWheelsPose, bc, lidar.ranges, mapWidth, mapHeight);
                LOG_DEBUG("findPath took %lu ms. Free heap after findPath=%d", millis()-started_ms, esp_get_free_heap_size());
                last_renew_path_ms = millis();
                if(best_path.size() == 0){
                    LOG_ERROR("A* failed to find a path from (%.3f,%.3f) to (%.3f,%.3f) for %d times",midWheelsPose.x, midWheelsPose.y, bc.x, bc.y, ++astar_fail_count);
                    if(astar_fail_count > 10 && breadcrumbs.size() > 1){
                        LOG_DEBUG("A* failed to find a path for 10 times. Remove the current breadcrumb (%.3f,%.3f).", bc.x, bc.y);
                        breadcrumbs.pop_back();
                        astar_fail_count = 0;
                    }        
                }else {
                    astar_fail_count = 0;
                    LOG_INFO("The breadcrumb (%.3f,%.3f) is reachable from (%.3f,%.3f,%.3f). Waypoints are:", bc.x, bc.y, midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta);
                    for(PoseInt poseint : best_path){
                        LOG_DEBUG("(%d,%d) in decimeters", poseint.x,poseint.y );
                    }
                }
            }else if(breadcrumbs.size() > 1 && calculateDistance(midWheelsPose, breadcrumbs.back()) < 0.3){
                // if the current breadcrumb is pretty close, try to get the path to the next breadcrumb
                Pose bc = breadcrumbs.at(breadcrumbs.size()-2);
                started_ms = millis();
                LOG_DEBUG("best_path.size()=%d. Check if the next breadcrumb (%.3f,%.3f) is reachable", best_path.size(), bc.x, bc.y);
                std::vector<PoseInt> new_path = findPath(midWheelsPose, bc, lidar.ranges, mapWidth, mapHeight);
                //LOG_DEBUG("findPath took %lu ms. Free heap after findPath=%d", millis()-started_ms, esp_get_free_heap_size());
                if(new_path.size() == 0){
                    LOG_ERROR("No valid new_path from find_path to reach (%.3f,%.3f,%.3f)", 
                        bc.x, bc.y, bc.theta);
                }else {
                    LOG_INFO("The breadcrumb (%.3f,%.3f) is reachable", bc.x, bc.y);
                    best_path = new_path;
                    Pose current_bc = breadcrumbs.back();
                    LOG_INFO("Erase the current breadcrumb (%.3f,%.3f). The new_path has size %d", current_bc.x, current_bc.y, best_path.size());
                    breadcrumbs.pop_back();
                }
            }
            break;

        case CROSS_X_AXIS:       // Goal: to x=0.5 y = 0.0
            if(!targetVisible5){
                LOG_INFO("Target not visible! Go to IDLE" );
                state = IDLE;
                break;
            }

            vel = computeVelocity(PoseA, midWheelsPose, {CROSS_X_THRESHOLD, 0.0, 0.0}, 0.01,1.0,0.01,1.0, 0.1, 0.08, velocityState);
                
            if ( (velocityState == AtTarget || (velocityState == RotatingToOrientation && abs(vel.error_theta) < 0.01))) {
                LOG_DEBUG("CROSS_X_AXIS reached the target. Switch to REVERSE_TO_GOAL, midWheels.x=%.3f midWheels.y=%.3f midWheels.theta=%.3f", 
                    midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta);
                state = REVERSE_TO_GOAL;  
                velocityState = RobotState::RotatingToPosition;
                PoseA = midWheelsPose;
                break;
            }
                        
            LOG_INFO("CROSS_X_AXIS vel.lin=%.3f vel.ang=%.3f error_theta=%.3f", 
                vel.linear, vel.angular, vel.error_theta);
            
            move(vel.linear, vel.angular);
            break;

        case REVERSE_TO_GOAL:                   // Goal: Move backward to 0,0 
            if(!targetVisible5){
                LOG_INFO("Target not visible! Go to IDLE" );
                state = IDLE;
                break;
            }

            velocityState = RobotState::MovingToPosition;
            vel = computeVelocity(PoseA, midWheelsPose, {0.0, 0.0, 0.0}, 0.01,1.0,0.01,1.0,0.1,0.05, velocityState);
            
            LOG_INFO("REVERSE_TO_GOAL distance=%.3f midWheels.x=%.3f midWheels.y=%.3f midWheels.theta=%.3f err_theta=%.3f", 
                        vel.distance, midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta, vel.error_theta);
            
            if( vel.distance > 0.2) {
                move( vel.linear, -vel.angular);    // Move backward and rotate
            }else if( vel.distance > 0.1) {
                move( vel.linear, 0);               // Move backward only
            }else {
                // Stop when close enough
                stop();
                state = DOCKING;
                return;  // Exit the function
            }
            break;
        
        case DOCKING:       // Goal: stay into the docking             
            if(!targetVisible5){
                LOG_INFO("Target not visible! Go to IDLE" );
                state = IDLE;

                setOdomPose({0.0,0.0,0.0}); // reset global Pose
                break;
            }
    
            error_x = x_goal - midWheelsPose.x;
            distance_to_goal = fabs(error_x);

            LOG_INFO("DOCKING distance_to_goal=%.3f", distance_to_goal);
            
            current = battery->getCurrentmA();
            if( current > 0.0 ){ // charging the battery
                state = CHARGING;
            } else if (distance_to_goal > 1.5 * APPROACH_X_THRESHOLD) {
                // restart
                state = IDLE;
            } else{
                // Stay in charging station
                return;  
            }
            // reset global Pose    
            setOdomPose({0.0,0.0,0.0});
            break;

        case CHARGING:      // Goal: charging the battery             
            current = battery->getCurrentmA();
            voltage = battery->getBusVoltage() + battery->getShuntVoltage();
            LOG_INFO("CHARGING Current=%.3f mA Voltage=%.3f V", current, voltage);
            setOdomPose({0.0,0.0,0.0}); // reset global Pose
            if(current > 0.0 ){
                state = CHARGING;
            }else
                state = DOCKING;
            break;
    }
}

void Charging::move(float linear_velocity, float angular_velocity) {
    cmd_vel_msg.linear.x  = linear_velocity;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = angular_velocity;

    //printf("cmd_vel_msg.linear.x=%.3f cmd_vel_msg.angular.z=%.3f\r\n", cmd_vel_msg.linear.x,cmd_vel_msg.angular.z);
   
    last_cmd_vel_msg = millis();
}

void Charging::stop() {
    move(0.0, 0.0);
}

