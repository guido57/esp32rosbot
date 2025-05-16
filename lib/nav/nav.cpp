// nav.cpp
#include <Arduino.h>
#include <cmath>
#include <map>
#include <functional>
#include <unordered_set>
#include <geometry_msgs/msg/twist.h>
#include "segments.h"
#include "cartesian.h"
#include "Ld19.h"
#include "nav.h"
#include "battery.h"
#include "odometry.h"
#include "astar.h"
#include "debuglog.h"

// Constants
const float CROSS_X_THRESHOLD = 0.5;              // Cross the x-axis at x > 0.5 meters
const float APPROACH_X_THRESHOLD = 0.10;          // Distance from the charging station to stop
const float BREADCRUMB_REACHED_THRESHOLD = 0.05;  // breadcrumb is reached within 5 cm
// const unsigned CMD_VEL_TIMEOUT = 15 * 60 * 1000; // 15 minutes
const unsigned CMD_VEL_TIMEOUT = 5 * 1000;   // 5 seconds

Segments *lidarSegments;
Breadcrumbs *breadcrumbs;

// declared in Ld19.cpp
extern Ld19 lidar;                                

// declared in motors.cpp
extern geometry_msgs__msg__Twist cmd_vel_msg;
extern unsigned long last_cmd_vel_msg;
extern unsigned long motor_timeout;
extern float actuating_signal_LW;
extern float actuating_signal_RW;

// declared in main.cpp
extern Battery *battery;            // the battery monitor

Nav::Nav() : state(IDLE) {}

void Nav::init() {
    lidarSegments = new Segments();
    lidarSegments->begin();
    angularResolution = 2.0 * PI / lidar.ranges.size();
    x_filter = new EMAFilter(0.1);   // Adjust alpha as needed
    y_filter = new EMAFilter(0.1);
    theta_filter = new EMAFilter(0.1);
    breadcrumbs = new Breadcrumbs();
    fillBreadcrumbs();
}

// Fill breadcrumbs with a simulated path
void Nav::fillBreadcrumbs(){
    // breadcrumbs->breadcrumbs.push_back({0.0,0.0,0.0});
    // breadcrumbs->breadcrumbs.push_back({0.4,0.0,0.0});
    // breadcrumbs->breadcrumbs.push_back({0.4,0.3,0.0});
    // breadcrumbs->breadcrumbs.push_back({0.4,0.6,0.0});
    // breadcrumbs->breadcrumbs.push_back({0.4,0.8,0.0});
    // breadcrumbs->breadcrumbs.push_back({0.4,1.1,0.0});
    // breadcrumbs->breadcrumbs.push_back({0.4,1.3,-1.57});
    // breadcrumbs->breadcrumbs.push_back({1.0,1.3,-1.57});
    // breadcrumbs->breadcrumbs.push_back({1.8,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({2.0,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({1.7,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({1.5,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({1.2,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({1.1,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({1.0,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.9,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.8,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.7,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.6,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.5,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,1.3,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,1.2,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,1.1,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,1.0,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.9,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.8,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.7,-1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.6,1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.5,1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.4,1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.3,1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.2,1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.1,1.57});
    breadcrumbs->breadcrumbs.push_back({0.4,0.0,1.57});
    breadcrumbs->breadcrumbs.push_back({0.3,0.0,1.57});
    breadcrumbs->breadcrumbs.push_back({0.2,0.0,1.57});
    breadcrumbs->breadcrumbs.push_back({0.1,0.0,1.57});
    // breadcrumbs->breadcrumbs.push_back({0.4,-0.4,1.57});
    // breadcrumbs->breadcrumbs.push_back({0.4,0.0,1.57});
}

// Calculate: maximum correlation and midwheels pose by lidar scan
bool Nav::getCurrentPose() {
    Pose lidarPose = {0.0, 0.0, 0.0}; 
    Segments::LineSegment maxCorrSegm;
    float maxCorr = 0.0;

    //lidar.uartRx(); // it was already called in lidar_loop()
    
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
    if(maxCorr < 0.5 || maxCorrSegm.pts < 8){
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
    
    LOG_DEBUG("Segm:[%d]=%.3f [%d]=%.3f Len=%.3f Corr=%.3f pts=%.0f x_target=%.3f y_target=%.3f th_target=%.3f",
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
    LOG_DEBUG("After filtering Wheels Pose is x=%.3f y=%.3f theta=%.3f", midWheelsPose.x,midWheelsPose.y, midWheelsPose.theta);
    return true;
}

void Nav::recordBreadcrumb() {
    Pose currentPose = midWheelsPose;
    if(breadcrumbs->breadcrumbs.empty()){
        breadcrumbs->breadcrumbs.push_back( (Goal&) currentPose);
    }else{
        float dist = calculateDistance(currentPose, breadcrumbs->breadcrumbs.back());
        //LOG_DEBUG("currentPose=(%.1f,%.1f). Distance from last breadcrumb: %.3f. breadcrumbs.size()=%d", 
        //    currentPose.x, currentPose.y, dist, breadcrumbs->breadcrumbs.size());
        if ( dist > 0.15) {
            breadcrumbs->breadcrumbs.push_back({currentPose.x, currentPose.y, currentPose.theta});
            //LOG_DEBUG("Added a breadcrumb at (%.1f, %.1f). Now breadcrumbs has %d elements",
            //    currentPose.x, currentPose.y, breadcrumbs->breadcrumbs.size());
        }
    }
}
 
// Custom comparator for priority queue of Node pointers
struct CompareNodePointers {
    bool operator()(const Nav::Node* a, const Nav::Node* b) const {
        return *a > *b; // Compare based on f-value
    }
};

// // Function to calculate distance between two poses
// float distance(const Pose& a, const Pose& b) {
//     return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
// }

// Heuristic function (e.g., Euclidean distance)
int heuristic(const Pose& current, const Pose& goal) {
    return static_cast<int>(calculateDistance(current, goal) * 10); // Multiply by 10 for integer cost
}
  
  // Function to check if a pose is valid (within bounds and not an obstacle)
  bool Nav::isValid(const Pose& pose, const std::vector<Obstacle>& obstacles, float mapWidth, float mapHeight) {
      
    // Calculate half widths for checking against center origin
    float halfWidth = mapWidth / 2.0f;
    float halfHeight = mapHeight / 2.0f;


    if (pose.x < -halfWidth || pose.x > halfWidth || pose.y < -halfHeight || pose.y > halfHeight) {
        return false; // Out of bounds
    }

    for (const auto& obstacle : obstacles) {
        if (calculateDistance(pose, {obstacle.x, obstacle.y,0,false,0}) < 0.2) { // Adjust 0.2 to obstacle size/clearance
            return false; // Collision with obstacle
        }
    }
    return true;
}

void printStackUsage() {
    Serial.print("Stack high water mark: ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));  // Works in ESP32 Arduino
}
void printHeapMemory() {
    Serial.print("Free heap: ");
    Serial.println(esp_get_free_heap_size());
}

std::string poseToKey(float x, float y) {
    return std::to_string(x) + "_" + std::to_string(y);
}


String Nav::getStateString() const {
    switch (state) {
        case IDLE: return "IDLE";
        case TELEOP: return "TELEOP";
        case BREADCRUMB_FOLLOWING: return "BREADCRUMB_FOLLOWING";
        case CROSS_X_AXIS: return "CROSS_X_AXIS";
        case REVERSE_TO_GOAL: return "REVERSE_TO_GOAL";
        case DOCKING: return "DOCKING";
        case CHARGING: return "CHARGING";
        default: return "UNKNOWN";
    }
}
// ------------------------------------------
// navigate to the charging station
// ------------------------------------------
void Nav::navigateToChargingStation() {
    float current;
    float voltage = battery->getBusVoltage() + battery->getShuntVoltage();
            
    float av_min = 0.0;
    float av_max = 0.0;
    bool targetVisible = false;
    
    // Charging station is at (0, 0) with theta=0
    float x_goal = 0.0, y_goal = 0.0;
    float error_x, error_y, distance_to_goal, desired_theta, error_theta;
    Velocity vel;
    unsigned long started_ms;
    
    float mapWidth    = 2.0;  // Total map width  in meters
    float mapHeight   = 2.0;  // Total map height in meters

    targetVisible = getCurrentPose();
    if( targetVisible == false ){
        if(millis() > lastTarget + targetTimeOut ){
            targetVisible5 = false; // after 5 seconds declare the target not visible
        }
    }else{ 
        // current position was load into midWheelsPose by getCurrentPose()
        lastTarget = millis();
        targetVisible5 = true;
    }
    switch (state) {
        case IDLE:

            // Check if a cmd_vel is received
            if (millis() < last_cmd_vel_msg + CMD_VEL_TIMEOUT) { 
                LOG_INFO("cmd_vel received by %d seconds. Switch from IDLE to TELEOP.", CMD_VEL_TIMEOUT/1000);   ;
                state = TELEOP;
                break;
            }
            //if(voltage < 7.4 ) {
                if(targetVisible5){
                    LOG_INFO("Target visible. Switch from IDLE to CROSS_X_AXIS.");
                    state = CROSS_X_AXIS;
                    PoseA = midWheelsPose;
                    velocityState = RobotState::RotatingToPosition;
                }else if( !breadcrumbs->breadcrumbs.empty()){
                    state = BREADCRUMB_FOLLOWING;
                    LOG_INFO("Raw Breadcrumbs available (%d). Switch from IDLE to BREADCRUMB_FOLLOWING.",
                        breadcrumbs->breadcrumbs.size());
                    int raw_breadcrumbs = breadcrumbs->breadcrumbs.size();
                    
                    breadcrumbs->Clean(); // get rid of the path loops
                    
                    LOG_INFO("No target visible and %d cleaned breadcrumbs available (%d raw breadcrumbs). Switch from IDLE to BREADCRUMB_FOLLOWING.",
                        breadcrumbs->breadcrumbs.size(), raw_breadcrumbs);
                    LOG_DEBUG("Clean Breadcrumbs available (%d).",
                        breadcrumbs->breadcrumbs.size());
                    for(Goal bc : breadcrumbs->breadcrumbs){ 
                        LOG_DEBUG("Breadcrumb (%.3f,%.3f,%.3f)", bc.x, bc.y, bc.theta); 
                    }    
                    PoseA = midWheelsPose;
                    velocityState = RobotState::RotatingToPosition;
                }else{
                    LOG_DEBUG("No target visible and no breadcrumbs. Stay in IDLE.");
                }
            //}
            break;

        case TELEOP:
            // Check if no cmd_vel received for 15 minutes
            if (millis() > last_cmd_vel_msg + CMD_VEL_TIMEOUT) { // 15 minutes in milliseconds
                LOG_INFO("No cmd_vel received for %d seconds. Switch from TELEOP to IDLE.", CMD_VEL_TIMEOUT/1000);
                state = IDLE;
                break;
            } 

            // read the actual position from Odometry
            midWheelsPose = getOdomPose();

            // Collect breadcrumbs
            recordBreadcrumb();
            break;

        case BREADCRUMB_FOLLOWING:
        
            if(targetVisible5){
                LOG_INFO("Target visible! Switch from BREADCRUMB_FOLLOWING TO CROSS_X_AXIS" );
                state = CROSS_X_AXIS;
                break;
            }

            if (millis() < last_cmd_vel_msg + CMD_VEL_TIMEOUT) { 
                LOG_INFO("cmd_vel received at %lu ms while following breadcrumbs. Switch FROM BREADCRUMB_FOLLOWING to TELEOP.", last_cmd_vel_msg);
                state = TELEOP;
                break;
            }
            
            if(breadcrumbs->breadcrumbs.empty()){
                LOG_INFO("No breadcrumbs. Switch from BREADCRUMB_FOLLOWING to IDLE.");
                state = IDLE;
                break;
            }

            // read the actual position from Odometry
            midWheelsPose = getOdomPose();

            // Check if the last breadcrumb is reached
            if(breadcrumbs->breadcrumbs.size() == 0){
                LOG_INFO("The last breadcrumb(%.3f,%.3f,%.3f) is reached! Stop and Switch from BREADCRUMB_FOLLOWING to IDLE.", 
                    breadcrumbs->breadcrumbs.back().x, breadcrumbs->breadcrumbs.back().y, breadcrumbs->breadcrumbs.back().theta);
                state = IDLE;
                stop();
            }
            
            // erase the reached breadcrumb, if any
            if(breadcrumbs->breadcrumbs.size() > 0){
                Pose PoseB = {0.01f * best_path[0].x, 0.01f *  best_path[0].y, 0.0}; // convert to meters        
                if(calculateDistance(midWheelsPose, breadcrumbs->breadcrumbs.back()) < BREADCRUMB_REACHED_THRESHOLD){
                    LOG_DEBUG("Reached the breadcrumb (%.3f,%.3f,%.3f)", 
                        breadcrumbs->breadcrumbs.back().x, breadcrumbs->breadcrumbs.back().y, breadcrumbs->breadcrumbs.back().theta);
                        breadcrumbs->breadcrumbs.pop_back();
                }
            }
                        
            // if the renew_path_timeout is over, try to find a new path to the farthest reachable using astar
            if(astar.state == AStar::ASTAR_IDLE && breadcrumbs->breadcrumbs.size() > 0 && millis() > last_renew_path_ms + renew_path_timeout_ms ){
                LOG_DEBUG("Renew path timeout. Try to find a new path using astar. bc.size()=%d",breadcrumbs->breadcrumbs.size());
                last_renew_path_ms = millis();
                // scan and check all the breadcrumbs to find the farthest inside the local map
                //LOG_DEBUG("midWheelsPose=(%.3f,%.3f,%.3f)", midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta);
                astar.initialize(midWheelsPose, & (breadcrumbs->breadcrumbs), lidar.ranges, mapWidth, mapHeight); // set ASTAR_STARTED
            }
            
            if(astar.state == AStar::ASTAR_STARTED){
                while(astar.state == AStar::ASTAR_STARTED){
                    astar.step();
                    if(astar.steps %50 == 0){
                        break;
                    }
                }
            }    

            if(astar.state == AStar::ASTAR_COMPLETE || astar.state == AStar::ASTAR_FAILED){
                
                int frg = astar.getFarthestReachableGoal();
                if(frg < 0){
                    LOG_DEBUG("No breadcrumb is reachable. Stop the motors.");
                    stop();
                    no_way = true;
                }else{
                    LOG_DEBUG("The farthest reachable goal is breadcrumbs[%d]=(%0.2f,%0.2f) with distance=%.3f from midWheelsPose(%.3f,%.3f,%.3f)", 
                        frg, breadcrumbs->breadcrumbs[frg].x, breadcrumbs->breadcrumbs[frg].y, 
                        calculateDistance(midWheelsPose, breadcrumbs->breadcrumbs[frg]),
                        midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta);
                    // get the farthest reachable goal
                    std::vector<PoseInt> new_path = astar.getPath(frg);
                    if(new_path.size() == 0){
                        LOG_ERROR("frg=%d but no path. Stop the motors",frg);
                        stop();
                        no_way = true;
                    }else{
                        LOG_DEBUG("A* succeeded to find a path from (%d,%d) to (%d,%d) with size=%d", 
                            new_path[0].x, new_path[0].y, new_path.back().x, new_path.back().y, new_path.size());
                        LOG_DEBUG("the new_path is:");      
                        for(PoseInt p : new_path){ 
                            LOG_DEBUG("PoseInt (%.2f,%.2f)", 0.01f * p.x, 0.01f * p.y); 
                        }

                        // Found a path to frg. Erase all the breadcrumbs from the current one to there.
                        breadcrumbs->breadcrumbs.erase(breadcrumbs->breadcrumbs.begin()+frg+1, breadcrumbs->breadcrumbs.end());
                        LOG_DEBUG("Found a reachable breadcrumb (%d,%d) with path size=%d. Erase breadcrumbs from %d to the end", 
                            new_path.back().x, new_path.back().y, new_path.size(), frg+1);
                        LOG_DEBUG("copy the full new path from the end to 0 into breadcrumbs->breadcrumbs from %d to the end ",frg+1);
                        for(int ndx = new_path.size()-2; ndx >= 0; ndx--){
                            PoseInt p = new_path.at(ndx);
                            Goal p2 = {0.01f * p.x, 0.01f * p.y, 0.0, false,0}; // convert to meters
                            breadcrumbs->breadcrumbs.push_back(p2);
                        }
                            
                        // the updated breadcrumbs are
                        for(Goal bc : breadcrumbs->breadcrumbs){ 
                            LOG_DEBUG("Breadcrumb (%.3f,%.3f,%.3f)", bc.x, bc.y, bc.theta); 
                        }
                        no_way = false;
                    }
                }
                astar.state = AStar::ASTAR_IDLE;
                LOG_DEBUG("%s Switch from BREADCRUMB_FOLLOWING to IDLE",
                    astar.state == AStar::ASTAR_COMPLETE ? "ASTAR_COMPLETE" : "ASTAR_FAILED" );
                
                astar.drawMap();
                        
            }    

            // move toward the first Pose of breadcrumbs
            if(breadcrumbs->breadcrumbs.size() > 0 && no_way == false){

                // find a breadcrumb not closer than 15 cm
                Goal GoalB;
                bool foundBreadcrumb = false;
                int ndx;
                // Iterate through breadcrumbs to find one not closer than 15 cm
                for (ndx = breadcrumbs->breadcrumbs.size()-1; ndx>= 0; ndx --) {
                    GoalB = breadcrumbs->breadcrumbs[ndx]; // Get the current breadcrumb
                    float dist = calculateDistance(midWheelsPose, GoalB);
                    if (dist >= 0.15) { // Check if the breadcrumb is at least 15 cm away
                        foundBreadcrumb = true;
                        break;          // Stop searching once a valid breadcrumb is found
                    }
                }
                
                if (foundBreadcrumb) {
                    LOG_DEBUG("Found a breadcrumb not closer than 15 cm: (%.3f, %.3f, %.3f)", 
                              GoalB.x, GoalB.y, GoalB.theta);
                } else {
                    LOG_DEBUG("No breadcrumb found that is farther than 15 cm. Use (%.3f, %.3f, %.3f)",
                              breadcrumbs->breadcrumbs[ndx].x, breadcrumbs->breadcrumbs[ndx].y, breadcrumbs->breadcrumbs[ndx].theta);
                }

                vel = computeVelocitySimple(midWheelsPose,{GoalB.x, GoalB.y,GoalB.theta },0.01,0.1,0.01,0.1,0.15,0.4);
                LOG_DEBUG("computed: vel.lin=%.3f vel.ang=%.3f LW=%.0f RW=%.0f to go from (%.3f,%.3f,%.3f) to (%.3f,%.3f,%.3f)", 
                    vel.linear,vel.angular,actuating_signal_LW, actuating_signal_RW,
                    midWheelsPose.x,midWheelsPose.y,midWheelsPose.theta,
                    GoalB.x,GoalB.y,GoalB.theta);
                move(vel.linear, vel.angular);
            }
            break;

        case CROSS_X_AXIS:       // Goal: to x=0.5 y = 0.0
            if(!targetVisible5){
                LOG_INFO("Target not visible! Switch from CROSS_X_AXIS to IDLE" );
                state = IDLE;
                break;
            }

            vel = computeVelocity(PoseA, midWheelsPose, {CROSS_X_THRESHOLD, 0.0, 0.0}, 0.01,1.0,0.01,1.0, 0.12, 0.12, velocityState);
                
            if ( (velocityState == AtTarget || (velocityState == RotatingToOrientation && abs(vel.error_theta) < 0.01))) {
                LOG_INFO("CROSS_X_AXIS reached the target. Switch to REVERSE_TO_GOAL, midWheels.x=%.3f midWheels.y=%.3f midWheels.theta=%.3f", 
                    midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta);
                state = REVERSE_TO_GOAL;  
                velocityState = RobotState::RotatingToPosition;
                PoseA = midWheelsPose;
                break;
            }
                        
            LOG_DEBUG("CROSS_X_AXIS vel.lin=%.3f vel.ang=%.3f error_theta=%.3f mid.x=%.3f mid.y=%.3f mid.theta=%.3f", 
                vel.linear, vel.angular, vel.error_theta, midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta);
            
            move(vel.linear, vel.angular);
            break;

        case REVERSE_TO_GOAL:                   // Goal: Move backward to 0,0 
            if(!targetVisible5){
                LOG_INFO("Target not visible! Switch from REVERSE_TO_GOAL to IDLE" );
                state = IDLE;
                break;
            }

            velocityState = RobotState::MovingToPosition;
            vel = computeVelocity(PoseA, midWheelsPose, {0.0, 0.0, 0.0}, 0.01,1.0,0.01,1.0,0.08,0.05, velocityState);
            
            LOG_DEBUG("REVERSE_TO_GOAL distance=%.3f midWheels.x=%.3f midWheels.y=%.3f midWheels.theta=%.3f err_theta=%.3f", 
                        vel.distance, midWheelsPose.x, midWheelsPose.y, midWheelsPose.theta, vel.error_theta);
            
            if( vel.distance > 0.2) {
                move( vel.linear, -vel.angular);    // Move backward and rotate
            }else if( vel.distance > 0.1) {
                move( vel.linear, 0);               // Move backward only
            }else {
                // Stop when close enough
                stop();
                LOG_INFO("Close enough to target. Switch from REVERSE_TO_GOAL to DOCKING" );
                state = DOCKING;
                return;  // Exit the function
            }
            break;
        
        case DOCKING:       // Goal: stay into the docking             
            if(!targetVisible5){
                LOG_INFO("Target not visible! Switch from DOCKING to IDLE" );
                state = IDLE;

                setOdomPose({0.0,0.0,0.0}); // reset global Pose
                break;
            }
    
            error_x = x_goal - midWheelsPose.x;
            distance_to_goal = fabs(error_x);

            LOG_DEBUG("DOCKING distance_to_goal=%.3f", distance_to_goal);
            
            current = battery->getCurrentmA();
            if( current > 0.0 ){ // charging the battery
                LOG_INFO("DOCKING Current=%.3f mA. Switch to CHARGING", current);
                state = CHARGING;
            } else if (distance_to_goal > 1.5 * APPROACH_X_THRESHOLD) {
                // restart
                state = IDLE;
                LOG_INFO("DOCKING distance_to_goal=%.3f, Switch from DOCKING to IDLE", distance_to_goal);
            } else{
                // Stay in charging station
                return;  
            }
            // reset global Pose    
            setOdomPose({0.0,0.0,0.0});
            breadcrumbs->breadcrumbs.clear();
            break;

        case CHARGING:      // Goal: charging the battery             
            current = battery->getCurrentmA();
            voltage = battery->getBusVoltage() + battery->getShuntVoltage();
            LOG_DEBUG("CHARGING Current=%.3f mA Voltage=%.3f V", current, voltage);
            setOdomPose({0.0,0.0,0.0}); // reset global Pose
            breadcrumbs->breadcrumbs.clear();
            if(current > 0.0 ){
                // charging the battery
            }else{
                LOG_INFO("Charging Current = 0. Switch from CHARGING to DOCKING");
                state = DOCKING;
            }
            break;
    }
}

extern void motors_set_velocity(float linear_velocity, float angular_velocity);

void Nav::move(float linear_velocity, float angular_velocity) {
    motors_set_velocity(linear_velocity, angular_velocity);
}

void Nav::stop() {
    motors_set_velocity(0.0, 0.0);
}

