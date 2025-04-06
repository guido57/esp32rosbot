// charging.h
#ifndef CHARGING_H
#define CHARGING_H

#include <Arduino.h>
#include <vector>
#include <queue>
#include <geometry_msgs/msg/twist.h>
#include "segments.h"
#include "cartesian.h"
#include "Ld19.h"
#include "ema.cpp"
#include "astar.h"

#define BUF_SIZE 400

class Charging {
public:
    Charging();
    void init();
    void move(float linear_velocity, float angular_velocity);
    void stop();
    bool getCurrentPose();
    void navigateToChargingStation();
    void recordBreadcrumb();
    void followBreadcrumbs();

    Pose lidarPose;
    Pose midWheelsPose;
    Pose PoseA;  // the starting Pose 
    
    EMAFilter * x_filter;   
    EMAFilter * y_filter;
    EMAFilter * theta_filter;

    // Define Node for A* search
    struct Node {
        Pose pose;
        int g; // Cost from start
        int h; // Heuristic cost to goal
        int f; // Total cost (g + h)
        Node *parent;

        bool operator>(const Node& other) const {
            return f > other.f; // For priority queue (min-heap)
        }
    };


private:
     enum State { IDLE, BREADCRUMB_FOLLOWING, BREADCRUMB_FOLLOWING_OLD, CHARGING, DOCKING, CROSS_X_AXIS, /* ROTATE_TO_GOAL,*/ REVERSE_TO_GOAL };
    State state;
    
    int velocityState;

    Segments *lidarSegments;
    float angularResolution;
    bool targetVisible5 = false;
    unsigned long targetTimeOut = 5000UL; // 5 seconds
    unsigned long lastTarget = 0UL;
    std::vector<Pose> breadcrumbs;  // Stores waypoints
    void fillBreadcrumbs();
    
    // Obstacle avoidance    
    enum class obstacle_avoid_ret {
        NO_OBSTACLE,      // No obstacle detected, proceed normally
        TURN_LEFT,        // Turn left to avoid an obstacle
        TURN_RIGHT,       // Turn right to avoid an obstacle
        MOVE_BACKWARD,    // Move backward in recovery mode
        MOVE_FORWARD,     // Move forward after avoiding an obstacle
        STOPPED           // No clear path, must stop
    };
    bool checkAngleClear(int startNdx, int stopNdx,float avoidance_dist);
    obstacle_avoid_ret obstacleAvoidance(Velocity & vel, float avoidance_dist, float turn_speed, float forward_speed) ;
    bool recovering_from_avoidance = false;  // Flag to track recovery state
    float recovery_distance_remaining = 0.0; // Distance left before returning to normal navigation
    const float RECOVERY_DISTANCE = 0.2;     // Distance in meters to move forward after avoiding
    int recovery_steps = 0;
    int RECOVERY_STEP_LIMIT = 10; // Adjust as needed
    int recovery_stage = 0;
    enum recovery_stage_enum  {recStageBackward, recStageTurn, recStageForward};
    float recovery_angle = 0.0;
    std::vector<PoseInt> best_path;
    
    
    std::vector<std::vector<float>> cost_map;   
    
    // A* 
    AStar astar;

    std::vector<Obstacle> obstacles;
        
    std::vector<PoseInt> findPath(const Pose& start, const Pose& goal, const std::vector<float>& obstacles, float mapWidth, float mapHeight) ;
    bool isValid(const Pose& pose, const std::vector<Obstacle>& obstacles, float mapWidth, float mapHeight);
    int astar_fail_count = 0;
    unsigned long last_renew_path_ms = 0;
    unsigned long renew_path_timeout_ms = 3000;
};

#endif // CHARGING_H
