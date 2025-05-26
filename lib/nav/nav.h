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
#include "breadcrumbs.h"

#define BUF_SIZE 400

class Nav {
public:
    Nav();
    void init();
    void move(float linear_velocity, float angular_velocity);
    void stop();
    bool getCurrentPose();
    void navigateToChargingStation();
    void recordBreadcrumb();
    String getStateString() const;

    // A* 
    AStar astar;
    bool no_way = false;

    Pose lidarPose;
    Pose midWheelsPose;
    Pose PoseA;  // the starting Pose 
    Goal GoalB;  // the current breadcrumb Pose we are 

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
    enum State { IDLE, TELEOP, BREADCRUMB_FOLLOWING, BREADCRUMB_FOLLOWING_OLD, CHARGING, DOCKING, CROSS_X_AXIS, /* ROTATE_TO_GOAL,*/ REVERSE_TO_GOAL };
    State state;
    
    int velocityState;

    Segments *lidarSegments;
    float angularResolution;
    bool targetVisible5 = false;
    unsigned long targetTimeOut = 5000UL; // 5 seconds
    unsigned long lastTarget = 0UL;
    
    void fillBreadcrumbs();
    
    bool recovering_from_avoidance = false;  // Flag to track recovery state
    float recovery_distance_remaining = 0.0; // Distance left before returning to normal navigation
    const float RECOVERY_DISTANCE = 0.2;     // Distance in meters to move forward after avoiding
    int recovery_steps = 0;
    int RECOVERY_STEP_LIMIT = 10; // Adjust as needed
    int recovery_stage = 0;
    enum recovery_stage_enum  {recStageBackward, recStageTurn, recStageForward};
    float recovery_angle = 0.0;
    std::vector<PoseInt> best_path;
        
    std::vector<Obstacle> obstacles;
        
    bool isValid(const Pose& pose, const std::vector<Obstacle>& obstacles, float mapWidth, float mapHeight);
    unsigned long last_renew_path_ms = 0;
    unsigned long renew_path_timeout_ms = 500;
};

#endif // CHARGING_H
