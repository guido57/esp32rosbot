#ifndef __CARTESIAN_H_
#define __CARTESIAN_H_

#include <vector>

// LiDAR offset relative to the wheels' midpoint
const float LIDAR_X_OFFSET = 0.16;  // x offset in meters
const float LIDAR_Y_OFFSET = 0.0;   // y offset in meters

struct Point {
    float x;
    float y;
};

struct PointInt {
    int x;
    int y;
};

struct Pose {
    float x;
    float y;
    float theta; // Angle in radians
};

struct Obstacle {
    float x;
    float y;
};

struct Velocity {
    float linear, angular, error_x, error_y, error_theta, distance;
};

void buildRover(Point middleWheelsPoint);
std::vector<Point> * getRoverPoints();

std::vector<Point> rotateRover(float angle);


// Rotate a vector by an angle
Point rotatePoint(Point pointA, float angle);

// Function to convert polar to Cartesian coordinates
void polarToCartesian(float r, float theta, float &x, float &y);

// Function to calculate the midpoint of a segment in polar coordinates
Pose calculateMidpoint(float r1, float theta1, float r2, float theta2);

// Extract pose A relative to B from the input pose (B relative to A)
Pose calculateRelativePose(Pose& poseBtoA);

// Function to calculate the wheels' midpoint pose from the LiDAR pose
Pose getWheelsMidpointPose(const Pose &lidarPose);

// calculate distance, regardless theta
float calculateDistance(Pose poseA, Pose poseB);

// calculate the angle to go to destPose
float angleToPose(const Pose& actualPose, const Pose& destPose) ;

enum computeVelocityStrategy  {ROTATE_ONLY, MOVE_ONLY, MOVE_ROTATE};

enum RobotState {
    RotatingToPosition,
    MovingToPosition,
    RotatingToOrientation,
    AtTarget
};

// compute linear and angular velocity to got from PoseA to PoseB
// given min and max linear and angular velocity 
// and given linear and angular coefficients
Velocity computeVelocity(Pose PoseA, Pose actualPose, Pose PoseB,
                         float min_linear, float max_linear,
                         float min_angular, float max_angular,
                         float Kp_linear, float Kp_angular, int& velocityState);


// compute linear and angular velocity to got from currentPose to PoseB
// given min and max linear and angular velocity 
// and given linear and angular coefficients
// without any internal current state
Velocity computeVelocitySimple(Pose currentPose, Pose PoseB,
                    float min_linear, float max_linear,
                    float min_angular, float max_angular,
                    float Kp_linear, float Kp_angular);
                        
// convert LiDAR data to Cartesian coordinates
void convertLidarToCartesian(std::vector<Obstacle> &obstacles,
                             const std::vector<float>& ranges, 
                             float angularResolution);

// compute the object position in the world, given:
//      the middle wheels pose, 
//      the distance from the lidar to the middle wheels,    
//      the distance from the object to the lidar 
//      and the object angle
void computeObjectPosition(Pose middleWheelsPose, 
                           float lidarToMiddleWheels, 
                           float objectToLidarDistance, 
                           float objectTheta, 
                           float &objectX, 
                           float &objectY);

#endif