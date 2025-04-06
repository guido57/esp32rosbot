#include <Arduino.h>
#include <math.h>
#include <vector>
#include "cartesian.h"
#include "debuglog.h"


// Function to convert polar from lidar to Cartesian coordinates
void polarToCartesian(float r, float theta, float &x, float &y) {
    x =   r * sin(theta); 
    y = - r * cos(theta);
}

// Rotate anticlockwise a vector by an angle
Point rotatePoint(Point pointA, float angle) {
    Point rotated;
    float c = cos(angle);
    float s = sin(angle);
    rotated.x = pointA.x * c - pointA.y * s;
    rotated.y = pointA.x * s + pointA.y * c;
    return rotated;
}

// Function to calculate the midpoint of a segment in polar coordinates
Pose calculateMidpoint(float r1, float theta1, float r2, float theta2) {
    // Convert polar coordinates to Cartesian for both points
    float x1, y1, x2, y2;
    polarToCartesian(r1, theta1, x1, y1);
    polarToCartesian(r2, theta2, x2, y2);

    // Calculate the midpoint in Cartesian coordinates
    float xMid = (x1 + x2) / 2.0;
    float yMid = (y1 + y2) / 2.0;

    // Calculate the angle of the midpoint in radians
    // From the x axis counterclockwise
    float thetaMid = - atan2(x1-x2,y1-y2 );

    // Return the midpoint
    return {xMid, yMid, thetaMid};
}

// Extract pose A relative to B from the input pose (B relative to A)
Pose calculateRelativePose(Pose& poseBtoA) {
    float x0 = poseBtoA.x;
    float y0 = poseBtoA.y;
    float theta0 = poseBtoA.theta;

    //printf("x0=%.3f y0=%.3f theta0=%.3f\r\n", x0,y0,theta0);

    // Calculate pose A relative to B
    float xA = -(x0 * cos(theta0) + y0 * sin(theta0));
    float yA =  (x0 * sin(theta0) - y0 * cos(theta0));
    float thetaA = -theta0;
    //printf("xA=%.3f yA=%f thetaA=%.3f\r\n", xA,yA,thetaA);

    // Normalize the angle thetaA to the range [-pi, pi]
    while (thetaA > M_PI) thetaA -= 2 * M_PI;
    while (thetaA < -M_PI) thetaA += 2 * M_PI;

    return {xA, yA, thetaA};
}

// Function to calculate the wheels' midpoint pose from the LiDAR pose
Pose getWheelsMidpointPose(const Pose &lidarPose) {
    Pose wheelsPose;

    // LIDAR_Y_OFFSET is 0, therefore distance = LIDAR_X_OFFSET

    // Position transformation
    wheelsPose.x = lidarPose.x - LIDAR_X_OFFSET * cos(lidarPose.theta);
    wheelsPose.y = lidarPose.y - LIDAR_X_OFFSET * sin(lidarPose.theta);

    // Orientation transformation
    wheelsPose.theta = lidarPose.theta;

    return wheelsPose;
}

float calculateDistance(Pose poseA, Pose poseB){
    float dx = poseA.x - poseB.x;
    float dy = poseA.y - poseB.y;
    return sqrt( dx * dx + dy * dy );
}

Pose lastPoseB = {0.0, 0.0, 0.0};

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

float clamp_abs(float value, float zero_abs_value, float min_abs_value, float max_abs_value) {
    float abs_value = std::abs(value);
  
    if (abs_value < zero_abs_value) {
        return 0.0; // Ignore small values
    } else if (abs_value < min_abs_value) {
        return (value > 0) ? min_abs_value : -min_abs_value;  // Preserve the sign
    } else if (abs_value > max_abs_value) {
      return (value > 0) ? max_abs_value : -max_abs_value; // Preserve the sign
    } else {
      return value; // Value is already within the linear range
    }
}

// Function to compute signed distance
double signedOrthoDistance(Pose poseA, Pose poseB) {
    // Compute the forward direction of the rover
    double dir_x = std::cos(poseA.theta);
    double dir_y = std::sin(poseA.theta);

    // Compute the vector from PoseA to PoseB
    double v_x = poseB.x - poseA.x;
    double v_y = poseB.y - poseA.y;

    // Compute sign
    return v_x * dir_x + v_y * dir_y;
}

float angleToPose(const Pose& actualPose, const Pose& destPose) {
    // Calculate the difference in x and y coordinates
    float dx = destPose.x - actualPose.x;
    float dy = destPose.y - actualPose.y;

    // Compute the angle to the breadcrumb in global coordinates
    float global_angle = atan2(dy, dx); // Angle in radians

    // Convert the rover's orientation (assumed to be in radians) to global frame
    float rover_angle = actualPose.theta;

    // Compute the relative angle from the rover's perspective
    float relative_angle = global_angle - rover_angle;

    // Normalize the angle to the range [-π, π]
    while (relative_angle > M_PI) relative_angle -= 2 * M_PI;
    while (relative_angle < -M_PI) relative_angle += 2 * M_PI;

    return relative_angle; // Angle in radians
}

// Global variables 
float previous_linear_velocity = 0.0;
float previous_angular_velocity = 0.0;
// const float alpha = 0.1; // Smoothing factor (0 < alpha < 1)

// Function to apply low-pass filter
float lowPassFilter(float current_value, float previous_value, float alpha) {
    return alpha * current_value + (1 - alpha) * previous_value;
}

Velocity computeVelocitySimple(Pose currentPose, Pose PoseB,
    float min_linear, float max_linear,
    float min_angular, float max_angular,
    float Kp_linear, float Kp_angular) {
    Velocity vel;

    float dx = PoseB.x - currentPose.x;
    float dy = PoseB.y - currentPose.y;

    float angle_to_target_global = std::atan2(dy, dx);
    float angle_diff_to_target = normalize_angle(angle_to_target_global - currentPose.theta);

    float angular_velocity = 0.0;
    float linear_velocity = 0.0;
    float signed_ortho_distance = signedOrthoDistance(currentPose, PoseB);
    
    float distance = sqrt(dx*dx+dy*dy);    

    linear_velocity = Kp_linear * signed_ortho_distance; // Use signed distance for velocity calculation
    linear_velocity = clamp_abs(linear_velocity,0.001f,min_linear,max_linear);
    vel.linear = linear_velocity;

    angular_velocity = Kp_angular * angle_diff_to_target;
    angular_velocity = clamp_abs(angular_velocity,0.005f,min_angular,max_angular);

    // give priority to angular velocity 
    // if(abs(angular_velocity) > min_angular )
    //     linear_velocity /= 10.0;

    LOG_DEBUG("From (%.3f,%.3f,%.3f) to (%.3f,%.3f,%.3f). signed_ortho_distance=%.3f. linear_velocity=%.3f angular_velocity=%.3f",
        currentPose.x,currentPose.y,currentPose.theta,
        PoseB.x,PoseB.y,PoseB.theta, signed_ortho_distance,
        linear_velocity, angular_velocity
    );    


    // Apply low-pass filter to smooth velocities
    vel.linear = lowPassFilter(linear_velocity, previous_linear_velocity, 1.0);
    vel.angular = lowPassFilter(angular_velocity, previous_angular_velocity, 1.0);

    // Update previous velocities
    previous_linear_velocity = vel.linear;
    previous_angular_velocity = vel.angular;

    // Add the error to the velocity message 
    vel.error_theta = angle_diff_to_target;
    vel.angular = angular_velocity;
    vel.distance = distance;
    
    return vel;
}

Velocity computeVelocity(Pose PoseA, Pose currentPose, Pose PoseB,
    float min_linear, float max_linear,
    float min_angular, float max_angular,
    float Kp_linear, float Kp_angular, int& current_state) {
    Velocity vel;

    float dx = PoseB.x - currentPose.x;
    float dy = PoseB.y - currentPose.y;

    float angle_to_target_global = std::atan2(dy, dx);
    float angle_diff_to_target = normalize_angle(angle_to_target_global - currentPose.theta);

    float angular_velocity = 0.0;
    float linear_velocity = 0.0;
    
    vel.error_theta = normalize_angle(PoseB.theta - currentPose.theta);
    float signed_ortho_distance = signedOrthoDistance(currentPose, PoseB);
    float distance = sqrt(dx*dx+dy*dy);    

    switch (current_state) {
        case RobotState::RotatingToPosition:
            if (std::abs(angle_diff_to_target) > 0.15) { // max error = 0.15 rad
                angular_velocity = Kp_angular * angle_diff_to_target;
                angular_velocity = clamp_abs(angular_velocity,0.0005f, min_angular,max_angular);
                //angular_velocity *= (std::min(1.0f, std::abs(angle_diff_to_target) / 0.3f)); // Tune 0.1
                LOG_DEBUG("RotatingToPosition angular_velocity=%.3f angle_diff_to_target=%.3f > 0.15 rad",
                    angular_velocity, angle_diff_to_target);
            } else {
                current_state = RobotState::MovingToPosition;
            }
            break;

        case RobotState::MovingToPosition:

            if (std::abs(signed_ortho_distance) > 0.01) {  // Use signed distance for the condition too.
                linear_velocity = Kp_linear * signed_ortho_distance; // Use signed distance for velocity calculation
                linear_velocity = clamp_abs(linear_velocity,0.001f, min_linear,max_linear);
                // Deceleration based on ABSOLUTE distance (for smooth stopping)
                //linear_velocity *= (std::min(1.0f, std::abs(signed_distance) / 0.05f)); // Tune 0.05

                angular_velocity = Kp_angular * angle_diff_to_target;

                angular_velocity = clamp_abs(angular_velocity,0.02f, min_angular,max_angular);
                // if(abs(angular_velocity) > 0.4 * abs(linear_velocity))
                //     angular_velocity = angular_velocity * 0.4 * abs(linear_velocity) / abs(angular_velocity);
                //angular_velocity *= (std::min(1.0f, std::abs(angle_diff_to_target) / 0.1f)); // Tune 0.1

                vel.error_theta = angle_diff_to_target;
                LOG_DEBUG("MovingToPosition distance=%.3f, linear_velocity=%.3f angular_velocity=%.3f", 
                    distance, linear_velocity, angular_velocity);
            } else {
                current_state = RobotState::RotatingToOrientation;
            }
            vel.distance = distance;
            break;

        case RobotState::RotatingToOrientation:
            
            if (std::abs(vel.error_theta) > 0.01) {
                // Proportional control with saturation and reduced gain near target
                angular_velocity = Kp_angular * vel.error_theta;
                angular_velocity = clamp_abs(angular_velocity,0.0005f,min_angular,max_angular);
                //angular_velocity *= (std::min(1.0f, std::abs(angle_diff_to_target_orientation) / 0.3f)); // Tune 0.05
                
            } else {
                current_state = RobotState::AtTarget;
                vel.linear = 0.0;
                vel.angular = 0.0;
                vel.distance = distance;
            }
            LOG_DEBUG("RotatingToOrientation angle_to_go=%.3f, angular_velocity=%.3f", vel.error_theta, angular_velocity);            break;
        
        case RobotState::AtTarget:
            LOG_DEBUG("AtTarget");
            vel.linear = 0.0;
            vel.angular = 0.0;
            break;
    }
    
    vel.linear = linear_velocity;
    vel.angular = angular_velocity;
    vel.distance = distance;

    // LOG_DEBUG("Ax=%.3f Ay=%.3f Bx=%.3f By=%.3f vel.lin=%.3f angle_to_target_global=%.3f angle_diff_to_target=%.3f vel.ang=%.3f",
    //     PoseA.x, PoseA.y, PoseB.x, PoseB.y, vel.linear, angle_to_target_global,angle_diff_to_target, vel.angular);                        
    
    return vel;
}

// Function to convert LiDAR data to Cartesian coordinates
void convertLidarToCartesian(std::vector<Obstacle> &obstacles,
                                              const std::vector<float>& ranges, 
                                              float angularResolution) {
    obstacles.clear();                                        
    for (size_t i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        if (range > 0) { // Ignore invalid ranges
            float angle = i * angularResolution - M_PI/2.0; // Angle in radians
            float x = range * cos(angle);
            float y = range * sin(angle);
            obstacles.push_back({x, y});
        }
    }
}

// Compute absolute scanned object position
void computeObjectPosition(Pose middleWheelsPose, float lidarToMiddleWheels, float objectToLidarDistance, float objectTheta, float &objectX, float &objectY) {
    // Precompute trigonometric functions
    float cosThetaM = cosf(middleWheelsPose.theta);
    float sinThetaM = sinf(middleWheelsPose.theta);
    
    float cosThetaMO = cosf(middleWheelsPose.theta + objectTheta);
    float sinThetaMO = sinf(middleWheelsPose.theta + objectTheta);

    // Compute final object coordinates
    objectX = middleWheelsPose.x + lidarToMiddleWheels * cosThetaM + objectToLidarDistance * cosThetaMO;
    objectY = middleWheelsPose.y + lidarToMiddleWheels * sinThetaM + objectToLidarDistance * sinThetaMO;
}

std::vector<Point> roverPoints;
void buildRover(Point middleWheelsPoint){

    roverPoints.clear();
    LOG_DEBUG("Building rover at (%.3f,%.3f)", middleWheelsPoint.x, middleWheelsPoint.y);
    roverPoints.push_back({middleWheelsPoint.x + 0.2f, middleWheelsPoint.y + 0.1f});
    roverPoints.push_back({middleWheelsPoint.x + 0.2f, middleWheelsPoint.y       });
    roverPoints.push_back({middleWheelsPoint.x + 0.2f, middleWheelsPoint.y - 0.1f});
    roverPoints.push_back({middleWheelsPoint.x + 0.1f, middleWheelsPoint.y + 0.1f});
    roverPoints.push_back({middleWheelsPoint.x + 0.1f, middleWheelsPoint.y - 0.1f});
    roverPoints.push_back({middleWheelsPoint.x       , middleWheelsPoint.y + 0.1f});
    roverPoints.push_back({middleWheelsPoint.x       , middleWheelsPoint.y - 0.1f});
    roverPoints.push_back({middleWheelsPoint.x - 0.1f, middleWheelsPoint.y + 0.1f});
    roverPoints.push_back({middleWheelsPoint.x - 0.1f, middleWheelsPoint.y - 0.1f});
    roverPoints.push_back({middleWheelsPoint.x - 0.1f, middleWheelsPoint.y       });
    LOG_DEBUG("roverPoints.size()=%d", roverPoints.size());
    
}

std::vector<Point> rotateRover(float angle){
    for(auto &point : roverPoints){
        point = rotatePoint(point, angle);
    }
    return roverPoints;
}

std::vector<Point> * getRoverPoints(){
    return & roverPoints;
}   