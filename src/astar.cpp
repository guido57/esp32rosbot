// astar.cpp
#include <Arduino.h>
#include <algorithm>
#include <iostream>
#include "debuglog.h"
#include "astar.h"
#include "cartesian.h"

AStar::AStar() : state(ASTAR_IDLE), current(nullptr) {

    // prepare the rows where print the map to
    for (int i = 0; i < 21; i++) {
        map_row[i] = "                                          ";
    }
}

inline int cmToTile(int cm){
    return (cm + 2) / 5;
}

// Initialize the A* search
void AStar::initialize(const Pose& midWheelsPose, const Pose& goal, const std::vector<float>& ranges_float,float mapWidth, float mapHeight) {
    // convert goal from float (meters) to int (centimeters)
    goalInt = {static_cast<int>(roundf(100.0f * goal.x)),static_cast<int>(roundf(100.0f * goal.y)),goal.theta};
    startInt = {static_cast<int>(roundf(100.0f * midWheelsPose.x)),static_cast<int>(roundf(100.0f * midWheelsPose.y)),midWheelsPose.theta};
    current = nullptr;
    visitedSet.clear();
    while (!openSet.empty()) openSet.pop();
    for (NodeInt* node : allNodes) delete node;
    allNodes.clear();

    // calculate map limits in centimeters
    map_right  = static_cast<int>(round(100.0f * (midWheelsPose.y - mapWidth / 2.0f)));   // y axis in centimeters
    map_left   = static_cast<int>(round(100.0f * (midWheelsPose.y + mapWidth / 2.0f)));   // y axis in centimeters
    map_top    = static_cast<int>(round(100.0f * (midWheelsPose.x + mapHeight / 2.0f)));  // x axis in centimeters
    map_bottom = static_cast<int>(round(100.0f * (midWheelsPose.x - mapHeight / 2.0f)));  // x axis in centimeters
    LOG_DEBUG("map_bottom=%d map_top=%d map_right=%d map_left=%d", map_bottom, map_top, map_right, map_left);

    buildRover({0.0,0.0});        
    std::vector<Point> * roverPoints = getRoverPoints();
    rotateRover(midWheelsPose.theta);

    // clear the rows to print the new map
    for (int i = 0; i < 21; i++) {
        map_row[i] = "                                          ";
    }

    // Draw the rover in rows
    for(Point pr : *roverPoints){
        int prx = static_cast<int>(round(100.0f * (pr.x + midWheelsPose.x)));
        int pry = static_cast<int>(round(100.0f * (pr.y + midWheelsPose.y)));
        map_row[cmToTile(prx-map_bottom)/2][cmToTile(map_left-1-pry)] = map_row[cmToTile(prx-map_bottom)/2][cmToTile(map_left-1-pry)  ] == '-' ? 'X' : 'R';
        // LOG_DEBUG("rover at (%d,%d). cmToTile(prx-map_bottom)/2=%d  cmToTile(map_left-1-pry)=%d", 
        //     prx, pry,cmToTile(prx-map_bottom)/2, cmToTile(map_left-1-pry));
    }
    int prx = static_cast<int>(round(100.0f * (roverPoints->at(1).x + midWheelsPose.x)));
    int pry = static_cast<int>(round(100.0f * (roverPoints->at(1).y + midWheelsPose.y)));
    map_row[cmToTile(prx-map_bottom)/2][cmToTile(map_left-1-pry)] = map_row[cmToTile(prx-map_bottom)/2][cmToTile(map_left-1-pry)  ] == '-' ? 'A' : 'F';
    
    obstacles_str.clear();
    int i = 0;
    for(float range : ranges_float){
        if(range < 0.01) continue; // ignore invalid ranges
        float angularResolution = 2.0 * M_PI / ranges_float.size();
        float angle = i++ * angularResolution - M_PI/2.0; // Angle in radians
            
        float obst_x_float, obst_y_float;
        // get obstacle_x and obstacle_y into world coordinates
        computeObjectPosition(midWheelsPose, LIDAR_X_OFFSET,range, angle, obst_x_float, obst_y_float);
        
        int obst_x = static_cast<int>(round(100.0f * obst_x_float));
        int obst_y = static_cast<int>(round(100.0f * obst_y_float));

        // printf("obstacle range=%.3f angle=%.3f at (%.3f,%.3f) float  (%d,%d) float  in world coordinates map_bottom=%d map_top=%d map_right=%d map_left=%d", 
        //         range, angle, obst_x_float, obst_y_float, obst_x, obst_y, map_bottom, map_top, map_right, map_left);

        // exclude obstacles out of the map    
        if(obst_x < map_bottom || obst_x >= map_top || obst_y < map_right || obst_y >= map_left){
            //LOG_DEBUG("obstacle at (%d,%d) is out of the map", obst_x, obst_y);
            continue;
        }

        // add obstacle to the set. A tile is 5 cm x 5 cm
        std::string key = poseToKey( cmToTile(obst_x), cmToTile(obst_y)); 
        // check if this obstacle is already in the set
        bool obstacle_bool = obstacles_str.find(key) == obstacles_str.end();
        if(obstacle_bool){
            // LOG_DEBUG("inserting an obstacle at %s  range=%.3f angle=%.3f (x=%.3f,y=%.3f) ", 
            //       key.c_str(), range, angle, obst_x_float, obst_y_float);
            obstacles_str.insert(key);

            // LOG_DEBUG("obstacle at (%d,%d) is in the map. cmToTile(obst_x-map_bottom)/2=%d  cmToTile(map_left-1-obst_y)=%d", 
            //     obst_x, obst_y,cmToTile(obst_x-map_bottom)/2, cmToTile(map_left-1-obst_y));    
            map_row[cmToTile(obst_x-map_bottom)/2][cmToTile(map_left - 1 - obst_y)] = '-';
        }
    }

    // Convert the goal from world coordinates and set it in map_row, if it is in the map
    int goalIntX = static_cast<int>(round(100.0f * goal.x ));
    int goalIntY = static_cast<int>(round(100.0f * goal.y ));
    if(goalIntX >= map_bottom && goalIntX < map_top && goalIntY >= map_right && goalIntY < map_left){
        map_row[cmToTile(goalIntX-map_bottom)/2][cmToTile(map_left-1-goalIntY)] = 'G';
    }    
    
    // create the first Node at the Start Pose
    NodeInt* startNode = new NodeInt{ startInt, 0, heuristic(startInt, goalInt), 0, nullptr};
    openSet.push(startNode);
    allNodes.push_back(startNode);
    // started = true;
    // path_ready = false;
    state = ASTAR_STARTED;
}

PointInt rotatedFootprints[8][4] = {
    // Shifted by (+5, 0) without rotation
    {{25, 10}, {25, -10}, {-5, -10}, {-5, 10}},
    // Rotated by PI/4 (around origin counterclockwise) and shifted by (+5, +5)
    {{12, 26}, {26, 12}, {5, -9}, {-9, 5}},
    // Rotated by PI/2 (around origin counterclockwise) and shifted by (+0, +5)
    {{-10, 25}, {10, 25}, {10, -5}, {-10, -5}},
    // Rotated by 3*PI/4 (around origin counterclockwise) and shifted by (-5, +5)
    {{-26, 12}, {-12, 26}, {9, 5}, {-5, -9}},
    // Rotated by PI (around origin counterclockwise) and shifted by (-5, 0)
    {{-25, -10}, {-25, 10}, {5, 10}, {5, -10}},  
    // Rotated by 5*PI/4 (around origin counterclockwise) and shifted by (-5, -5)
    {{-12, -26}, {-26, -12}, {-5, 9}, {9, -5}},
    // Rotated by 3*PI/2 (around origin counterclockwise) and shifted by (0, -5)
    {{10, -25}, {-10, -25}, {-10, 5}, {10, 5}},
    // Rotated by 7*PI/4 (around origin counterclockwise) and shifted by (+5, -5)
    {{26,-12}, {12,-26}, {-9,-5}, {5, 9}}
};

bool AStar::isValidMove(PoseInt pose, int ndx) {
    // Adjust for the position of the pose within the footprint
    int xh = 0, xl = 0, yh = 0, yl = 0;

    switch(ndx){
        case 0:     // along x
            xh=25; xl=-5; yh=10; yl=-10;                            
        break;
        case 1:     // along x,y
            xh=25; xl=-5; yh=25; yl=-5;
        break;
        case 2:     // along y
            xh=10; xl=-10; yh=25; yl=-5;
        break;
        case 3:    // along -x,y
            xh=5; xl=-25; yh=25; yl=-5;
        break;
        case 4:    // along -x
            xh=-5; xl=-25; yh=10; yl=-10;
        break;
        case 5:    // along -x,-y
            xh=-5; xl=-25; yh=-5; yl=-25;
        break;
        case 6:    // along -y
            xh=10; xl=-10; yh=-5; yl=-25;              
        break;
        case 7:    // along x,-y
            xh=25; xl=-5; yh=5; yl=-25;
        break;
    }
    
    for (int dx = xl; dx <=xh; dx+=5) {       
        for (int dy = yl; dy <= yh; dy+=5) {  
            std::string key = poseToKey(cmToTile(pose.x + dx),cmToTile(pose.y + dy));
            if (obstacles_str.find(key) != obstacles_str.end()) {
                map_row[cmToTile(pose.x + dx -map_bottom)/2][cmToTile(map_left - 1 - pose.y - dy)] = (char(48+ndx));
                return false;  // Collision detected
            }
        }
    }
    // Valid move
    //map_row[cmToTile(pose.x -map_bottom)/2][cmToTile(map_left - 1 - pose.y )] = 't';
    return true;
}

// Perform one step of A* search
bool AStar::step() {
    
    if(openSet.empty()){
        LOG_DEBUG("no valid path from (%d,%d) to goal (%d,%d).", 
            startInt.x, startInt.y, goalInt.x, goalInt.y);

        LOG_DEBUG("openSet is empty. Set state=ASTAR_FAILED");
        state = ASTAR_FAILED;
        return false;
    }

    // always start with the top of the openSet which is the node with the lowest total cost
    current = openSet.top();
    openSet.pop();
        
    if ( (cmToTile(current->pose.x - goalInt.x) == 0) && (cmToTile(current->pose.y - goalInt.y)== 0) ){
        LOG_DEBUG("Found a path to (%d,%d) centimeters",
            goalInt.x,goalInt.y     
        );
        state = ASTAR_COMPLETE;
         
        // Include the goal in the final path
        NodeInt* goalNode = new NodeInt{goalInt, current->g + 1, 0, current->g + 1, current};
        current = goalNode;
        allNodes.push_back(goalNode);
        return true;
    }

    // LOG_DEBUG("exploring around (%d,%d,%.3f)",
    //      current->pose.x, current->pose.y, current->pose.theta);

    int  dx[] =     {5   , 5   , 0 ,    -5,  -5,-5    ,      0,  5    };
    int  dy[] =     {0   , 5,    5 ,     5,   0,-5    ,     -5, -5    };
    int  ndx[] =    {0   , 1,    2 ,     3,   4, 5    ,      6,  7    };
    float dtheta[] ={0 , PI/4,PI/2 ,3*PI/4, PI ,5*PI/4, 3*PI/2,7*PI/4 };
    
    for (int i = 0; i < 8; ++i) {
        PoseInt neighborPose = {current->pose.x + dx[i],current->pose.y + dy[i], dtheta[i]};
        std::string key = poseToKey(cmToTile(neighborPose.x),cmToTile(neighborPose.y));    
        // LOG_DEBUG("exploring (%d,%d) from (%d,%d) key=%s",
        //     neighborPose.x, neighborPose.y, current->pose.x, current->pose.y, key.c_str());
        bool no_obstacle = obstacles_str.find(key) == obstacles_str.end();
        if(     neighborPose.y >= map_right  && 
                neighborPose.x >= map_bottom && 
                neighborPose.y < map_left    && 
                neighborPose.x < map_top     
        ){ 
            if (visitedSet.find(key) == visitedSet.end()) {
                visitedSet.insert(key);
                
                if(isValidMove(neighborPose, ndx[i])){
                    int newG = current->g + 1;
                    int newH = heuristic(neighborPose, goalInt);
                    NodeInt* neighbor = new NodeInt{neighborPose, newG, newH, newG + newH, current};
                    
                    openSet.push(neighbor);
                    allNodes.push_back(neighbor);
                }
            }
        }
    }
    if(allNodes.size() > 400){
        LOG_ERROR("A* search occupying more than 400 nodes. HeapSize=%d. Aborting.", esp_get_free_heap_size());
        state = ASTAR_FAILED;
        return false;
    }
    return false;
}

// Get the path from start to goal
std::vector<PoseInt> AStar::getPath()  {
    std::vector<PoseInt> path;
    //if (!path_ready) return path;
    if (state != ASTAR_COMPLETE) return path;
    
    NodeInt* node = current;
    while (node->parent) {
        path.push_back(node->pose);
        node = node->parent;

        // draw the path in the map
        map_row[cmToTile(node->pose.x -map_bottom)/2][cmToTile(map_left - 1 - node->pose.y )] = 
                map_row[cmToTile(node->pose.x -map_bottom)/2][cmToTile(map_left - 1 - node->pose.y )] == 'G' ? 'G' : 'P';
        
    }
    std::reverse(path.begin(), path.end());

    // // Remove collinear points
    std::vector<PoseInt> optimizedPath;
    if (path.size() < 3) {
        return path; // No need to optimize if the path has less than 3 points
    }

    optimizedPath.push_back(path[0]);
    for (size_t i = 1; i < path.size() - 1; ++i) {
        PoseInt& prev = path[i - 1];
        PoseInt& curr = path[i];
        PoseInt& next = path[i + 1];

        // Check if the three points are collinear
        if ((curr.x - prev.x) * (next.y - curr.y) != (curr.y - prev.y) * (next.x - curr.x)) {
            optimizedPath.push_back(curr);
        }
    }
    optimizedPath.push_back(path.back());

    // Remove points closer than 0.15 meters to midWheelsPose
    std::vector<PoseInt> finalPath;
    for (const PoseInt& pose : optimizedPath) {
        float distance = sqrt(pow( 0.01f * (pose.x - startInt.x), 2) + pow(0.01f * ( pose.y - startInt.y), 2));
        if (distance >= 0.15) {
                finalPath.push_back(pose);
        }
    }
    if(finalPath.size()==0){
        // add the last point
        finalPath.push_back(optimizedPath.back());
    }
    return finalPath;
}

void AStar::drawMap(){
    // Draw the map
    LOG_DEBUG(" y  +9+8+7+6+5+4+3+2+1+0-1-2-3-4-5-6-7-8-9-0");
    for(int x = 19; x >= 0; x--) {
        LOG_DEBUG("%3d %s",x-10, map_row[x].c_str());
    }

}

// Convert tile coordinates (x,y) to a string key
std::string AStar::poseToKey(int x, int y) {
    return std::to_string(x) + "_" + std::to_string(y);
}

// Heuristic function for A* search
int AStar::heuristic(const PoseInt& a, const PoseInt& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}
