// astar.cpp
#include <Arduino.h>
#include <algorithm>
#include <iostream>
#include "astar.h"
#include "cartesian.h"
#include <debuglog.h>

AStar::AStar() : state(ASTAR_IDLE), current(nullptr) {

    // prepare the rows where print the map to
    for (int i = 0; i < 21; i++) {
        map_row[i] = "                                          ";
    }
}

inline int cmToTile(int cm) {
    return (cm + (cm >= 0 ? 2 : -2)) / 5;
}

NodeInt* AStar::allocateNodeInPsram(const PoseInt& pose, int g, int h, NodeInt* parent) {
    // Allocate memory for NodeInt in PSRAM
    NodeInt* node = static_cast<NodeInt*>(heap_caps_malloc(sizeof(NodeInt), MALLOC_CAP_SPIRAM));
    if (!node) {
        LOG_ERROR("Failed to allocate NodeInt in PSRAM!");
        return nullptr;
    }

    // Initialize the NodeInt object
    node->pose = pose;
    node->g = g;
    node->h = h;
    node->parent = parent;

    return node;
}

// Initialize the A* search
void AStar::initialize(const Pose& midWheelsPose, std::vector<Goal> * _goals, const std::vector<float, PsramAllocator<float>>& ranges_float,float mapWidth, float mapHeight) {
    
    // reindex the goals
    int ndx = 0;
    for(Goal & bc : *_goals){ 
        bc.ndx = ndx++;
        bc.reachable = false;
        LOG_DEBUG("Goal (%.3f,%.3f,%.3f) reachable=%d ndx=%d", bc.x, bc.y, bc.theta, bc.reachable, bc.ndx);
    }

    goals = _goals;
    // convert midWheelsPose from float (meters) to int (centimeters)
    startInt = {static_cast<int>(roundf(100.0f * midWheelsPose.x)),static_cast<int>(roundf(100.0f * midWheelsPose.y))};

    int heap_before_cleaning = esp_get_free_heap_size();

    // Reset the state clearing all the data structures
    current = nullptr;
    visitedSet.clear();
    while (!openSet.empty()) openSet.pop();
    for (NodeInt* node : allNodes) delete node;
    allNodes.clear();
    //reachableGoals.clear();
    obstacles_str.clear();
    goalInts.clear();

// convert goal from float (meters) to int (centimeters)
    ndx = 0;            
    for (Goal& goal : *_goals) {
        goalInts.push_back({static_cast<int>(roundf(100.0f * goal.x)), static_cast<int>(roundf(100.0f * goal.y)), goal.theta, false, ndx++});
    }

    // calculate map limits in centimeters
    map_right  = static_cast<int>(round(100.0f * (midWheelsPose.y - mapWidth / 2.0f)));   // y axis in centimeters
    map_left   = static_cast<int>(round(100.0f * (midWheelsPose.y + mapWidth / 2.0f)));   // y axis in centimeters
    map_top    = static_cast<int>(round(100.0f * (midWheelsPose.x + mapHeight / 2.0f)));  // x axis in centimeters
    map_bottom = static_cast<int>(round(100.0f * (midWheelsPose.x - mapHeight / 2.0f)));  // x axis in centimeters
    //LOG_DEBUG("map_bottom=%d map_top=%d map_right=%d map_left=%d", map_bottom, map_top, map_right, map_left);

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
    }
    int prx = static_cast<int>(round(100.0f * (roverPoints->at(1).x + midWheelsPose.x)));
    int pry = static_cast<int>(round(100.0f * (roverPoints->at(1).y + midWheelsPose.y)));
    map_row[cmToTile(prx-map_bottom)/2][cmToTile(map_left-1-pry)] = map_row[cmToTile(prx-map_bottom)/2][cmToTile(map_left-1-pry)  ] == '-' ? 'A' : 'F';
    
    // add the obstacles to the map
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

        // printf("create obstacle: range=%.3f angle=%.3f at (%.3f,%.3f) float   (%d,%d) int  in world coordinates map_bottom=%d map_top=%d map_right=%d map_left=%d", 
        //          range, angle, obst_x_float, obst_y_float, obst_x, obst_y, map_bottom, map_top, map_right, map_left);

        // exclude obstacles out of the map    
        if(obst_x < map_bottom || obst_x >= map_top || obst_y < map_right || obst_y >= map_left){
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

    // draw the goals in the map
    for(GoalInt goalInt : goalInts){
        if(goalInt.x >= map_bottom && goalInt.x < map_top && goalInt.y >= map_right && goalInt.y < map_left){
            map_row[cmToTile(goalInt.x-map_bottom)/2][cmToTile(map_left-1-goalInt.y)] = 'G';
        }    
    }

    // create the first Node at the Start Pose
    //NodeInt* startNode = new NodeInt{ startInt, 0, 0, nullptr};
    NodeInt* startNode = allocateNodeInPsram( startInt, 0, 0, nullptr);
    openSet.push(startNode);
    allNodes.push_back(startNode);
    state = ASTAR_STARTED;
    steps = 0;
}

// Verify if the move is valid, 
bool AStar::isValidMove(PoseInt pose, int ndx) {
    // Adjust for the position of the pose within the footprint
    int xh = 0, xl = 0, yh = 0, yl = 0;

    switch(ndx){
        case 0:     // 5 cm along x
            xh=25; xl=-5; yh=10; yl=-10;                            
        break;
        case 1:     // 5 cm along x and 5 cm along y
            xh=25; xl=-5; yh=15; yl=-5;
        break;
        case 2:     // 5 cm along y
            xh=20; xl=-10; yh=15; yl=-5;
        break;
        case 3:    // along -x,y
            xh=5; xl=-25; yh=15; yl=-5;
        break;
        case 4:    // along -x
            xh=-5; xl=-25; yh=10; yl=-10;
        break;
        case 5:    // along -x,-y
            xh=-5; xl=-25; yh=5; yl=-15;
        break;
        case 6:    // along -y
            xh=20; xl=-10; yh=5; yl=-15;              
        break;
        case 7:    // along x,-y
            xh=25; xl=-5; yh=5; yl=-15;
        break;
    }
    
    for (int dx = xl; dx <=xh; dx+=5) {       
        for (int dy = yl; dy <= yh; dy+=5) {  
            std::string key = poseToKey(cmToTile(pose.x + dx),cmToTile(pose.y + dy));
            // LOG_DEBUG("ndx=%d Checking collision (%d,%d) with dx=%d dy=%d key=%s", 
            //     ndx, pose.x + dx, pose.y + dy, dx, dy, key.c_str()); 
            if (obstacles_str.find(key) != obstacles_str.end()) {
                // map_row[cmToTile(pose.x + dx -map_bottom)/2][cmToTile(map_left - 1 - pose.y - dy)] = (char(48+ndx));
                // LOG_DEBUG("ndx=%d Collision detected at (%d,%d) with x=%d y=%d dx=%d dy=%d key=%s", 
                //     ndx, pose.x + dx, pose.y + dy, pose.x, pose.y, dx, dy, key.c_str());
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
   
   
    if (openSet.empty()) {
        if (goalInts.empty()) {
            LOG_DEBUG("All goals have been examined. Set state=ASTAR_COMPLETE");
            state = ASTAR_COMPLETE;
            steps++;
            return true;
        } else {
            LOG_DEBUG("openSet is empty. No more reachable goals. Set state=ASTAR_COMPLETE");
            state = ASTAR_COMPLETE;
            steps++;
            return true;
        }
    }

    // always start with the top of the openSet which is the node with the lowest total cost
    current = openSet.top();
    openSet.pop();
        
    // Check if the current node is close to any goal
    for (auto it = goalInts.begin(); it != goalInts.end();  ) {
        if (cmToTile(current->pose.x - it->x) == 0 && cmToTile(current->pose.y - it->y) == 0) {
            //LOG_DEBUG("Reached goal at (%d, %d).", it->x, it->y);

            // Include the goal in the final path
            //NodeInt* goalNode = new NodeInt{{it->x,it->y}, current->g + 1, 0, current};
            NodeInt* goalNode = allocateNodeInPsram({it->x,it->y}, current->g + 1, 0, current);
            // LOG_DEBUG("Adding goal node (%d, %d) with parent (%d, %d)",
            //     it->x, it->y, current->pose.x, current->pose.y);
            current = goalNode;
            allNodes.push_back(goalNode);
            
            // Draw the goal in the map
            map_row[cmToTile(it->x-map_bottom)/2][cmToTile(map_left-1-it->y)] = 'G';

            LOG_DEBUG("Mark goal (%.02f, %.02f) at index %d as reachable.", 
                (*goals)[it->ndx].x, (*goals)[it->ndx].y, it->ndx);
            //reachableGoals.push_back({it->x / 100.0f, it->y / 100.0f, it->theta, true, it->ndx});
            (*goals)[it->ndx].reachable = true; // Mark the goal as reachable in the original goals vector 

            it = goalInts.erase(it); // Remove the goal from the list

        }else {
            it++; // Move to the next goal
        }   
    }
    // Stop if all goals are reached
    if (goalInts.empty()) {
        LOG_DEBUG("All reachable goals have been found. Set state=ASTAR_COMPLETE");
        state = ASTAR_COMPLETE;
        steps++;
        return true;
    }

    // Explore neighbors
    int  dx[] =     {5   , 5   , 0 ,    -5,  -5,-5    ,      0,  5    };
    int  dy[] =     {0   , 5,    5 ,     5,   0,-5    ,     -5, -5    };
    int  ndx[] =    {0   , 1,    2 ,     3,   4, 5    ,      6,  7    };
    float dtheta[] ={0 , PI/4,PI/2 ,3*PI/4, PI ,5*PI/4, 3*PI/2,7*PI/4 };
    
    for (int i = 0; i < 8; ++i) {
        //PoseInt neighborPose = {current->pose.x + dx[i],current->pose.y + dy[i], dtheta[i]};
        PoseInt neighborPose = {current->pose.x + dx[i],current->pose.y + dy[i]};
        std::string key = poseToKey(cmToTile(neighborPose.x),cmToTile(neighborPose.y));    
        //  LOG_DEBUG("exploring (%d,%d) from (%d,%d) key=%s",
        //      neighborPose.x, neighborPose.y, current->pose.x, current->pose.y, key.c_str());
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
                    
                    // int newH = heuristic(neighborPose, goalInts[0]);
                    //Calculate heuristic relative to the closest goal
                    int newH = INT_MAX;
                    for (const GoalInt& goal : goalInts) {
                        newH = std::min(newH, heuristic(neighborPose, goal));
                    }
                    
                    //NodeInt* neighbor = new NodeInt{neighborPose, newG, newH, current};
                    NodeInt* neighbor = allocateNodeInPsram(neighborPose, newG, newH, current);
                    
                    // LOG_DEBUG("Adding neighbor (%d, %d) with parent (%d, %d)",
                    //     neighborPose.x, neighborPose.y, current->pose.x, current->pose.y);

                    openSet.push(neighbor);
                    allNodes.push_back(neighbor);
                }
            }
        }
    }
    if(allNodes.size() > 1600){
        LOG_ERROR("A* search occupying more than 1600 nodes. HeapSize=%d. Aborting.", esp_get_free_heap_size());
        state = ASTAR_FAILED;
        steps++;
        return true;
    }else{
        //LOG_DEBUG("A* search occupying %d nodes. HeapSize=%d", allNodes.size(), esp_get_free_heap_size());
    }
    steps++;
    return false; // Continue stepping
}

// Get the path from start to the goal with the specified index
std::vector<PoseInt> AStar::getPath(int goalIndex) { 
    std::vector<PoseInt> path;

    // get the goal with the specified index
    Goal goal = goals->at(goalIndex);
    LOG_DEBUG("Getting path to goal %d: (%.2f, %.2f)", goalIndex, goal.x, goal.y);
    // convert Goal to GoalInt
    GoalInt goalInt = {static_cast<int>(roundf(100.0f * goal.x)), static_cast<int>(roundf(100.0f * goal.y)), goal.theta, false, goalIndex};
    LOG_DEBUG("GoalInt: (%d, %d)", goalInt.x, goalInt.y);

    // Find the node corresponding to the specified goalInt
    NodeInt* goalNodeInt = nullptr;
    for (NodeInt* node : allNodes) {
        // LOG_DEBUG("Checking node (%d, %d) against goal (%d, %d)",
        //     node->pose.x, node->pose.y, goalInt.x, goalInt.y);
        // Check if the node's pose matches the goal pose
        if (cmToTile(node->pose.x - goalInt.x) == 0 &&
            cmToTile(node->pose.y - goalInt.y) == 0 ) {
            goalNodeInt = node;
            break;
        }
    }

    if (goalNodeInt == nullptr) {
        LOG_ERROR("Goal node not found in the openSet.");
        return path;
    }
    
    LOG_DEBUG("Goal node found at (%d, %d).", goalNodeInt->pose.x, goalNodeInt->pose.y);
    
    // Backtrack to find the path
    NodeInt* node = goalNodeInt;
    while (node->parent) {
        path.push_back(node->pose);
        node = node->parent;

        // draw the path in the map
        map_row[cmToTile(node->pose.x -map_bottom)/2][cmToTile(map_left - 1 - node->pose.y )] = 
                map_row[cmToTile(node->pose.x -map_bottom)/2][cmToTile(map_left - 1 - node->pose.y )] == 'G' ? 'G' : 'P';
        
    }
    std::reverse(path.begin(), path.end());

    // Remove collinear points
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

    std::vector<PoseInt> finalPath;
    
    // Remove points closer than 0.15 meters to midWheelsPose
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

std::vector<Goal> AStar::getReachableGoals() {
    std::vector<Goal> reachableGoals;
    // Iterate through the goals and check if they are reachable
    for(Goal& goal : *goals) {
        // Check if the goal is reachable
        if (goal.reachable) {
            reachableGoals.push_back(goal);
        }
    }
    return reachableGoals;
}

// get the farthest reachable goal
int AStar::getFarthestReachableGoal(){
    
    // for(int i=0; i <goals->size();i++){
    //     Goal farthestGoal = (*goals)[i];
    //     LOG_DEBUG("Checking goal %d: (%f, %f) reachable=%d", i, farthestGoal.x, farthestGoal.y, farthestGoal.reachable); 
    // }
    

    for(int i=0; i <goals->size();i++){
        Goal &farthestGoal = (*goals)[i];
        // LOG_DEBUG("Checking goal %d: (%f, %f) reachable=%d", i, farthestGoal.x, farthestGoal.y, farthestGoal.reachable); 
        if(farthestGoal.reachable){
            LOG_DEBUG("%2d Goal %f,%f is reachable", i, farthestGoal.x, farthestGoal.y);
            return i;
        }
    }
    // LOG_DEBUG("No reachable goals found");
    return -1; // Return -1 if no reachable goals are found
}


void AStar::drawMap(){
    // Draw the map
    LOG_INFO(" y   +9+8+7+6+5+4+3+2+1+0-1-2-3-4-5-6-7-8-9-0");
    for(int x = 19; x >= 0; x--) {
        LOG_INFO("%3d %s",x-10, map_row[x].c_str());
    }
}

// Convert tile coordinates (x,y) to a string key
std::string AStar::poseToKey(int x, int y) {
    return std::to_string(x) + "_" + std::to_string(y);
}

// Heuristic function for A* search
int AStar::heuristic(const PoseInt& a, const GoalInt& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}
