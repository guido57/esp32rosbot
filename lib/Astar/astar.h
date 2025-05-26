// astar.h
#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_set>
#include <string>
#include <cmath>
#include "cartesian.h"
#include "PsramAllocator.h"

struct PoseInt {
    int x;       // centimeters 
    int y;       // centimeters
//    float theta; // Angle in radians
};

struct GoalInt {
    int x;       // centimeters   
    int y;       // centimeters
    float theta; // Angle in radians
    bool reachable;
    int ndx;
};

struct NodeInt {
    PoseInt pose;
    int g; // Cost from start to this node
    int h; // Heuristic cost to the goal
    NodeInt* parent;
    int getF() const { return g + h; } // Calculate f on the fly
};

struct CompareNodeIntPointers {
    bool operator()(const NodeInt* a, const NodeInt* b) const{
//        return a->f > b->f;
        return a->getF() > b->getF();
    }
};

class AStar {
public:
    AStar();
    void initialize(const Pose & start, std::vector<Goal> * goals,  const std::vector<float, PsramAllocator<float>>& ranges,float mapWidth, float mapHeight);
    bool step();
    void drawMap();
    std::vector<PoseInt> getPath(int goalIndex = 0);
    std::vector<Goal> getReachableGoals();
    int getFarthestReachableGoal();

    enum {ASTAR_IDLE, ASTAR_INIT,ASTAR_STARTED,ASTAR_COMPLETE, ASTAR_FAILED} state;
    std::unordered_set<std::string> obstacles_str;
    std::priority_queue<NodeInt*, std::vector<NodeInt*>, CompareNodeIntPointers> openSet;
    std::vector<NodeInt*> allNodes;
    int consecutive_no_progress;
    std::unordered_set<std::string> visitedSet;
    int steps;
    
private:
    std::vector<Goal> * goals;
    int map_left, map_right, map_top, map_bottom;
    std::vector<GoalInt> goalInts;
    //std::vector<Goal> reachableGoals;
    PoseInt startInt;
    NodeInt* current;
    NodeInt* allocateNodeInPsram(const PoseInt& pose, int g, int h, NodeInt* parent) ;
    std::string poseToKey(int x, int y);
    int heuristic(const PoseInt& a, const GoalInt& b);
    // bool isValidMove(PoseInt pose) ;
    bool isValidMove(PoseInt pose, int ndx) ;
    std::string map_row[21];
    
};

#endif // ASTAR_H
