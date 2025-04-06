// astar.h
#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <unordered_set>
#include <string>
#include <cmath>
#include "cartesian.h"

struct PoseInt {
    int x;
    int y;
    float theta; // Angle in radians
};

struct NodeInt {
    PoseInt pose;
    int g, h, f;
    NodeInt* parent;
};

struct CompareNodeIntPointers {
    bool operator()(const NodeInt* a, const NodeInt* b) const{
        return a->f > b->f;
    }
};

class AStar {
public:
    AStar();
    void initialize(const Pose& start, const Pose& goal,  const std::vector<float>& ranges,float mapWidth, float mapHeight);
    bool step();
    void drawMap();
    std::vector<PoseInt> getPath();
    enum {ASTAR_IDLE, ASTAR_INIT,ASTAR_STARTED,ASTAR_COMPLETE, ASTAR_FAILED} state;
    std::unordered_set<std::string> obstacles_str;
    std::priority_queue<NodeInt*, std::vector<NodeInt*>, CompareNodeIntPointers> openSet;
    std::vector<NodeInt*> allNodes;
    int consecutive_no_progress;
    std::unordered_set<std::string> visitedSet;
    
private:
    int map_left, map_right, map_top, map_bottom;
    PoseInt goalInt, startInt;
    NodeInt* current;
    std::string poseToKey(int x, int y);
    int heuristic(const PoseInt& a, const PoseInt& b);
    // bool isValidMove(PoseInt pose) ;
    bool isValidMove(PoseInt pose, int ndx) ;
    std::string map_row[21];
    
};

#endif // ASTAR_H
