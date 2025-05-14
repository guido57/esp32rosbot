#include <Arduino.h>
#include <vector>
#include <unity.h>

#include <cartesian.h>
#include "astar.h"

#define LOG_INFO(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define LOG_DEBUG(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...) printf(format "\r\n", ##__VA_ARGS__)
#define LOG_WARN(format, ...) printf(format "\r\n", ##__VA_ARGS__)

// Global variables
// ROS Shared flags
bool ROS_connected = false;

std::vector<float> obstacles; // Simulated obstacle data
bool initialized = false;
AStar astar;
Pose startPose = {-0.6f, -0.2f, 0.0f};
std::vector<Goal> goals = {
    { 0.0f, 0.0f, 0.0f,false,0},
    {-0.2f, 0.0f, 0.0f,false,1},
    {-0.2f, 0.2f, 0.0f,false,2},
    {-0.2f, 0.4f, 0.0f,false,3},
    {-0.4f, 0.4f, 0.0f,false,4},
    {-0.6f, 0.4f, 0.0f,false,5},
    {-0.6f, 0.2f, 0.0f,false,6},
    {-0.6f, 0.0f, 0.0f,false,7},
    {-0.6f,-0.2f, 0.0f,false,8},
};

void test_initial_state() {

    printf("Starting A* Pathfinding Testing\r\n");
    // Simulate some obstacles. Don't forget they are lidar ranges i.e. they are measured from the lidar Pose to the obstacle
    //obstacles = {1.0, 1.0, 0.40, 1.0, 1.0, 1.0, 1.0, 1.0};
    for(int i = 0; i < 360; ++i) {
        obstacles.push_back(2.0f); // Simulated lidar range data
    }
    for(int i = 0; i <= 180; ++i) {
      obstacles[i] = 0.4; // Simulated lidar range data
    }
    // for(int i = 0; i <= 359; ++i) {
    //   obstacles[i] = 0.4; // Simulated lidar range data
    // }

    LOG_INFO("Init A*... heapsize=%d", esp_get_free_heap_size( ));
      
    // Initialize A* with the start pose, goals, and obstacles
    astar.initialize(startPose, &goals, obstacles, 2.0f, 2.0f); // Map width and height in meters
    initialized = true;

    TEST_ASSERT_EQUAL( AStar::ASTAR_STARTED, astar.state );
}


bool start_measure = false;
unsigned long startTime = 0;
int steps = 0;


void test_loop() {

  TEST_ASSERT_TRUE( initialized);
    
  if (!initialized) {
      return;
  }

  if(!start_measure) {
      start_measure = true;
      startTime = millis();
      LOG_INFO("Starting A* search... heapsize=%d", esp_get_free_heap_size( ));
      steps = 0;
  }
  
  // Perform A* steps until all reachable goals are found
  while (astar.step() == false) {
      // Continue stepping
      steps++;
  } 
  
  // A* search is complete
  LOG_INFO("A* search completed in %lu ms in %d steps", millis() - startTime, steps);
  LOG_INFO("State: %d", astar.state);

  startTime = millis();
  // Getting the reachable goals
  std::vector<Goal> reachableGoals = astar.getReachableGoals();
  //LOG_INFO("Reachable Goals found in %lu ms", millis() - startTime);
  
  // Print goals
  LOG_INFO("Goals:");
  for (const Goal goal : goals) {
      LOG_INFO("Goal: (%.3f, %.3f, %.3f) reachable=%d", goal.x, goal.y, goal.theta, goal.reachable);
  }
  
  // Stop further execution
  initialized = false;
  
  LOG_INFO("finished! AStar state=%d heapsize=%d", astar.state, esp_get_free_heap_size());

    // get the farthest reachable goal
  int frg = astar.getFarthestReachableGoal();
  TEST_ASSERT_TRUE( frg == 3);
  TEST_ASSERT_TRUE(goals[frg].reachable == true);
  TEST_ASSERT_TRUE(goals[frg].x == -0.20f);
  TEST_ASSERT_TRUE(goals[frg].y ==  0.40f);
  
  if(frg >= 0){
    LOG_INFO("The farthest reachable goal is goals[%d]=(%0.2f,%0.2f)", 
      frg, goals[frg].x, goals[frg].y);
      std::vector<PoseInt> path = astar.getPath(frg);
      for(const PoseInt pose : path) {
          LOG_INFO("Path Point: (%d, %d, %.3f)", pose.x, pose.y, pose.theta);
      } 
  }else{
    LOG_INFO("No reachable goals found");
  }

  // Draw the map for visualization
  astar.drawMap();

}

void setup() {
    delay(1000);
    UNITY_BEGIN();
    RUN_TEST(test_initial_state);
    RUN_TEST(test_loop);

    UNITY_END();
}

void loop() {}
