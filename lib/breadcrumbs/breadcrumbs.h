#ifndef BREADCRUMBS_H
#define BREADCRUMBS_H

#include <vector>
#include <cmath>
#include <Arduino.h>
#include "cartesian.h"

class Breadcrumbs {

private:

    // Helper function to calculate the orientation of three points
    int orientation(const Pose& p, const Pose& q, const Pose& r);

    // Helper function to check if two segments intersect and calculate the intersection point
    bool segmentsIntersect(const Goal& a0, const Goal& a1, const Goal& b0, const Goal& b1, Goal& intersection);

public:
    
    // Constructor
    Breadcrumbs();

    // Destructor
    ~Breadcrumbs();

    // The list of breadcrumbs
    //std::vector<Pose> breadcrumbs;
    std::vector<Goal> breadcrumbs;

    // Add a breadcrumb
    void AddBreadcrumb(const Pose pose);

    // Print breadcrumbs
    void PrintBreadcrumbs(const char* message);

    // Clean loops in breadcrumbs
    void Clean();
};

#endif // BREADCRUMBS_H