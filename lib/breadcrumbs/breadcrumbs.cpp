
#include <vector>
#include <cmath>
#include <Arduino.h>
#include "breadcrumbs.h"

// Define a Pose structure
// struct Pose {
//     float x;
//     float y;
//     float theta;
// };

// Constructor
Breadcrumbs::Breadcrumbs() {}

// Destructor
Breadcrumbs::~Breadcrumbs() {}

    
// Helper function to calculate the orientation of three points
int Breadcrumbs::orientation(const Pose& p, const Pose& q, const Pose& r) {
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (fabs(val) < 1e-6) return 0; // Collinear
    return (val > 0) ? 1 : 2;       // Clockwise or Counterclockwise
}

// Helper function to check if two segments intersect and calculate the intersection point
bool Breadcrumbs::segmentsIntersect(const Goal& a0, const Goal& a1, const Goal& b0, const Goal& b1, Goal& intersection) {
    float denominator = (a0.x - a1.x) * (b0.y - b1.y) - (a0.y - a1.y) * (b0.x - b1.x);

    // If denominator is zero, the lines are parallel or collinear
    if (fabs(denominator) < 1e-6) return false;

    float t = ((a0.x - b0.x) * (b0.y - b1.y) - (a0.y - b0.y) * (b0.x - b1.x)) / denominator;
    float u = -((a0.x - a1.x) * (a0.y - b0.y) - (a0.y - a1.y) * (a0.x - b0.x)) / denominator;

    // Check if the intersection point lies on both segments
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        intersection.x = a0.x + t * (a1.x - a0.x);
        intersection.y = a0.y + t * (a1.y - a0.y);
        intersection.theta = 0.0; // Optional: Set theta to 0 for simplicity
        return true;
    }

    return false;
}

// Add a breadcrumb
void Breadcrumbs::AddBreadcrumb(const Pose pose) {
    breadcrumbs.push_back({pose.x, pose.y, pose.theta, false, 0});
}

// Print breadcrumbs
void Breadcrumbs::PrintBreadcrumbs(const char* message) {
    Serial.println(message);
    for (const auto& pose : breadcrumbs) {
        Serial.printf("Pose: x=%.2f, y=%.2f, theta=%.2f\n", pose.x, pose.y, pose.theta);
    }
}

// Clean path loops in breadcrumbs
void Breadcrumbs::Clean() {
    for (size_t i = 0; i < breadcrumbs.size() - 1; ++i) {
        for (size_t j = i + 2; j < breadcrumbs.size() - 1; ++j) {
            Goal intersection;
            if (segmentsIntersect(breadcrumbs[i], breadcrumbs[i + 1], breadcrumbs[j], breadcrumbs[j + 1], intersection)) {
                // Remove breadcrumbs between the intersecting segments
                breadcrumbs.erase(breadcrumbs.begin() + i + 1, breadcrumbs.begin() + j + 1);

                // Insert the intersection point as a new breadcrumb
                breadcrumbs.insert(breadcrumbs.begin() + i + 1, intersection);

                Serial.printf("Intersection detected at (%.2f, %.2f). Breadcrumbs now have %d elements.\n",
                                intersection.x, intersection.y, breadcrumbs.size());

                // Restart the loop to recheck from the beginning
                i = -1; // Reset outer loop
                break;
            }
        }
    }
}

