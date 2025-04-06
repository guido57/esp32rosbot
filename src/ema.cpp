#include <math.h>

class EMAFilter {
private:
    double alpha;  // Smoothing factor (0.0 - 1.0)
    double filtered_value;
    bool initialized;
    double filtered_cos;
    double filtered_sin;
   
public:
    EMAFilter(double alpha) : alpha(alpha), 
        filtered_value(0.0), filtered_cos(0.0),filtered_sin(0.0), initialized(false) {}

    double update(double new_value) {
        if (!initialized) {
            filtered_value = new_value;
            initialized = true;
        } else {
            filtered_value = (1.0 - alpha) * new_value + alpha * filtered_value;
        }
        return filtered_value;
    }

    double updateTheta(double new_angle) {
        if (!initialized) {
            filtered_cos = cos(new_angle);
            filtered_sin = sin(new_angle);
            initialized = true;
        } else {
            // Apply EMA filtering on cosine and sine components separately
            filtered_cos = (1 - alpha) * cos(new_angle) + alpha * filtered_cos;
            filtered_sin = (1 - alpha) * sin(new_angle) + alpha * filtered_sin;
        }
        
        // Reconstruct the angle using atan2 to handle wrap-around correctly
        return atan2(filtered_sin, filtered_cos);
    }
};
