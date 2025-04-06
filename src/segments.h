// segments.h
#ifndef SEGMENTS_H
#define SEGMENTS_H

#include <Arduino.h>
#include <vector>
#include <numeric>
#include <cmath>

class Segments {
public:
    struct LineSegment {
        int startIndex;
        int endIndex;
        float variance;
        float length;
        float slope;
        float maxCorrelation;
        float energyPerc;
        int patternLength;
        int segmentLength;
        float u;
        float uh;
        float ul;
        float pts;
    };

    std::vector<float> pattern;

    void begin();
    float calclength(float D1, float D2, int pos1, int pos2, float angularResolution);
    float calculateMeans(const std::vector<float>& data, float& u, float& uh, float& ul);
    float calculateSegmentLength(const std::vector<float>& ranges, int startIndex, int endIndex, float angularResolution);
    float calculateSegmentSlope(const std::vector<float>& ranges, int startIndex, int endIndex, float angularResolution);
    float calculateLineVariance(const std::vector<float>& ranges, int startIndex, int endIndex, float angularResolution);
    std::vector<float> standardizeData(const std::vector<float>& data) ;
    std::vector<LineSegment> findSegments(const std::vector<float>& ranges, float angularResolution);
    void buildPattern25();
    float energyPerc(const std::vector<float>& data);
    std::vector<float> crossCorrelationNormalized(const std::vector<float>& data, const std::vector<float>& pattern);

    struct Point {
        float x, y, intensity;
    };

    // Compute segment length and resample intensities at even spacing
    std::vector<float> resampleIntensities(const std::vector<Point>& points, int numSamples) ;
    void convertToCartesian(std::vector<Point>& points, 
                            const std::vector<float>& ranges, 
                            const std::vector<float>& angles, 
                            const std::vector<float>& intensities) ;

    std::vector<float> GetSegmentIntensities(Segments::LineSegment segment, std::vector<float> &qualities);
    std::string GetChartValues(std::vector<float> &IntensitiesToBePrinted);

    bool detectPattern(LineSegment & segment, std::vector<float> & ranges, std::vector<float> & qualities);
    bool detectPatternAll(std::vector<LineSegment> & segments, std::vector<float> & ranges, std::vector<float> & qualities, int minimalLength = 8, float minLength = 0.2, float maxLength = 0.3);
};

#endif // SEGMENTS_H
