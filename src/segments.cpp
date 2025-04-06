// segments.cpp
#include "segments.h"
#include "cartesian.h"
#include "debuglog.h"

void Segments::begin(){

    buildPattern25();
}

float Segments::calclength(float D1, float D2, int pos1, int pos2, float angularResolution) {
    float angleDiff = static_cast<float>(std::abs(pos2 - pos1)) * angularResolution;
    return std::sqrt(D1 * D1 + D2 * D2 - 2 * D1 * D2 * std::cos(angleDiff));
}

float Segments::calculateSegmentLength(const std::vector<float>& ranges, int startIndex, int endIndex, float angularResolution) {
    if (startIndex == endIndex || startIndex < 0 || endIndex >= ranges.size())
        return 0.0f;
    if(endIndex > startIndex)
        return calclength(ranges[startIndex], ranges[endIndex], startIndex, endIndex, angularResolution);
    else
        return calclength(ranges[startIndex], ranges[endIndex], startIndex, endIndex + ranges.size(), angularResolution);
}

float Segments::calculateSegmentSlope(const std::vector<float>& ranges, int startIndex, int endIndex, float angularResolution) {
    int size = ranges.size();
    if (size == 0 || startIndex < 0 || endIndex < 0 || startIndex >= size || endIndex >= size || startIndex == endIndex)
        return 0.0f;

    // Calculate the distance, handling circular buffer wrap-around
    int distance = (startIndex <= endIndex) ? (endIndex - startIndex) : (size - startIndex + endIndex);

    // Avoid division by zero (though the earlier check ensures distance > 0)
    if (distance == 0)
        return 0.0f;

    // Calculate the slope
    return (ranges[endIndex] - ranges[startIndex]) / distance;
}

float Segments::calculateLineVariance(const std::vector<float>& ranges, int startIndex, int endIndex, float angularResolution) {
    int size = ranges.size();
    if (size == 0 || startIndex < 0 || endIndex < 0 || startIndex >= size || endIndex >= size)
        return 0.0f;

    int count = (startIndex <= endIndex) ? (endIndex - startIndex + 1) : (size - startIndex + endIndex + 1);
    if (count <= 0)
        return 0.0f;

    // Calculate the mean
    float mean = 0.0f;
    if (startIndex <= endIndex) {
        mean = std::accumulate(ranges.begin() + startIndex, ranges.begin() + endIndex + 1, 0.0f);
    } else {
        mean = std::accumulate(ranges.begin() + startIndex, ranges.end(), 0.0f);
        mean += std::accumulate(ranges.begin(), ranges.begin() + endIndex + 1, 0.0f);
    }
    mean /= count;

    // Calculate the variance
    float variance = 0.0f;
    if (startIndex <= endIndex) {
        variance = std::accumulate(ranges.begin() + startIndex, ranges.begin() + endIndex + 1, 0.0f,
            [mean](float acc, float val) { return acc + (val - mean) * (val - mean); });
    } else {
        variance = std::accumulate(ranges.begin() + startIndex, ranges.end(), 0.0f,
            [mean](float acc, float val) { return acc + (val - mean) * (val - mean); });
        variance += std::accumulate(ranges.begin(), ranges.begin() + endIndex + 1, 0.0f,
            [mean](float acc, float val) { return acc + (val - mean) * (val - mean); });
    }
    variance /= count;

    return variance;
}

// Find all the segments in a full scan
std::vector<Segments::LineSegment> Segments::findSegments(const std::vector<float>& ranges, float angularResolution) {
    int n = ranges.size();
    std::vector<LineSegment> segments;

    int startIndex = 0;

    int i = 1;
    while(i < n * 5 / 4) { // ranges is scanned from 0 to 399 and then from 0 to 99
                           // but it will stop at the end of the segment across 0
        int currIdx = i % n;         // Current index with wraparound
        int prevIdx = (currIdx - 1 + n) % n;
        int nextIdx = (currIdx + 1) % n;

        //printf("i=%d n=%d ranges[%d]=%.3f\r\n",i,n, currIdx, ranges[currIdx]);

        if(ranges[currIdx] == 0.0){
            startIndex = nextIdx;
            i++;
            continue;
        }
        float angle0 = (prevIdx) * angularResolution;
        float angle1 = currIdx * angularResolution;
        float angle2 = (nextIdx + 1) * angularResolution;

        // Compute X and Y coordinates for three consecutive points
        float x0 = ranges[prevIdx] * std::cos(angle0);
        float y0 = ranges[prevIdx] * std::sin(angle0);

        float x1 = ranges[currIdx] * std::cos(angle1);
        float y1 = ranges[currIdx] * std::sin(angle1);

        float px2 = 2 * x1 - x0;
        float py2 = 2 * y1 - y0;
        float dist_tol = 0.04;

        float x2 = ranges[nextIdx] * std::cos(angle2);
        float y2 = ranges[nextIdx] * std::sin(angle2);

        float dist2 = sqrt((x2-px2)*(x2-px2) + (y2-py2)*(y2-py2));

        // printf("segment [%d]=%.3f [%d]=%.3f angle0=%.3f angle1=%.3f angle2=%.3f dist2=%.3f\r\n", 
        //     startIndex,ranges[startIndex],currIdx, ranges[currIdx],angle0, angle1, angle2,dist2);
        
        // Check distance between x2,y2 and the point projected from x0,y0 x1,y1
        if (dist2 > dist_tol ){
            // End current segment
            if (i - startIndex > 1) {
                float segmentVariance = calculateLineVariance(ranges, startIndex, currIdx, angularResolution);
                float segmentLength = calculateSegmentLength(ranges, startIndex, currIdx, angularResolution);
                float segmentSlope = calculateSegmentSlope(ranges, startIndex, currIdx, angularResolution);

                segments.push_back({startIndex, currIdx, segmentVariance, segmentLength, segmentSlope});
            }
            // check if this segment is across the boundary n-1 0
            if(i > n - 1){
                //printf("this last segment from %d to %d is across the boundary\r\n", startIndex, currIdx);
                // let's check if the first segment, from 0 to currIdx, was recorded at the beginning
                if(segments[0].startIndex == 0 && segments[0].endIndex == currIdx)
                    // erase it!
                    // printf("the segment from %d to %d is erased because it has been recorded from %d to %d\r\n",
                    //     0, currIdx, startIndex, currIdx);
                    segments.erase(segments.begin());


                break;
            }    
            startIndex = currIdx; // Start a new segment
        }
        i++;
    }
    return segments;
}

// Function to build a pattern 25 points long
// 250 250 ... 250 100 ... 100 250 ... 250 100 ... 100 250 ... 250 
void Segments::buildPattern25() {

    float high = 250.0;
    float low = 150.0;

    for (int i=0;i<25;i++) {
        if(i>=5 && i< 10 || i >= 15 && i < 20)
            pattern.push_back(low);
        else    
            pattern.push_back(high);
    }
}


// Precompute MAX_ENERGY_5 / 5.0
constexpr float MAX_ENERGY_5_PER_ELEMENT = ((255 - 175) * (255 - 175) * 3 + (100 - 175) * (100 - 175) * 2) / 5.0f;

// Get the energy in percentage of its maximum theoretical value 
float Segments::energyPerc(const std::vector<float>& data) {
    float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    float sumSq = 0.0;

    for (float value : data) {
        sumSq += (value - mean) * (value - mean);
    }
    float energy_val = sumSq ;
    float energy_max = MAX_ENERGY_5_PER_ELEMENT * data.size() ; 

    return energy_val / energy_max;
}

float Segments::calculateMeans(const std::vector<float>& data, float& u, float& uh, float& ul) {
    if (data.empty()) {
        u = uh = ul = 0.0f;
        return 0.0;
    }

    // Calculate 5 means (un0 ...un4)
    float un[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    int num_samples[5] = {0,0,0,0,0};
    int nn = data.size()/5;
    for (int i=0; i< data.size();i++) {
        un[i/nn] += data[i];
        num_samples[i/nn] ++;
    }
    for (int i=0; i< nn;i++) {
        if(num_samples[i] > 0)
            un[i] /= (float) num_samples[i];
    }
    // LOG_DEBUG("%.3f %.3f %.3f %.3f %.3f", un[0],un[1],un[2],un[3],un[4] );
    
    uh = 1000.0 * (un[0] + un[2] + un[4]) / 3.0;
    ul = 1000.0 * (un[1] + un[3]) / 2.0;
    u  = (uh + ul) / 2;
    // calculate points
    // if(uh < 200.0)
    //     return 0.0;
    if(1000.0 * un[0] < u || 1000.0 * un[1] > u || 1000.0 * un[2] < u || 1000.0 * un[3] > u || 1000.0 * un[4] < u)
        return 0.0;
    if(uh - u < 10.0)
        return 0.0;
    if(u-ul < 10)
        return 0.0;
    
    float points = uh - ul;
    
    return points;
}

// Standardize the data (zero mean, unit variance)
std::vector<float> Segments::standardizeData(const std::vector<float>& data) {
    float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    float sumSq = 0.0;

    for (float value : data) {
        sumSq += (value - mean) * (value - mean);
    }

    float stdDev = std::sqrt(sumSq / data.size());
    
    std::vector<float> standardizedData;
    for (float value : data) {
        standardizedData.push_back((value - mean) / stdDev);
    }

    return standardizedData;
}


// Compute normalized cross-correlation 
// It returns a vector of correlation values (one for each shift)
std::vector<float> Segments::crossCorrelationNormalized(const std::vector<float>& data, const std::vector<float>& pattern) {
    int dataSize = data.size();
    int patternSize = pattern.size();
    int maxSize = std::max(dataSize, patternSize);
    int minSize = std::min(dataSize, patternSize);
    std::vector<float> correlation(maxSize - minSize + 1, 0.0);

    // Compute pattern norm
    float patternNorm = std::sqrt(std::inner_product(pattern.begin(), pattern.end(), pattern.begin(), 0.0));

    // printf("correlate pattern with data\r\n");
    for (int i = 0; i <= maxSize - minSize; ++i) {
        float sum = 0.0;
        float dataNorm = 0.0;

        for (int j = 0; j < minSize; j++) {
            //printf("data[%d]=%.3f * pattern[%d]=%.3f\r\n", i+j,data[i+j], j, pattern[j]);
            if (dataSize >= patternSize) {
                sum += data[i + j] * pattern[j];
                dataNorm += data[i + j] * data[i + j];
            } else {
                sum += pattern[i + j] * data[j];
                dataNorm += pattern[i + j] * pattern[i + j];
            }
        }
        // printf("end of correlate: dataNorm=%.3f patternNorm=%.3f sum=%.3f\r\n",dataNorm, patternNorm, sum);

        dataNorm = std::sqrt(dataNorm);
        correlation[i] = sum / (dataNorm * patternNorm);  // Normalized between -1 and 1
        if(correlation[i] < 0)
            correlation[i] = 0;
        //printf("patternLength=%d sampledIntensitieslength=%d correlation[%d]=%f\r\n", pattern.size(), data.size(),i,  correlation[i]);
    }
    return correlation;
}

// struct Point {
//     float x, y, intensity;
// };

// Compute segment length and resample intensities at even spacing
std::vector<float> Segments::resampleIntensities(const std::vector<Point>& points, int numSamples) {
    std::vector<float> sampledIntensities;
    
    // Compute cumulative segment length
    std::vector<float> distances = {0.0f};  
    float totalLength = 0.0f;
    
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        totalLength += dist;
        distances.push_back(totalLength);
    }

    // Resample at uniform distance intervals
    for (int i = 0; i < numSamples; ++i) {
        float targetDist = (i * totalLength) / (numSamples - 1);

        // Find the segment containing targetDist
        size_t j = 1;
        while(j < points.size() && distances[j] < targetDist) {
            j++;
        }

        if (j >= points.size()) {
            j = points.size() - 1;  // Edge case: make sure we don't exceed bounds
        }

        // Linear interpolation
        float d1 = distances[j - 1];
        float d2 = distances[j];
        float alpha = (d2 > d1) ? (targetDist - d1) / (d2 - d1) : 0.0f; // Avoid division by zero

        // Linear interpolation of intensity
        float I1 = points[j - 1].intensity;
        float I2 = points[j].intensity;
        float interpolatedIntensity = I1 + alpha * (I2 - I1);
        
        sampledIntensities.push_back(interpolatedIntensity);
    }
    return sampledIntensities;
};

// Convert polar to Cartesian coordinates
void Segments::convertToCartesian(
                        std::vector<Point>& points, 
                        const std::vector<float>& ranges, 
                        const std::vector<float>& angles, 
                        const std::vector<float>& intensities) {

    for (size_t i = 0; i < ranges.size(); ++i) {
        float x = ranges[i] * cos(angles[i]);
        float y = ranges[i] * sin(angles[i]);
        points.push_back({x, y, intensities[i]});
    }
}

std::vector<float> Segments::GetSegmentIntensities(Segments::LineSegment segment, std::vector<float> &qualities){

    std::vector<float> IntensitiesToBePrinted;
    if(segment.startIndex < segment.endIndex)
        for(int i=segment.startIndex; i <= segment.endIndex; i++){
            IntensitiesToBePrinted.push_back(qualities[i]);
        }
    else
        for(int i=segment.startIndex; i <= qualities.size() + segment.endIndex; i++){
            IntensitiesToBePrinted.push_back(qualities[i%qualities.size()]);
        }
    return IntensitiesToBePrinted;
}

// get the char values (UTF-8 encoded) to build the chart
std::string Segments::GetChartValues(std::vector<float> &IntensitiesToBePrinted) {

    std::string result;
    const char *blocks[] = {"▁", "▂", "▃", "▄", "▅", "▆", "▇", "█"};
    
    for (float val : IntensitiesToBePrinted) {
        int level = std::round((val / 255.0) * 7);  // Normalize to 0-7
        if(level < 0) level = 0;
        if(level > 7) level = 7;    
        result += blocks[level];  // Append block
    }
    
    // Return the dynamically allocated buffer
    return result;
}

// Function to detect if the pattern is present in this segment
bool Segments::detectPattern(LineSegment & segment, std::vector<float> & distances, std::vector<float> & qualities) {
         
    std::vector<float> segmentIntens;
    std::vector<float> segmentRanges;
    std::vector<float> segmentAngles;
    float angle_increment = 2 * PI / qualities.size();
           
    std::vector<float> intensitiesVector = GetSegmentIntensities(segment,qualities);
    std::string chartValuesBefore = GetChartValues(intensitiesVector);
    LOG_INFO("segment before conditioning: start=%d end=%d %s",
        segment.startIndex,segment.endIndex, chartValuesBefore.c_str());
    
    if(segment.startIndex < segment.endIndex)
        for(int i=segment.startIndex; i <=segment.endIndex; i++){
            float q = qualities[i];
            float angle = i * angle_increment;
            float range = distances[i];
            segmentIntens.push_back(q);
            segmentAngles.push_back(angle);
            segmentRanges.push_back(range);
        }
    else
        for(int i=segment.startIndex; i <=segment.endIndex + qualities.size(); i++){
            float q = qualities[i%(qualities.size())];
            float angle = (i%(qualities.size())) * angle_increment;
            float range = distances[i%(qualities.size())];
            segmentIntens.push_back(q);
            segmentAngles.push_back(angle);
            segmentRanges.push_back(range);
        }
    
    // skip the extremities if the segment is longer or equal 7
    if(segmentIntens.size() >=7){
        segmentIntens.erase(segmentIntens.begin());
        segmentIntens.erase(segmentIntens.end()-1);
        segmentAngles.erase(segmentAngles.begin());
        segmentAngles.erase(segmentAngles.end()-1);
        segmentRanges.erase(segmentRanges.begin());
        segmentRanges.erase(segmentRanges.end()-1);
        // recalc length    
        segment.length = calculateSegmentLength(segmentRanges,0, segmentRanges.size()-1, 2.0 * PI / (float)distances.size());
    }
    
    std::vector<Point> points;
    convertToCartesian(points, segmentRanges, segmentAngles, segmentIntens);
    // resample intensities to 25    
    std::vector<float> sampledIntensities = resampleIntensities(points, 25);
    
    // Standardize the LIDAR data
    std::vector<float> standardizedsamplesIntensities = standardizeData(sampledIntensities);
    //std::vector<float> standardizedsamplesIntensities = sampledIntensities;
    // Standardize the pattern
    std::vector<float> standardizedPattern = standardizeData(pattern);
    //std::vector<float> standardizedPattern = pattern;
    
    // copy values to be printed
    std::vector<float> values2print;
    for(float value : standardizedsamplesIntensities){
        values2print.push_back((value + 4) * 32);
    }
    // find their minimum and maximum value
    auto [minIt, maxIt] = std::minmax_element(values2print.begin(), values2print.end());
    float minVal = *minIt;
    float maxVal = *maxIt;
    // condition the values to stay between 0 and 255
    // but avoid division by zero if all values are the same
    if (minVal != maxVal) {
        for (float &val : values2print) {
            val = ((val - minVal) / (maxVal - minVal)) * 255.0f;
        }
    } else {
        // If all values are the same, set them all to 127 (mid-gray)
        std::fill(values2print.begin(), values2print.end(), 127.0f);
    }

    std::string chartValuesAfter = GetChartValues(values2print);
    
    std::vector<float> correlations = crossCorrelationNormalized(standardizedsamplesIntensities, standardizedPattern);
    segment.maxCorrelation = correlations[0];
    segment.energyPerc = energyPerc(segmentIntens);
    segment.patternLength = pattern.size();;
    segment.segmentLength = segmentIntens.size();
    
    // calculate u, uh and ul
    float u,uh,ul;
    //float pts = calculateMeans(segmentIntens, u, uh, ul);
    float pts = calculateMeans(standardizedsamplesIntensities, u, uh, ul);
    LOG_DEBUG("points=%.3f u=%.3f uh=%.3f ul=%.3f", pts,u,uh,ul);
    segment.u = u;
    segment.uh = uh;
    segment.ul = ul;
    segment.pts = pts;

    LOG_INFO("segment after conditioning: start=%d end=%d corr=%.2f pts=%.0f %s",
        segment.startIndex,segment.endIndex,segment.maxCorrelation,segment.pts,  chartValuesAfter.c_str());

    return true;
}

// Function to calculate correlation between the pattern and each segment
// with a minimal length
bool Segments::detectPatternAll(std::vector<LineSegment> & segments, std::vector<float> & ranges,std::vector<float> & qualities, int minimumPoints, float minLength, float maxLength) {
                   
        // Output results
        printf("-------------------------------------------------------\r\n");
        printf("start detectPatternAll for %d segments...\r\n", segments.size());
        for (LineSegment & segment : segments) {
            // printf("startIndex=%d endIndex%d Length=%.3f\r\n", 
            //     segment.startIndex, segment.endIndex, segment.length);
            
            if(
                ( segment.endIndex > segment.startIndex && (segment.endIndex - segment.startIndex > minimumPoints) ||
                  segment.endIndex < segment.startIndex && (qualities.size() + segment.endIndex - segment.startIndex > minimumPoints) 
                ) &&       
                segment.length > minLength && segment.length < maxLength
                 ){
                bool ret = detectPattern(segment, ranges , qualities);
            }
        }
        return false;
}
