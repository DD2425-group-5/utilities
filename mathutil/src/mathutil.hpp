#include <ros/console.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <utility>

namespace MathUtil {
    double vectorSum(std::vector<double> v);
    int vectorSum(std::vector<int> v);
    float vectorSum(std::vector<float> v);

    void lineEquation(float x1, float y1, float x2, float y2, float& a, float& b, float& c);
    bool approxEqual(float a, float b, float epsilon);

    std::pair<float, float> rotateAroundOrigin(float x, float y, float angle);
    
}
