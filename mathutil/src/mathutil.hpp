#include <ros/console.h>
#include <iostream>
#include <vector>
#include <exception>
#include <cmath>
#include <utility>

namespace MathUtil {
    
    
    struct Point {
        Point(){}
        Point(float _x, float _y) : x(_x), y(_y){}
        float x;
        float y;
    };
    
    struct Line {
        Line(){}
        Line(Point _begin, Point _end) : begin(_begin), end(_end){}
        Line(float x1, float y1, float x2, float y2) : begin(Point(x1, y1)), end(Point(x2, y2)){}
        Point begin;
        Point end;
    };
    
    struct Bounds2D
    {
        Bounds2D(){}
        Bounds2D(Point _min, Point _max) : min(_min), max(_max){}
        Bounds2D(float minx, float miny, float maxx, float maxy) : min(Point(minx, miny)), max(Point(maxx, maxy)){}
        Point min;
        Point max;
    };
    
    double vectorSum(std::vector<double> v);
    int vectorSum(std::vector<int> v);
    float vectorSum(std::vector<float> v);

    void lineEquation(float x1, float y1, float x2, float y2, float& a, float& b, float& c);
    void lineEquation(float x1, float y1, float x2, float y2, float& m, float& c);

    
    Bounds2D lineBounds(const Line& l);
    Bounds2D lineBounds(float x1, float y1, float x2, float y2);
    Point lineIntersection(const Line& l1, const Line& l2);
    Point lineIntersection(float x1, float y1, float x2, float y2,
                           float x3, float y3, float x4, float y4);
    
    bool pointInBounds(const Point& p, const Bounds2D& b);

    bool approxEqual(float a, float b, float epsilon);

    Point rotateAroundOrigin(const Point& p, float angle);
    Point rotateAroundOrigin(float x, float y, float angle);

}
