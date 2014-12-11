#include "mathutil.hpp"

using namespace std;

namespace MathUtil {
    double vectorSum(std::vector<double> v){
	double sum = 0;
	for (size_t i = 0; i < v.size(); i++) {
	    sum += v[i];
	}
	return sum;
    }

    int vectorSum(std::vector<int> v){
	int sum = 0;
	for (size_t i = 0; i < v.size(); i++) {
	    sum += v[i];
	}
	return sum;
    }

    float vectorSum(std::vector<float> v){
	float sum = 0;
	for (size_t i = 0; i < v.size(); i++) {
	    sum += v[i];
	}
	return sum;
    }

    /**
     * Compute the line equation for the line from point (x1,y1) to (x2,y2). The
     * coefficients of the line ax+by+c=0 are returned through the references.
     */
    void lineEquation(float x1, float y1, float x2, float y2, float& a, float& b, float& c){
        float dx = x2 - x1;
        float dy = y2 - y1;
        
        a = dy;
        b = -dx;
        c = y1 * dx - x1 * dy;
    }

    /**
     * Compute the line equation for the line from point (x1,y1) to (x2,y2). The
     * coefficients of the line y=mx+c are returned through the references.
     */
    void lineEquation(float x1, float y1, float x2, float y2, float& m, float& c){
        float dx = x2 - x1;
        float dy = y2 - y1;
        
        m = dy/dx;
        c = y1-m*x1;
    }

    Point lineIntersection(const Line& l1, const Line& l2){
        return lineIntersection(l1.begin.x, l1.begin.y, l1.end.x, l1.end.y,
                                l2.begin.x, l2.begin.y, l2.end.x, l2.end.y);
    }

    /**
     * Compute the intersection of the lines given by the four specified points.
     * 1-2 is a line, 3-4 is a line. The determinant is used to compute the intersection
     * http://en.wikipedia.org/wiki/Line%E2%80%93line_intersection.
     *
     * throws an exception with integer value 1 if the lines are parallel or coincident
     */
    Point lineIntersection(float x1, float y1, float x2, float y2,
                           float x3, float y3, float x4, float y4){
        float denominator = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
        
        if (approxEqual(0, denominator, 0.00001)){ // lines are parallel/coincident -> denominator is zero
            throw 1; // throw integer error, just for simplicity
        }
        float x = ((x1 * y2 - y1 * x2)*(x3 - x4) - (x1 - x2)*(x3 * y4 - y3 * x4))/denominator;
        float y = ((x1 * y2 - y1 * x2)*(y3 - y4) - (y1 - y2)*(x3 * y4 - y3 * x4))/denominator;

        return Point(x, y);
    }

    Bounds2D lineBounds(const Line& l, float shrink){
        return lineBounds(l.begin.x, l.begin.y, l.end.x, l.end.y, shrink);
    }
    

    /**
     * Find the min max of the points that make up a line. Return a line which
     * goes from the minimum to the maximum. line.begin.x = minx, line.begin.y =
     * miny, line.end.x = maxx, line.end.y = maxy. The shrink parameter defines
     * how much is added (min) or subtracted from (max) the values found. This
     * will reduce the size of the bounds
     */
    Bounds2D lineBounds(float x1, float y1, float x2, float y2, float shrink){
        float minX = x1 < x2 ? x1 : x2;
        float maxX = x1 > x2 ? x1 : x2;
        float minY = y1 < y2 ? y1 : y2;
        float maxY = y1 > y2 ? y1 : y2;

        // if the line is horizontal or vertical, the shrinking should only be
        // added in the horizontal or vertical direction, otherwise the bounding
        // box of the line will be shifted somewhere that is not actually over
        // the line.
        if (approxEqual(minY, maxY, 0.00001)){
            // if the line is horizontal, only the min and max x values should
            // be shrunk
            minX += shrink;
            maxX -= shrink;
        } else if (approxEqual(minX, maxX, 0.00001)){
            // if the line is vertical, only the min and max y values should be
            // shrunk
            minY += shrink;
            maxY -= shrink;
        } else { // line is not horizontal or vertical, shrink all values
            minX += shrink;
            maxX -= shrink;
            minY += shrink;
            maxY -= shrink;
        }
        
        Bounds2D minmax(minX, minY, maxX, maxY);

        return minmax;
    }

    bool pointInBounds(const Point& p, const Bounds2D& b){
        return p.x >= b.min.x && p.x <= b.max.x
               && p.y >= b.min.y && p.y <= b.max.y;
    }
    
    bool approxEqual(float a, float b, float epsilon){
        return std::fabs(a - b) < epsilon;
    }

    Point rotateAroundOrigin(const Point& p, float angle){
        return rotateAroundOrigin(p.x, p.y, angle);
    }
    
    Point rotateAroundOrigin(float x, float y, float angle){
        float rad = (M_PI*angle)/180;
        float sin = std::sin(rad);
        float cos = std::cos(rad);

        float newx = x * cos - y * sin;
        float newy = x * sin + y * cos;
        
        return Point(newx, newy);
    }
}
