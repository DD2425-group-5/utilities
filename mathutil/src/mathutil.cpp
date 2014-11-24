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
     * coefficients of the line ax+by+c are returned through the references.
     */
    void lineEquation(float x1, float y1, float x2, float y2, float& a, float& b, float& c){
        float dx = x2 - x1;
        float dy = y2 - y1;
        
        a = dy;
        b = -dx;
        c = y1 * dx - x1 * dy;
    }
}
