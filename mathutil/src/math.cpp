#include "math.hpp"

using namespace std;

namespace mathutil {
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
}
