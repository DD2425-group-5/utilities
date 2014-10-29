#include "visionmath.hpp"

namespace VisionMath {
    void histogram(cv::Mat img, int nbins) {
	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::min();
	int lzeroc = 0;
    
	for (int row = 0; row < img.rows; row++) {
	    const float* rowptr = img.ptr<float>(row); // extract row of the matrix
	    // create depthpoints and insert them into the flattened vector.
	    for (int col = 0; col < img.cols; col++) {
		if (rowptr[col] < min)
		    min = rowptr[col];
		if (rowptr[col] > max)
		    max = rowptr[col];
		if (rowptr[col] <= 0)
		    lzeroc++;
	    }
	}

	float rangeSkip = (max - min)/nbins;
	std::cout << rangeSkip << std::endl;
	std::vector<float> binRanges;
	binRanges.push_back(min);
	for (int i = 1; i < nbins; i++) {
	    binRanges.push_back(binRanges[i - 1] + rangeSkip);
	}

	std::vector<int> hist(nbins);
    
	for (int row = 0; row < img.rows; row++) {
	    const float* rowptr = img.ptr<float>(row); // extract row of the matrix
	    // create depthpoints and insert them into the flattened vector.
	    for (int col = 0; col < img.cols; col++) {
		for (int bin = 1; bin < nbins; bin++) {
		    if (rowptr[col] <= binRanges[bin]){
			hist[bin - 1]++;
			break;
		    }
		}

	    }
	}

	for (int i = 0; i < nbins; i++) {
	    std::cout << i << "th bin: " << hist[i] << std::endl;
	}
    
	std::cout << "max is " << max << ", min is " << min << ", lzero is " << lzeroc << std::endl;
	std::exit(0);
    }
}
