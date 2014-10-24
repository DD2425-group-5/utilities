#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>

class MathUtil {
private:
    MathUtil(){};
public:
    static void histogram(cv::Mat img, int nbins);
};

