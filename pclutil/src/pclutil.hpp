#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl_ros/point_cloud.h>


namespace PCLUtil {
    template <typename T>
    struct CloudBounds {
	CloudBounds(T l, T u) : lower(l), upper(u) {}
	T lower;
	T upper;
    };

    void printBounds(CloudBounds<pcl::PointXYZRGB> bounds);
    CloudBounds<pcl::PointXYZRGB> scaleBounds(CloudBounds<pcl::PointXYZRGB> bounds,
					      float xScale, float yScale, float zScale);

    template <typename T>
    CloudBounds<T> getCloudBounds(const typename pcl::PointCloud<T>::ConstPtr& cloud){
	T min;
	T max;
	pcl::getMinMax3D(*cloud, min, max);
	return CloudBounds<T>(min, max);
    }

    bool pointInBounds(pcl::PointXYZRGB point, CloudBounds<pcl::PointXYZRGB> bounds);

    CloudBounds<pcl::PointXYZRGB> scaleBounds(CloudBounds<pcl::PointXYZRGB> bounds,
					      float scale);

    pcl::PointXYZRGB initXYZRGB(float x, float y, float z, int r, int g, int b);
//    std::vector<pcl::PointXYZ> ransacFindLine(const std::vector<pcl::PointXYZ> points,
//                                              float distanceThreshold);
}
