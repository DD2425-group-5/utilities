#include "pclutil.hpp"

namespace PCLUtil {
    void printBounds(CloudBounds<pcl::PointXYZRGB> bounds){
	std::cout << bounds.lower.x << " <= x <= " << bounds.upper.x << std::endl;
	std::cout << bounds.lower.y << " <= y <= " << bounds.upper.y << std::endl;
	std::cout << bounds.lower.z << " <= z <= " << bounds.upper.z << std::endl;
    }

    CloudBounds<pcl::PointXYZRGB> scaleBounds(CloudBounds<pcl::PointXYZRGB> bounds, float scale){
	return scaleBounds(bounds, scale, scale, scale);
    }

    CloudBounds<pcl::PointXYZRGB> scaleBounds(CloudBounds<pcl::PointXYZRGB> bounds, float xScale, float yScale, float zScale){ 
	pcl::PointXYZRGB newLower;
	pcl::PointXYZRGB newUpper;

	newLower.x = bounds.lower.x * xScale;
	newLower.y = bounds.lower.y * yScale;
	newLower.z = bounds.lower.z * zScale;
	newLower.rgb = bounds.lower.rgb;

	newUpper.x = bounds.upper.x * xScale;
	newUpper.y = bounds.upper.y * yScale;
	newUpper.z = bounds.upper.z * zScale;
	newUpper.rgb = bounds.upper.rgb;

	return CloudBounds<pcl::PointXYZRGB>(newLower, newUpper);
    }
    
    bool pointInBounds(pcl::PointXYZRGB point, CloudBounds<pcl::PointXYZRGB> bounds) {
	// short circuit evaluation
	if (point.x < bounds.lower.x || point.x > bounds.upper.x
	    || point.y < bounds.lower.y || point.y > bounds.upper.y
	    || point.z < bounds.lower.z || point.z > bounds.upper.z){
	    return false;
	}
	return true;
    }

    
    pcl::PointXYZRGB initXYZRGB(float x, float y, float z, int r, int g, int b){
	pcl::PointXYZRGB p;
	p.x = x;
	p.y = y;
	p.z = z;
	p.r = r;
	p.g = g;
	p.b = b;
    
	return p;
    }



    /* rotate a single pcl::PointXYZ point */
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToRotate, 
                                                            float angleToRotateTo){
       
    //create the rotation transformation matrix
    Eigen::Affine3f rotationMatrix = Eigen::Affine3f::Identity();
    rotationMatrix.rotate (Eigen::AngleAxisf (angleToRotateTo, Eigen::Vector3f::UnitZ()));
    
    //Apply rotation
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotatedCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloudToRotate, *rotatedCloud, rotationMatrix);
    
    return rotatedCloud;
    
    }




    // std::vector<pcl::PointXYZ> ransacFindLine(const std::vector<pcl::PointXYZ> points,
    //                                           float distanceThreshold){
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //     cloud->width = (int)points.size();
    //     cloud->height = 1;
    //     cloud->is_dense = false;
    //     cloud->points.resize(cloud->width * cloud->height);
        
    //     // push all points in the vector into the cloud
    //     for (size_t i = 0; i < points.size(); i++) {
    //         cloud->push_back(points[i]);
    //     }

    //     std::vector<int> inliers;
        
    //     // Create the sample consensus object using the initialised cloud
    //     pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr 
    //         modelLine(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
        
    //     pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelLine);
    //     ransac.setDistanceThreshold(distanceThreshold);
    //     ransac.computeModel();
    //     ransac.getInliers(inliers);

    //     // Copy points on the line to the output cloud
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr extracted (new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *extracted);
    // }
}
