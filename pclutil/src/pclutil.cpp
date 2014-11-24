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

    geometry_msgs::Point pclToGeomPoint(pcl::PointXYZ p){
        geometry_msgs::Point q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        return q;
    }

    /**
     * Rotates a point around (0,0,0). The z coordinate is untouched by the rotation.
     * Expects angle in degrees.
     */
    pcl::PointXYZ rotatePointAroundOriginXY(pcl::PointXYZ p, float angle){
        float rad = (M_PI*angle)/180;
        
        float sin = std::sin(rad);
        float cos = std::cos(rad);

        // rotate point
        float xrot = p.x * cos - p.y * sin;
        float yrot = p.x * sin + p.y * cos;

        return pcl::PointXYZ(xrot, yrot, 0);
    }
}
