#include <ros/ros.h>

class ROSUtil {
public:
    template<typename T>
    static void getParamGeneric(ros::NodeHandle handle, std::string paramName, T &paramVar);
    static void getParam(ros::NodeHandle handle, std::string paramName, int &paramVar);
    static void getParam(ros::NodeHandle handle, std::string paramName, double &paramVar);
    static void getParam(ros::NodeHandle handle, std::string paramName, std::string &paramVar);
    static void getParam(ros::NodeHandle handle, std::string paramName, float &paramVar);
};
