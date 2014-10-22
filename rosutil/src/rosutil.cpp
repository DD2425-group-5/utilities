#include "rosutil.hpp"

/**
 * Get a parameter from the parameter server with name paramName and type T, and
 * assign its value to paramVar. If there is no parameter on the server with
 * that name, print an error and stop execution.
 */
template<typename T>
void ROSUtil::getParamGeneric(ros::NodeHandle handle, std::string paramName, T &paramVar) {
    if (!handle.getParam(paramName, paramVar)){
	ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	std::exit(1);
    }
    ROS_INFO_STREAM("Successfully loaded param " << paramName << " with value " << paramVar);
}

void ROSUtil::getParam(ros::NodeHandle handle, std::string paramName, int &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

void ROSUtil::getParam(ros::NodeHandle handle, std::string paramName, bool &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

void ROSUtil::getParam(ros::NodeHandle handle, std::string paramName, double &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

void ROSUtil::getParam(ros::NodeHandle handle, std::string paramName, std::string &paramVar) {
    getParamGeneric(handle, paramName, paramVar);
}

void ROSUtil::getParam(ros::NodeHandle handle, std::string paramName, float &paramVar) {
    double tmp;
    getParamGeneric(handle, paramName, tmp);
    paramVar = tmp;
}
