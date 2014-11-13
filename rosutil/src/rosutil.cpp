#include "rosutil.hpp"

namespace ROSUtil {
/**
 * Get a parameter from the parameter server with name paramName and type T, and
 * assign its value to paramVar. If there is no parameter on the server with
 * that name, print an error and stop execution.
 */
    template<typename T>
    void getParamGeneric(ros::NodeHandle handle, std::string paramName, T &paramVar) {
	if (!handle.getParam(paramName, paramVar)){
	    ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	    std::exit(1);
	}
	ROS_INFO_STREAM("Successfully loaded param " << paramName << " with value " << paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, int &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, bool &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, double &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::string &paramVar) {
	getParamGeneric(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, float &paramVar) {
	double tmp;
	getParamGeneric(handle, paramName, tmp);
	paramVar = tmp;
    }

    template<typename T>
    void getParamGenericVec(ros::NodeHandle handle, std::string paramName, std::vector<T> &paramVar) {
	if (!handle.getParam(paramName, paramVar)){
	    ROS_ERROR_STREAM("Parameter " << paramName << " has not been defined!");
	    std::exit(1);
	}
	ROS_INFO_STREAM("Successfully loaded vector param " << paramName);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<std::string> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<bool> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<int> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<float> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

    void getParam(ros::NodeHandle handle, std::string paramName, std::vector<double> &paramVar) {
	getParamGenericVec(handle, paramName, paramVar);
    }

}
