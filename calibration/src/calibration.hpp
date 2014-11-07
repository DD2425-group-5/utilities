#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ras_arduino_msgs/ADConverter.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

class calibration{
	public:
	calibration(int argc, char *argv[]);
	
	private:
	ros::Subscriber sub_sensor;	//sub to get distance values
	
	int NUMBERS;
	int channel;
	int start;
	int end;
	int step;
	int current;
	int distance;
	int iteration;
	int running;
	int values[50];
	std::ofstream myfile;
	
	void sensorCallback(const ras_arduino_msgs::ADConverter msg);
	void putObject();
	void PressEnterToContinue();
};