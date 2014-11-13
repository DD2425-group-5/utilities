#include "calibration.hpp"
//using namespace std;

void calibration::sensorCallback(const ras_arduino_msgs::ADConverter msg){
	int tmp[] ={msg.ch1,msg.ch2,msg.ch3,msg.ch4,msg.ch7,msg.ch8};
	
	/*ROS_INFO("sensor distance: 1: [%d] 2: [%d] 3: [%d] 4: [%d] 5: [%d] 6: [%d] \n",\
	tmp[0],\
	tmp[1],\
	tmp[2],\
	tmp[3],\
	tmp[4],\
	tmp[5]);*/
	if(distance>end){
		myfile.close();
		std::exit(0);
	}
	
	if(iteration<NUMBERS && running){
		values[iteration]=tmp[channel];
		iteration++;
		running=1;
		ROS_INFO("sensor point: [%d]",iteration);
	}
	else{
		if(running /*&& distance<end*/){
			/*if(distance>end){
				std::exit(0);
			}*/
			double avg=0;
			for(int i=0;i<NUMBERS;i++){
				avg+=values[i];
			}
			avg=avg/NUMBERS;
			std::cout << "The average at "<<(distance)<<" cm is "<<(avg)<<"\n";
			myfile << distance <<", " <<avg<<"\n";
			distance=distance+step;
			running=0;
			iteration=0;
		}
	}
}

calibration::calibration(int argc, char *argv[]){
	ros::init(argc, argv, "calibration");	//name of node
	ros::NodeHandle handle;					//the handle
	
	NUMBERS = 50;
	channel=-1;
	int c=0;
	end = 1;
	start = 1;
	step = 1;
	running=0;
	distance = 0;
	iteration = 0;;
	sub_sensor = handle.subscribe("/arduino/adc", 1, &calibration::sensorCallback, this);
	
	//struct passwd *pw = getpwuid(getuid());
	//const char *homedir = pw->pw_dir;
	//std::cout << getenv("HOME") <<"/data.txt\n";
	//string str = homedir;
	//string dir = string(getenv("HOME"))+"/Desktop/testfile.txt";
	std::string test(getenv("HOME"));
	test=test+"/data"; //.txt";
	
	std::cout << "\n\nAnti Gnome calibrator V 0.1\n";
	std::cout << "Enter the adc channel [0-5]\n";
	std::cin >> channel;
	std::ostringstream convert;
	convert << channel;
	test=test+convert.str()+".txt";
	std::cout << "Data will be stored at " << test << "\n";
	std::cout << "Enter the start value in cm\n";
	std::cin >> start;
	std::cout << "Enter the end value in cm\n";
	std::cin >> end;
	std::cout << "Enter the step size\n";
	std::cin >> step;
	std::cout << "This will generate "<<((end-start)/step+1)<<" points\n";
	myfile.open(test.c_str());
	PressEnterToContinue();
	distance=start;
	
	ros::Rate loop_rate(10);	//10 Hz
	while (ros::ok()){
		if(!running){
			if(distance>end){
				myfile.close();
				std::exit(0);
			}
			putObject();
		}
		//ROS_INFO("RUNNING");
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void calibration::putObject(){
	std::cout << "Please put the object  "<<(distance)<<" cm from the sensor\n";
	PressEnterToContinue();
	running=1;
}

void calibration::PressEnterToContinue(){
	int c;
	printf( "Press ENTER to continue...\n" );
	fflush( stdout );
	do c = getchar(); while ((c != '\n') && (c != EOF));
}

int main(int argc, char *argv[]) 
{
    calibration calibration(argc, argv);
}