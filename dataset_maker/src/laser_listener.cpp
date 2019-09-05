#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "iostream"
#include "fstream"
void formalization(const sensor_msgs::LaserScan& msg)
{
	std::ofstream output ;
	output.open("/home/leo/h20_ws/dataset_laser.txt",std::ios::app) ;
	//------------------------TO DO-----------------------//
	//How to determine the amount of data before collecting
	//UST-20LX data amount 721-723
	output<<"FLASER 723 ";
	int length,data ;
	//length = sizeof(msg.ranges)/sizeof(msg.ranges[0]) ;
	for(data=0; ;data++)
	{
		if(!msg.ranges[data])
		{
			//std::cout<<data<<std::endl ;
			output<<'\n' ;
			break ;
		}		
		output<<msg.ranges[data]<<' ' ;

	}
	//std::cout<<length ;
	//std::cout<<' '<<msg.ranges[1]<<std::endl ;
	output.close() ;
}

int main(int argc, char **argv)
{
	std::ofstream output ;
	//create file and clean previous data
	output.open("/home/leo/h20_ws/dataset_laser.txt",std::ios::trunc) ;
	output.close() ;
	
	ros::init(argc,argv,"laser_listener") ;
	ros::NodeHandle n ;
	ros::Subscriber sub ;
	sub = n.subscribe("scan",20,formalization);
	ros::spin() ;
	return 0 ;
}
