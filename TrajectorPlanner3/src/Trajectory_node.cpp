#include "trajectory_planner/Constants.hpp"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <geometry_msgs/Pose.h>
using namespace std;
/*
	Uses ros to subscibe to the /map and publishes a path to /path
	Author:Thomas Vy
	Date: June 12 2018
	Email: thomas.vy@ucalgary.ca
*/

ros::Publisher pub; //publisher to path
tf::TransformListener * plr; //transform listener
//sends the transform of the path relative to the map

void calculations (Pose & goal);
nav_msgs::OccupancyGrid::ConstPtr msg;

void sendTransform()
{
	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.0)),   // map and path do need need to be tranformed.
				ros::Time::now(), "map", "path" ));
}
//gets a goal position
void b_cCallback(const geometry_msgs::Pose::ConstPtr& orimsg)
{
cout<<"hello"<<endl;
	Pose goal;
  goal.x = orimsg->position.x; //Stores goal marker so we don't get seg faults
	goal.y = orimsg->position.y;
	if(msg != nullptr)//if goal was called and there is a map avaliable then start the path planning.
	{
		goal.x = (goal.x - (int)msg->info.origin.position.x) / msg->info.resolution; //Changes goal marker real position to grid
		goal.y = (goal.y - (int)msg->info.origin.position.y) / msg->info.resolution; //Changes goal marker real position to grid
		calculations(goal);
	}
}

//if a map message is given, then update the map avaliable.
void publishInfo(const nav_msgs::OccupancyGrid::ConstPtr& orginalmsg)
{
	msg = orginalmsg;
}
//starts the calculations for the path planner
void calculations (Pose & goal)
{
	tf::StampedTransform transform;
  try{
    plr->lookupTransform("/map", "/base_link",
                             msg->info.map_load_time , transform); //looks for the transform between map and the robot
  }
  catch (tf::TransformException ex){
		return; //returns if there is an error in the transformation
  }
	int grid_x = (transform.getOrigin().x() - (int)msg->info.origin.position.x) / msg->info.resolution; //changes the robot real position to the grid
  int grid_y = (transform.getOrigin().y() - (int)msg->info.origin.position.y) / msg->info.resolution;//changes the robot real position to the grid

	int firstx =(int)msg->info.width, firsty =(int)msg->info.height , lastx =-1,lasty =-1; //reduces the size of the map.

	Pose start(grid_x,grid_y, tf::getYaw(transform.getRotation())); //the start position (the robot's current position)

	matrix original((int)msg->info.height, vector<double>((int)msg->info.width)); //the original map in a 2d vector
	for(int y =0 , k=0; y<(int)msg->info.height; y++)
	{
		for(int x=0; x<(int)msg->info.width ;x++)
		{
			int value =  msg->data[k++];
			//make start and end
			original[x][y] = value;
			if(value!=-1) //checks for empty or wall space in the map
			{
				if(x<firstx)
					firstx = x;
				if(y<firsty)
					firsty = y;
				if(x>lastx)
					lastx = x;
				if(y>lasty)
					lasty =y;
			}
		}
	}
	if(lastx<firstx || lasty<firsty) //If there are no bounds.
	{
			return;
	}
	Pose first(firstx, firsty);
	Pose last(lastx, lasty);
	Image img(original, first, last); //creates a converted image bsaed off original map
	img.insert_borders(); //creates cost map

	nav_msgs::Path path;
	static int num =0;
	path.header.seq = num++;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "path";
	ROS_INFO("Goal Coordinates -  x: [%f] y: [%f]", goal.x, goal.y);
	if(original[goal.x][goal.y]!=0) // Cannot reach the goal position at the moment
	{
		if(img.findNearestFreeSpace(goal, start)) //finds the nearest unknown space and creates a path to that location
		{
			ROS_INFO("Found Path");
			path.poses = img.getPath();
			for(int i =0; i<path.poses.size();i++)
			{
				path.poses[i].pose.position.x = (path.poses[i].pose.position.x*msg->info.resolution+msg->info.origin.position.x);
				path.poses[i].pose.position.y = (path.poses[i].pose.position.y*msg->info.resolution+msg->info.origin.position.y);
			}
		}
		else{ //There is no unknown avaliable
			ROS_ERROR("Could NOT find path");
			return;
		}
	}
	else if(img.planner(start, goal)) //plans the path if it find a path it publishes it
	{
		ROS_INFO("Found Path");
		path.poses = img.getPath();
		for(int i =0; i<path.poses.size();i++)
		{
			path.poses[i].pose.position.x = (path.poses[i].pose.position.x*msg->info.resolution+msg->info.origin.position.x);
			path.poses[i].pose.position.y = (path.poses[i].pose.position.y*msg->info.resolution+msg->info.origin.position.y);
		}
	}
	else
	{
		ROS_ERROR("Could NOT find path");
		return;
	}
	sendTransform();
	pub.publish(path);
}

int main(int argc, char ** argv)
{
	Pose goal;
	ros::init(argc, argv, "TrajectoryNode"); //init the node
	ros::NodeHandle n;
	pub = n.advertise<nav_msgs::Path>("path", 1000);// publishes to path
	ros::Subscriber sub = n.subscribe("map", 1000, publishInfo);// subscribes to map
	ros::Subscriber goal_marker_sub = n.subscribe("goal_post", 100, b_cCallback); //subscribes to feedback from basic_controls
	tf::TransformListener listener; //tf listener
	plr = &listener;
	ros::spin();
	return 0;
}
