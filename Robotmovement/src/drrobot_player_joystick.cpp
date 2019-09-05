#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <drrobot_jaguar4x4_player/Matrix.h>
#include <drrobot_jaguar4x4_player/Eigen/Dense>
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


#include <std_msgs/Float64.h>
#include <std_msgs/MultiArrayLayout.h>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"


#include <sstream>  //added
#include <drrobot_jaguar4x4_player/variable.h>  //added
#include <drrobot_jaguar4x4_player/Rscanpose.h>  //added for save the scan plus encorder for EKF implemetation offline
#include <drrobot_jaguar4x4_player/KFodom.h>  //added
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/LaserScan.h"

#include <drrobot_jaguar4x4_player/MotorInfo.h>
#include <drrobot_jaguar4x4_player/MotorInfoArray.h>
#include <drrobot_jaguar4x4_player/RangeArray.h>
#include <drrobot_jaguar4x4_player/Range.h>
#include <drrobot_jaguar4x4_player/PowerInfo.h>
#include <drrobot_jaguar4x4_player/StandardSensor.h>
#include <drrobot_jaguar4x4_player/CustomSensor.h>
#include <drrobot_jaguar4x4_player/DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM       6
#define IR_NUM          10
#define US_NUM          6
#define cFULL_COUNT				32767
#define cWHOLE_RANGE			1200
#define ENCODER_SCALAR_H20		0.00155		//1revoultion of wheel = .53m/400encoder readings//theoretical 0.001399055
#define TRACK					0.425				//Distance between two wheels(cm)

using namespace Eigen;
using namespace std;
using namespace DrRobot_MotionSensorDriver;
//----------------------------------------------------------------------
	double dt;
	int encoder1, encoder2;

	int encoderLeft=0, encoderRight=0;

	int encoderDir_L, encoderDir_R;
	int encoder_difference=0;
	int temp_encorderLeft;
	int temp_encorderRight;
	int diff_encoderLeft=0;
	int diff_encoderRight=0;
	int preEncoder1=0;
	int preEncoder2=0;


	int pre_encorder_difference=0;
    int rate_encorder_difference=0;

	double wheel_speed_L, wheel_speed_R;

	int initial = 0;
	double x_1=0, y_1=0,theta_1=0, rotational_theta=0;
//----------------------------------------------------------------------
	double x_displacement=0;

	double x=0,y=0,theta=0,alpha=0,previous_alpha=0,alpha_dot=0,alpha_I=0,beta=0;

	double p=0, p_dot=0,previous_p=0;
	double g_coordinates_x=0,x_coordinates_error=0;
    double g_coordinates_y=0,y_coordinates_error=0;
    double t_angle=0,t_angle_old=0;

	double deltaD=0;
	double deltaTheta=0;
	double thetaIntergral=0;
	double velocity_angle=0;
//----------------------------------------------------------------------
	double Vx=0, Vy=0,Vz=0,Vxy=0,Vr=0,Wr=0,tog=0,ptog; //word coordinate /robot coordinates frames
	double  robotVx=0; //No robotVy
	double Vcommand=0,command=0;

	double f_w_left=0,f_w_right=0;
	double w_left=0;
	double w_right=0;
//----------------------------------------------------------------------
	double command_v=50,s=0;
//----------------------------------------------------------------------
	int loopcount=0;
	double pi=M_PI; //3.1415927;
//----------------------------------------------------------------------

	double gvariable1=0;
	double gvariable2=0;
	double gvariable0=0;
//----------------------------------------------------------------------
    int left_PWM=16384,right_PWM=16384;
    int Wheel_1_velocity[3],Wheel_2_velocity[3];
    int W_left_wheel_error_I=0,W_right_wheel_error_I=0;
    int W_left_wheel_error_Previous=0,W_right_wheel_error_Previous=0;

    int w_left_wheel, w_right_wheel,w_pre_left_wheel=0,w_pre_right_wheel=0;
    int D_encorder1,D_encorder2;
    double K_e_l,K_e_r;
//----------------------------------------------------------------------
	double scanpose[645];
//----------------------------------------------------------------------


class DrRobotPlayerNode
{
public:

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster path_broadcaster;

    ros::Publisher motorInfo_pub_;
    ros::Publisher powerInfo_pub_;
    ros::Publisher ir_pub_;
    ros::Publisher sonar_pub_;
    ros::Publisher standardSensor_pub_;
    ros::Publisher customSensor_pub_;
	ros::Publisher odom_pub_;
	ros::Publisher Rscanpose_publisher;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_DesiredPose_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber joy_sub;
  	ros::Subscriber path_sub;
	ros::Publisher path_pub_;
	ros::Publisher variable_publisher;
	ros::Publisher encorder_publisher;


    ros::Subscriber new_cmd_sub_;
    std::string robot_prefix_;

	ros::Time current_time, last_time;

	double enc_count_left;   //encorder count
	double enc_count_right;
	double x;
	double y;
	double th;
	double vr;
	double vl;
	double vth;

//======================================================================

    DrRobotPlayerNode()
    {
        ros::NodeHandle private_nh("~");

        robotID_ = "drobot1";
        private_nh.getParam("RobotID",robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        robotType_ = "Hawk_H20";
        private_nh.getParam("RobotType",robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        robotCommMethod_ = "Network";
        private_nh.getParam("RobotCommMethod",robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        robotIP_ = "192.168.0.95";
        private_nh.getParam("RobotBaseIP",robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        commPortNum_ = 10001;
        private_nh.getParam("RobotPortNum",commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("RobotSerialPort",robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        enable_ir_ = true;
        private_nh.getParam("Enable_IR", enable_ir_);
        if (enable_ir_)
          ROS_INFO("I get Enable_IR: true");
        else
          ROS_INFO("I get Enable_IR: false");


        enable_sonar_ = true;
        private_nh.getParam("Enable_US", enable_sonar_);
        if (enable_sonar_)
          ROS_INFO("I get Enable_US: true");
        else
          ROS_INFO("I get Enable_US: false");

        motorDir_ = 1;
        private_nh.getParam("MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.0835;
        private_nh.getParam("WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.305;
        private_nh.getParam("WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 1.0;
        private_nh.getParam("MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 400;
        private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
          robotConfig1_.commMethod = Network;
          robotConfig2_.commMethod = Network;
        }
        else
        {
          robotConfig1_.commMethod = Serial;
          robotConfig2_.commMethod = Serial;
        }

        if (robotType_ == "Hawk_H20")
        {
          robotConfig1_.boardType = Hawk_H20_Power;
          robotConfig2_.boardType = Hawk_H20_Motion;
        }


        robotConfig1_.portNum = commPortNum_;
        robotConfig2_.portNum = commPortNum_ + 1;

      //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig1_.robotIP,robotIP_.c_str());
      //  strcat(robotConfig2_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig2_.robotIP,robotIP_.c_str());

      //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());
      //  strcat(robotConfig2_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig2_.serialPortName,robotSerialPort_.c_str());


        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<drrobot_jaguar4x4_player::MotorInfoArray>("drrobot_motor", 10);
        powerInfo_pub_ = node_.advertise<drrobot_jaguar4x4_player::PowerInfo>("drrobot_powerinfo", 10);
        if (enable_ir_) { ir_pub_ = node_.advertise<drrobot_jaguar4x4_player::RangeArray>("drrobot_ir", 10); }
        if (enable_sonar_) { sonar_pub_ = node_.advertise<drrobot_jaguar4x4_player::RangeArray>("drrobot_sonar",10); }
        standardSensor_pub_ = node_.advertise<drrobot_jaguar4x4_player::StandardSensor>("drrobot_standardsensor", 10);
        customSensor_pub_ = node_.advertise<drrobot_jaguar4x4_player::CustomSensor>("drrobot_customsensor", 10);

		odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom", 1000);
		Rscanpose_publisher= node_.advertise<drrobot_jaguar4x4_player::Rscanpose>("Rscanpose", 1000);//added to save scan plus encorder for EKF offline
		path_pub_ = node_.advertise<nav_msgs::Path>("path", 1000);
		variable_publisher = node_.advertise<drrobot_jaguar4x4_player::variable>("variable", 1000);//added
		encorder_publisher = node_.advertise<std_msgs::Int32MultiArray>("encorder", 1000);//added


        drrobotPowerDriver_ = new DrRobotMotionSensorDriver();
        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        if (  (robotType_ == "Jaguar") )
        {
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        }
        else
        {
          drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);
        }
        cntNum_ = 0;

		x = 0.0;
		y = 0.0;
		th = 0.0;



		current_time = ros::Time::now();
		last_time = ros::Time::now();

		enc_count_left=0; //encorder init
		enc_count_right=0;

    }

    ~DrRobotPlayerNode()
    {
    }
//======================================================================
    int start()
    {

      int res = -1;
        drrobotMotionDriver_->openNetwork(robotConfig2_.robotIP,robotConfig2_.portNum);
        drrobotPowerDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);


     cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));
     scan_sub= node_.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&DrRobotPlayerNode::processLaserScan, this, _1));
     joy_sub= node_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&DrRobotPlayerNode::get_joystick, this, _1));
     new_cmd_sub_ = node_.subscribe<geometry_msgs::Twist>("new_cmd", 1, boost::bind(&DrRobotPlayerNode::cmdNewCommand, this, _1));
		 path_sub = node_.subscribe<nav_msgs::Path>("path",1, boost::bind(&DrRobotPlayerNode::pathRecieved, this, _1) );


       return(0);
    }
		void pathRecieved (const nav_msgs::Path::ConstPtr& path)
		{
			exit(0);
		}
//======================================================================
    int stop()
    {
        int status = 0;
         drrobotMotionDriver_->close();
        drrobotPowerDriver_->close();
        usleep(1000000);
        return(status);
    }
//======================================================================
	void cmdNewCommand(const geometry_msgs::Twist::ConstPtr& new_cmd)
	{
			ROS_INFO("Got new command");

	}

//======================================================================
 void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {


     //g_coordinates_x= cmd_vel->linear.x;
     //g_coordinates_y= cmd_vel->linear.y;
     //t_angle = cmd_vel->angular.z;
     //command=cmd_vel->linear.z;


    }

//======================================================================
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){

	//gvariable1=scan->ranges[15];
	int i=5;
	for(i=5;i<645;i++){
		scanpose[i]=scan->ranges[i-5];
	}
}

//======================================================================
void get_joystick(const sensor_msgs::Joy::ConstPtr& msg){

	//speed_factor=msg->axes[3]
	Vr=(msg->axes[3])*(msg->axes[1]);
	Wr=(msg->axes[3])*(msg->axes[2]);
}
//======================================================================
 void doUpdate()
    {
		ROS_INFO("Wheel: [V= %f,W= %f]",Vr,Wr);
		ROS_INFO("Wheel: [L= %f,R= %f]",w_left,w_right);
		ROS_INFO("Current: [X= %f,Y= %f,Theta= %f]",x,y,theta*180/pi);


		update_parameters();
		move(Vr,Wr,tog);

		publish_variables();
		//publish_encorder();


      if (robotConfig1_.boardType == Hawk_H20_Power)
      {
        if (drrobotPowerDriver_->portOpen())
        {
          drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
          drrobot_jaguar4x4_player::PowerInfo powerInfo;
          powerInfo.ref_vol = 1.5 * 4095 /(double)powerSensorData_.refVol;

          powerInfo.bat1_vol = (double)powerSensorData_.battery1Vol  * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;

          powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
          powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

          powerInfo.dcin_vol = (double)powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.charge_path = powerSensorData_.powerChargePath;
          powerInfo.power_path = powerSensorData_.powerPath;
          powerInfo.power_status = powerSensorData_.powerStatus;

          powerInfo_pub_.publish(powerInfo);
        }
      }
      if (drrobotMotionDriver_->portOpen())
      {
        drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
        drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
        drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);

        drrobotMotionDriver_->readCustomSensorData(&customSensorData_);
              // Translate from driver data to ROS data
            cntNum_++;
              drrobot_jaguar4x4_player::MotorInfoArray motorInfoArray;
              motorInfoArray.motorInfos.resize(MOTOR_NUM);
              for (uint32_t i = 0 ; i < MOTOR_NUM; ++i)
              {
                  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                  motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                  motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                  motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                  motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                  motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                  motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                  if (robotConfig1_.boardType == Hawk_H20_Motion)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] * 3 /4096;;
                  }
                  else if(robotConfig1_.boardType != Jaguar)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] / 728;
                  }
                  else
                  {
                    motorInfoArray.motorInfos[i].motor_current = 0.0;
                  }
                  motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
              }


              motorInfo_pub_.publish(motorInfoArray);
              drrobot_jaguar4x4_player::RangeArray rangerArray;
              rangerArray.ranges.resize(US_NUM);
	      if(enable_sonar_)
	      {
		      for (uint32_t i = 0 ; i < US_NUM; ++i)
		      {

		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = (float)rangeSensorData_.usRangeSensor[i]/100;     //to meters

		          // around 30 degrees
		          rangerArray.ranges[i].field_of_view = 0.5236085;
		          rangerArray.ranges[i].max_range = 2.55;
		          rangerArray.ranges[i].min_range = 0;
		          rangerArray.ranges[i].radiation_type = drrobot_jaguar4x4_player::Range::ULTRASOUND;
		      }

		      sonar_pub_.publish(rangerArray);
		}


	      if(enable_ir_)
	      {
		      rangerArray.ranges.resize(IR_NUM);
		      for (uint32_t i = 0 ; i < IR_NUM; ++i)
		      {
		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
		          rangerArray.ranges[i].radiation_type = drrobot_jaguar4x4_player::Range::INFRARED;
		      }

		      ir_pub_.publish(rangerArray);
	     }

              drrobot_jaguar4x4_player::StandardSensor standardSensor;
              standardSensor.humanSensorData.resize(4);
              standardSensor.tiltingSensorData.resize(2);
              standardSensor.overHeatSensorData.resize(2);
              standardSensor.header.stamp = ros::Time::now();
              standardSensor.header.frame_id = string("drrobot_standardsensor");
              for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

              standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

              standardSensor.boardPowerVol = (double)standardSensorData_.boardPowerVol * 9 /4095;
              standardSensor.servoPowerVol = (double)standardSensorData_.servoPowerVol * 9 /4095;

              if (robotConfig1_.boardType != Jaguar)
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 24 /4095;
              }
              else
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
              }
              standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
              standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
              standardSensor_pub_.publish(standardSensor);

              drrobot_jaguar4x4_player::CustomSensor customSensor;
              customSensor.customADData.resize(8);
              customSensor.header.stamp = ros::Time::now();
              customSensor.header.frame_id = string("drrobot_customsensor");

              for (uint32_t i = 0; i < 8; i ++)
              {
                customSensor.customADData[i] = customSensorData_.customADData[i];
              }
              customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
              customSensor_pub_.publish(customSensor);
      }
      ComputeSteering(motorSensorData_.motorSensorEncoderPos[0],motorSensorData_.motorSensorEncoderPos[1],motorSensorData_.motorSensorEncoderVel[0],motorSensorData_.motorSensorEncoderVel[1],motorSensorData_.motorSensorEncoderDir[0],motorSensorData_.motorSensorEncoderDir[1]);

    }

//======================================================================
void sendTransform()
	{
	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "base_link", "camera_link" ));  //parent, child

	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "map", "scans" ));  //parent, child

	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "base_link", "camera_depth_frame" ));  //parent, child

	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "map", "odom" ));  //parent, child

	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(),  "world","map"));  //parent, child

	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "base_link", "laser" ));  //parent, child

	  broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "map", "points3d" ));  //parent, child

      broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "map", "feature" ));  //parent, child

       broadcaster_.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),   //x,y,z (0.05, 0.0, 0.75)
      ros::Time::now(), "map", "featurepoint" ));  //parent, child
}
//======================================================================
void publish_variables(){
drrobot_jaguar4x4_player::variable var;
var.variable0=gvariable0;
var.variable1=gvariable1;
var.variable2=gvariable2;

variable_publisher.publish(var);
}
//======================================================================
void publish_Rscanpose(){
drrobot_jaguar4x4_player::Rscanpose msg;

int i=0;
for(i=0;i<645;i++){
			msg.Rscanpose[i]=scanpose[i];
		}

Rscanpose_publisher.publish(msg);
}
//======================================================================
void publish_encorder(){
std_msgs::Int32MultiArray encorderArray;
encorderArray.data.clear();
encorderArray.data.push_back(D_encorder1);
encorderArray.data.push_back(D_encorder2);
encorder_publisher.publish(encorderArray);
}

//----------------------------------------------------------------------
//				 control
//----------------------------------------------------------------------

void move(double Vr, double Wr,double tog) //target values
{


	w_left=(2*Vr-Wr*TRACK)/(2*wheelRadius_ ); //WL=(2*v-w*d)/(2*r);
	w_right=(2*Vr+Wr*TRACK)/(2*wheelRadius_ ); //WR=(2*v+w*d)/(2*r);
	drrobotMotionDriver_->sendMotorCtrlAllCmd(PWM,left_PWM, right_PWM,NOCONTROL,NOCONTROL, NOCONTROL,NOCONTROL);/////	///// use this-Recent PID controller move
	Vr=0.0;
	Wr=0.0;


}


// ------------------------------------------------------
//					EliminateRollover
// ------------------------------------------------------
int EliminateRollover(int currEncoder, int preEncoder, int encoderDir)
{
	int diffEncoder = (currEncoder - preEncoder);	//As robot go forward, left encoder decreases and

																//Right encoder increases...

	if(((diffEncoder) > 10000)||((diffEncoder) <-10000))
	{
		if(diffEncoder<0)

			diffEncoder=(cFULL_COUNT-preEncoder+currEncoder+1);

		else
			diffEncoder=-(cFULL_COUNT-currEncoder+preEncoder+1);
	}
	else
	{
		//return diffEncoder;
	}


	return diffEncoder;
}

// ------------------------------------------------------(1)
//					GetDeltaDisplacement
// ------------------------------------------------------
double GetDeltaDisplacement(int dEncoder1,int dEncoder2)
{

	double dispLeft		= ENCODER_SCALAR_H20*dEncoder1;
	double dispRight	= ENCODER_SCALAR_H20*dEncoder2;

	return (dispLeft+dispRight)/2;

}

//-------------------------------------------------------(2)
//					Robot Velocity Profiles
//-------------------------------------------------------
double get_velocity(double Vx, double Vy, double Theta)
{

	Vxy=sqrt (Vx*Vx+Vy*Vy);

}
// ------------------------------------------------------(3)
//					GetDeltaAngle
// ------------------------------------------------------
double GetDeltaAngle(int diffEncoder1,int diffEncoder2)
{
	double dispLeft		= ENCODER_SCALAR_H20*diffEncoder1;
	double dispRight	= ENCODER_SCALAR_H20*diffEncoder2;

	return ((-dispLeft+dispRight)/TRACK);
}
//-------------------------------------------------------(4)
//					Control wheel Velocity Profiles
//-------------------------------------------------------
double control_wheel_velocity(double w_left_d,double w_right_d,int LeftwheelW,int RightwheelW )
{


	double KwP=21;
	double KwD=4;
	double KwI=7;

	//******************************************************************

	int w_left_error_D=0;
	int w_right_error_D=0;

	int w_left_error=w_left_d-LeftwheelW;
	int w_right_error=w_right_d-RightwheelW;

	if(abs(w_left_error-W_left_wheel_error_Previous)<100) w_left_error_D=(w_left_error-W_left_wheel_error_Previous)/dt; //avoid startup impulse
	if(abs(w_right_error-W_right_wheel_error_Previous)<100) w_right_error_D=(w_right_error-W_right_wheel_error_Previous)/dt;


	W_left_wheel_error_Previous=w_left_error;
	W_right_wheel_error_Previous=w_right_error;

	W_left_wheel_error_I=W_left_wheel_error_I+w_left_error*dt;
	W_right_wheel_error_I=W_right_wheel_error_I+w_right_error*dt;

if(w_left_d>0){
	left_PWM=KwI*W_left_wheel_error_I+KwP*w_left_error+KwD*w_left_error_D+21000;
	if(left_PWM>27000)left_PWM=27000;
    }
else if (w_left_d<0){
	left_PWM=KwI*W_left_wheel_error_I+KwP*w_left_error+KwD*w_left_error_D+12000;
	if(left_PWM<6000)left_PWM=6000;
	}
else {
	left_PWM=16384;
	}

if(w_right_d>0){
	right_PWM=-KwI*W_right_wheel_error_I-KwP*w_right_error-KwD*w_right_error_D+12000;
	if(right_PWM<6000)right_PWM=6000;
	}
else if(w_right_d<0){
	right_PWM=-KwI*W_right_wheel_error_I-KwP*w_right_error-KwD*w_right_error_D+21000;
	if(right_PWM>27000)right_PWM=27000;
	}
else{
	right_PWM=16384;
    }



}
//-------------------------------------------------------------------(5)
//					wheel filter
//----------------------------------------------------------------------
void wheel_filter(int diffEncoder1,int diffEncoder2,int W_wheel_1,int W_wheel_2){

	Wheel_1_velocity[3]=Wheel_1_velocity[2];
	Wheel_1_velocity[2]=Wheel_1_velocity[1];
	Wheel_1_velocity[1]=W_wheel_1;

	W_wheel_1 =(Wheel_1_velocity[1]+Wheel_1_velocity[2]+Wheel_1_velocity[3])/3;

	Wheel_2_velocity[3]=Wheel_2_velocity[2];
	Wheel_2_velocity[2]=Wheel_2_velocity[1];
	Wheel_2_velocity[1]=W_wheel_2 ;

	W_wheel_2=(Wheel_2_velocity[1]+Wheel_2_velocity[2]+Wheel_2_velocity[3])/3;
	//******************************************************************
		w_pre_left_wheel=w_left_wheel;
		w_pre_right_wheel=w_right_wheel;

		w_left_wheel=W_wheel_1;
		w_right_wheel=W_wheel_2;
	//******************************************************************
	int D_encorder1_predicted=w_pre_left_wheel*dt;
	int D_encorder2_predicted=w_pre_right_wheel*dt;
	//******************************************************************


	if(((abs(diffEncoder1-D_encorder1))>0)||((abs(D_encorder1_predicted-D_encorder1))>0)){
	  K_e_l=pow((diffEncoder1-D_encorder1),2.0)/(pow((diffEncoder1-D_encorder1),2.0)+pow((D_encorder1_predicted-D_encorder1),2.0));
	}

	if(((abs(diffEncoder2-D_encorder2))>0)||((abs(D_encorder2_predicted-D_encorder2))>0)){
	  K_e_r=pow((diffEncoder2-D_encorder2),2.0)/(pow((diffEncoder2-D_encorder2),2.0)+pow((D_encorder2_predicted-D_encorder2),2.0));
	}

	D_encorder1=(1-K_e_l)*diffEncoder1+K_e_l*D_encorder1_predicted;
	D_encorder2=(1-K_e_r)*diffEncoder2+K_e_r*D_encorder2_predicted;

	gvariable0=W_wheel_1;
	gvariable1=diffEncoder1;
	gvariable2=D_encorder1;


}
//-------------------------------------------------------------------(6)
//					Update Parameters
//-------------------------------------------------------
double update_parameters()
{
			Vx = (x - x_1)/dt;
			Vy = (y - y_1)/dt;
			Vz = deltaTheta;//(theta - theta_1)/dt;
			robotVx=deltaD/dt;

	x_1			= x;
	y_1			= y;
	theta_1		= theta;

}

// -------------------------------------------------------------------(6)
//					ComputeSteering
// ------------------------------------------------------
void ComputeSteering(int tempEncoder1,int tempEncoder2,int wheel_1_Vel,int wheel_2_Vel,int tempEncoderDir1,int tempEncoderDir2)
{
	loopcount++;
	if(loopcount>10000)loopcount=10000;


	tempEncoderDir2		= -1*tempEncoderDir2;		//To get the same direction as encoder2

	int diffEncoder11	= EliminateRollover(tempEncoder1,preEncoder1,tempEncoderDir1);
	int diffEncoder22	= EliminateRollover(tempEncoder2,preEncoder2,tempEncoderDir2);

	encoder1		= tempEncoder1;
	encoder2		= tempEncoder2;
//---------------------------------------------------------

	if(loopcount>5){
	encoderLeft=encoderLeft+diffEncoder11;
	encoderRight=encoderRight+diffEncoder22;

	encoderDir_L= tempEncoderDir1;
	encoderDir_R=tempEncoderDir2;

	encoder_difference= encoderLeft-encoderRight;


	wheel_filter(diffEncoder11,-diffEncoder22,tempEncoderDir1*wheel_1_Vel,tempEncoderDir2*wheel_2_Vel);

	deltaD		= GetDeltaDisplacement(D_encorder1,D_encorder2);
	deltaTheta	= GetDeltaAngle(D_encorder1,D_encorder2);

	control_wheel_velocity(w_left*375/(2*M_PI),w_right*375/(2*M_PI),w_left_wheel,w_right_wheel);

		}

	preEncoder1		= tempEncoder1;
	preEncoder2		= tempEncoder2;

//-----------------------------------------time calculation
	current_time = ros::Time::now();
	dt = (current_time - last_time).toSec();
	last_time	= current_time;
//---------------------------------------------------------


	if(initial<4)
	{
		deltaD		= 0;
		deltaTheta	= 0;

		initial++;
	}

	rotational_theta =rotational_theta+ deltaTheta;

	if(rotational_theta>=0){
	theta=rotational_theta+pi;
	theta=fmod(theta, 2*pi)-pi;
	}

	else {
		theta			=rotational_theta-pi;
		theta=fmod(theta, 2*pi)+pi;
	}

	if(deltaTheta == 0)
	{
		x				= x_1 + deltaD*cos(theta);
		y				= y_1 + deltaD*sin(theta);
	}
	else
	{
		x				= x_1 + deltaD*cos(theta + deltaTheta/2)*sin(deltaTheta/2)/(deltaTheta/2);
		y				= y_1 + deltaD*sin(theta + deltaTheta/2)*sin(deltaTheta/2)/(deltaTheta/2);
	}

//--------------------------------------------------------------------------------------------------------

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

			//set the velocity

			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = robotVx;
			odom.twist.twist.linear.y = 0;
			odom.twist.twist.angular.z = Vz;

			//publish the message
			odom_pub_.publish(odom);

			ROS_DEBUG("Current pose: x=%f, y=%f, theta=%f\n",x,y,theta*180/3.14);

	scanpose[0]=x;
	scanpose[1]=y;
	scanpose[2]=theta;
	scanpose[3]=D_encorder1;
	scanpose[4]=D_encorder2;


publish_encorder();
publish_Rscanpose();
}


private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;
    DrRobotMotionSensorDriver* drrobotPowerDriver_;
    struct DrRobotMotionConfig robotConfig1_;
    struct DrRobotMotionConfig robotConfig2_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct RangeSensorData rangeSensorData_;
    struct PowerSensorData powerSensorData_;
    struct StandardSensorData standardSensorData_;
    struct CustomSensorData customSensorData_;

	tf::TransformBroadcaster broadcaster_;

    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    bool enable_ir_;
    bool enable_sonar_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;


    int cntNum_;
    double ad2Dis(int adValue)
    {
      double temp = 0;
      double irad2Dis = 0;

      if (adValue <= 0)
        temp = -1;
      else
        temp = 21.6 /((double)adValue * 3 /4096 - 0.17);

      if ( (temp > 80) || (temp < 0))
      {
        irad2Dis = 0.81;
      }
      else if( (temp < 10) && (temp > 0))
      {
        irad2Dis = 0.09;
      }
      else
        irad2Dis = temp /100;
      return irad2Dis;
    }
};

//======================================================================MAIN=========================

int main(int argc, char** argv)
{



    ros::init(argc, argv, "drrobot_jaguar4x4_player");

    DrRobotPlayerNode drrobotPlayer;
    ros::NodeHandle n;

    // Start up the robot
    if (drrobotPlayer.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

    ros::Rate loop_rate(10);      //10Hz


    while (n.ok())
    {
      drrobotPlayer.doUpdate();
      ROS_INFO("Gayan Package");
      drrobotPlayer.sendTransform();
      ros::spinOnce();
     loop_rate.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    drrobotPlayer.stop();

    return(0);
}
