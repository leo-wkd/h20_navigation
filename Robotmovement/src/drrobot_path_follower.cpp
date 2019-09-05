/*

Based on a path published by Trajectory_node (Thomas Vy), this node sends commands to a DrRobot driver program to follow a path.
It also converts motor information from the same driver to odometry that can be sent to GMapping.

Author: Noam Anglo; Thomas Vy
Date: August 2018
Email: noam.anglo@ucalgary.ca; thomas.vy@ucalgary.ca

*/

//-----------------------------------------Parameters and headers----------------------------------------------------------//

//ROS
#include "ros/ros.h"

//Odometry information
#include <drrobot_jaguar4x4_player/MotorInfoArray.h>
//Message types
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sstream>
#include <iostream>


#define WHEELRAD         0.083              //Radius of the wheel
#define WHEELDIST        0.42               //Distance between the two wheels
#define WHEELWIDTH       0.036              //Width of individual wheels

#define LOOKAHEAD        0.17               //Distance from current goal before robot changes to next goal

#define ENCODERCOUNT     376                //Number of encoder steps per full revolution
#define MAXENCODER       32767              //Maximum encoder value
#define ENCROLL          1000               //Encoder rollover threshold (just set it to something high)
//-------------------------------------Node Class--------------------------------------------------------------------------//

class TrajectoryPlotterNode
{
  private: //private structures
    //2D Pose struct
     typedef struct Pose2D
     {
       double x;
       double y;
       double theta;

       //Constructors
       Pose2D():x(0), y(0), theta(0){}
       Pose2D(double a, double b): x(a), y(b), theta(0) {}
       Pose2D(double a, double b, double c): x(a), y(b), theta(c) {}
       Pose2D(const Pose2D & rhs): x(rhs.x), y(rhs.y), theta(rhs.theta){}
       Pose2D(Pose2D && rhs): x(rhs.x), y(rhs.y), theta(rhs.theta){}
       //add a difference checker.
       //Overloading assignment operator
       Pose2D & operator=(Pose2D & rhs){
         if(this != &rhs)
         {
           x = rhs.x;
           y = rhs.y;
           theta = rhs.theta;
         }
         return *this;
       }
       Pose2D & operator=(Pose2D && rhs){
         if(this != &rhs)
         {
           x = rhs.x;
           y = rhs.y;
           theta = rhs.theta;
         }
         return *this;
       }
       bool checkSamePosition (Pose2D & rhs)    //Checks if previous position is the same as the current position - i.e. if robot stalled
       {
         if(fabs(x-rhs.x)>0.001)
          return false;
         if(fabs(y-rhs.y)>0.001)
          return false;
         if(fabs(theta-rhs.theta)>0.01)
          return false;
        return true;
       }
     }Pose2D;

     //Encoder values
     typedef struct Encoder
     {
       int left;
       int right;

       //Constructor
       Encoder():left(0), right(0){}
       Encoder(int a, int b): left(a), right(b){}

     }Encoder;

     //2D Twist struct, for differential drive
     typedef struct Twist2D
     {
       double linear;
       double angular;

       //Constructors
       Twist2D():linear(0), angular(0){}
       Twist2D(Twist2D & rhs)
       {
         linear = rhs.linear;
         angular = rhs.angular;
       }
       //Overloading assignment operator
       Twist2D & operator=(Twist2D & rhs)
       {
        if(this != &rhs)
        {
          linear = rhs.linear;
          angular = rhs.angular;
        }
        return *this;
       }
     }Twist2D;

  private: //attributes
  //--------------------------------------Initializing stuff-------------------------------------------------------------//
    ros::NodeHandle n;
    ros::Publisher velocity_pub;
    ros::Publisher odom_pub;
    ros::Publisher marker_pub;
    ros::Publisher goal_pub;
    ros::Subscriber goal_sub;
    ros::Subscriber path_sub;
    ros::Subscriber MotorInfo_sub;
    ros::Timer timer;
    bool pathRecieved;
    bool ignorePath;
    std::vector<Pose2D> path;
    Pose2D curPose;                          //Current position
    Pose2D pastcurPose;
    Pose2D GoalPose;                         //Goal position
    std::vector<Pose2D>::iterator it;        //Iterator
    ros::Time currentTime = ros::Time::now(); //Time
    tf::TransformListener tflistener;
    geometry_msgs::Pose goalOfFinal;

    //Parameters
    double speedMax;
    double speedInterval;
    double angleP;
    double angleI;
    double angleD;
    double maxIVal;

  private: // private methods
  //--------------------------------------Geometry Functions---------------------------------------------------//

    //Returns scalar length from A to B
    double lengthAB(Pose2D & A, Pose2D & B)
    {
      return hypot((B.x-A.x), (B.y-A.y));
    }

    //Returns shortest difference in angle between current pose vs. angle from A to B
    double angleAB(Pose2D & A, Pose2D & B)
    {
        double goalangle = atan2((B.y-A.y),(B.x-A.x));
    if(goalangle<0)
			goalangle += 2*M_PI;
        double onedir = goalangle - A.theta;
    if(onedir >M_PI)
			onedir -= 2*M_PI;
		else if (onedir<-M_PI)
			onedir += 2*M_PI;
        return onedir;
    }
    //--------------------------------------Movement Function-------------------------------------------------------//
	Pose2D getCurrentPosition()
   {
        tf::StampedTransform transform;
        while(true){
                try{
                        tflistener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
                        break;
                }
                catch(tf::TransformException t)
                {}
        }
        Pose2D currentPosition(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation()));
        return currentPosition;
   }

    //Movement command publisher from A to B using 'Follow the Carrot' algorithm

    //Movement command publisher from A to B using 'Follow the Carrot' algorithm, using PI control
    void cmd_velPublisher(Pose2D & B)
      {
        if(pathRecieved == true && ignorePath == false)
        {
            markerPub(B.x, B.y);
            Pose2D A = getCurrentPosition();
            static geometry_msgs::Twist msg;
            //ROS_INFO("Distance from GoalPose - [%f], GoalPose Coordinates - X: [%f], Y: [%f]", lengthAB(A, B), B.x, B.y);
            if(lengthAB(A, B)>LOOKAHEAD) //If the look-ahead distance hasn't been reached...
            {
              //A
              double angle = angleAB(A, B);
              double length = lengthAB (A,B);
              ROS_INFO("Angle Difference:[%f]: ", angle);

                if (fabs(angle)>M_PI/4)           //If the angle is greater than 45 degrees, rotate but don't move forward
                {
                   ROS_INFO("Rotating");
                   //lowers noise by damping deceleration a bit when previous velocity is above ~0
                   if (msg.linear.x > 0.0001)
                    {msg.linear.x -= speedMax/speedInterval;}
                   else
                    {msg.linear.x = 0;}

                    //PID-controlled angular velocity
                    msg.angular.z = PIDVel(angle, angleP, angleI, angleD, maxIVal);
                }

                else                                      //Else, rotate and move forward
                {
                 ROS_INFO("Forward and rotating");

                 //lowers noise by damping acceleration a bit when velocity is below 0.06 maximum
                 if (msg.linear.x < speedMax)
                  {msg.linear.x += speedMax/speedInterval;}

                 else
                  {msg.linear.x = speedMax;}

                  //PID-controlled angular velocity
                  msg.angular.z = PIDVel(angle, angleP, angleI, angleD, maxIVal);
                }
              }
            else                         //Else...
            {
              pastcurPose = curPose;
              if(it!=path.end())  //If you're not at the end of the path, go the next point
              {
                GoalPose = (*it);
                it++;
              }
              else                //Else, stop
              {
                ROS_INFO("Goal Reached");
                path.clear();
                pathRecieved = false;
                msg.angular.z = 0;
                msg.linear.x = 0;
              }
            }
            ROS_INFO("Velocity message sent: [%f], [%f]", msg.linear.x, msg.angular.z);
            velocity_pub.publish(msg);
      }
    }

    //--------------------------------------Encoder Conversion Functions---------------------------------------------------//

      //Eliminates rollover
      int RollRemover(int encoderValue)
      {
        if (abs(encoderValue)>=ENCROLL) //If the encoder value is larger than the rollover threshold...
        {
          if (encoderValue < 0)               //If the encoder value is negative, add it to the max. encoder
            return MAXENCODER+encoderValue;
          else                                //Else, subtract the max. encoder from the encoder value
            return -MAXENCODER+encoderValue;
        }
        else                           //Else, just return it as it is
          return encoderValue;
      }

      //Returns conversion from encoder odometry to values in meters
      double EncToMeters(int EncoderVal)
      {
        return EncoderVal*2*M_PI*(WHEELRAD)/(ENCODERCOUNT);
      }

      //Takes encoder movement and converts it to forward + angular rotation (i.e. 2D twist)
      Twist2D EncToTwist(Encoder & dEncoder)
      {
        Twist2D vel;

        vel.linear = (EncToMeters(dEncoder.right)+EncToMeters(dEncoder.left))/2;
        vel.angular = (EncToMeters(dEncoder.right)-EncToMeters(dEncoder.left))/WHEELDIST;

           ////ROS_INFO("Twist Velocity: [%f] cm/s, [%f] degrees", vel.linear*100, vel.angular*(180/M_PI));
        return vel;
      }

      //Adds Twist to a current pose
      void AddTwistTo(Pose2D & Pose, Twist2D & Twist)
      {
        Pose.x += Twist.linear*cos(Pose.theta);
        Pose.y += Twist.linear*sin(Pose.theta);
        Pose.theta += Twist.angular;
        if (Pose.theta < 0)            //Keeps rotational pose positive
          Pose.theta += 2*M_PI;

        else if (Pose.theta >= 2*M_PI) //Keeps rotational pose below 360 degrees
           Pose.theta -= 2*M_PI;

           ////ROS_INFO("curPose - X:[%f], Y:[%f], Theta:[%f]", Pose.x, Pose.y, Pose.theta);
      }

  //-------------------------------------------Transform functions---------------------------------------------------------//
      //Publishes odometry to topic "odom"
      void odomPublisher(Twist2D && twistVel)
    {
      static tf::TransformBroadcaster odom_broadcaster; //The broadcaster for the odometry->baselink transform
      currentTime = ros::Time::now();
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = currentTime;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = curPose.x;
      odom_trans.transform.translation.y = curPose.y;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(curPose.theta);
      odom_broadcaster.sendTransform(odom_trans);

      //Publishing the odometry message
      nav_msgs::Odometry odom;
      odom.header.stamp = currentTime;
      odom.header.frame_id = "odom";

      //Position
      odom.pose.pose.position.x = curPose.x;
      odom.pose.pose.position.y = curPose.y;
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(curPose.theta);

      //Velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = twistVel.linear;;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = twistVel.angular;

      odom_pub.publish(odom);
      AddTwistTo(curPose, twistVel);
      sendTransform();
    }
    void sendTransform ()
    {
      static tf::TransformBroadcaster broadcaster;
      broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.03,0,0.1)), currentTime, "base_link", "laser"));
      broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)), currentTime, "laser", "scan"));

    }

  public:
    TrajectoryPlotterNode(){
      pathRecieved =false;
      ignorePath = false;
      timer = n.createTimer(ros::Duration(5.0), boost::bind(&TrajectoryPlotterNode::timerCallback, this, _1));
      velocity_pub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 10);
      odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
      marker_pub = n.advertise<visualization_msgs::Marker>("goalmark", 10);
      path_sub = n.subscribe<nav_msgs::Path>("path", 1, boost::bind(&TrajectoryPlotterNode::pathCallback, this, _1));
      goal_pub = n.advertise<geometry_msgs::Pose>("goal_post", 10);
      goal_sub = n.subscribe<geometry_msgs::Pose>("goal_post", 10, boost::bind(&TrajectoryPlotterNode::goalCallback, this, _1));
      MotorInfo_sub = n.subscribe<drrobot_jaguar4x4_player::MotorInfoArray>("drrobot_motor", 1, boost::bind(&TrajectoryPlotterNode::motorCallback, this, _1));

      //PID parameters

      ros::NodeHandle pnh("~");

      pnh.param<double>("/speedMax", speedMax, 0.08);
      pnh.param<double>("/speedInterval", 64);
      pnh.param<double>("/angularP", angleP, 0.6);
      pnh.param<double>("/angularI", angleI, 0.005);
      pnh.param<double>("/angularD", angleD, 10);
      pnh.param<double>("/maxIVal", maxIVal, M_PI/5);

      ////ROS_INFO("PID parameters received: [%f, %f, %f, %f]", angleP, angleI, angleD, maxIVal);
    }
  //--------------------------------------Callback and publisher functions-------------------------------------------------//
  //Publishes a marker based on x and y
  void markerPub(const double & x, const double & y)
  {
    static int i = 0;

    //Initializing stuff
    visualization_msgs::Marker mark;
    mark.header.frame_id = "map";
    mark.header.stamp = ros::Time();
    mark.ns = "m_ns";
    mark.id = i;

    //Marker type
    mark.type = visualization_msgs::Marker::CUBE;
    mark.action = visualization_msgs::Marker::ADD;

    //Marker pose
    mark.pose.position.x = x;
    mark.pose.position.y = y;
    mark.pose.position.z = 0.05;

    //Marker orientation
    mark.pose.orientation.x = 0;
    mark.pose.orientation.y = 0;
    mark.pose.orientation.z = 0;
    mark.pose.orientation.w = 0;

    //Marker scale
    mark.scale.x = 0.05;
    mark.scale.y = 0.05;
    mark.scale.z = 0.1;

    //Marker alpha and color
    mark.color.a = 0.8;
    mark.color.r = 1.0;
    mark.color.g = 0;
    mark.color.b = 0.5;

    i++;

    //Publishes the marker to 'goalmark'
    marker_pub.publish(mark);
  }
  void goalCallback(const geometry_msgs::Pose::ConstPtr & goal)
  {
	goalOfFinal = *goal;
  }

  void timerCallback(const ros::TimerEvent& e)
 {
    static Pose2D currentPose;
    currentPose = getCurrentPosition();
    if(pathRecieved == true)
    {
      if(pastcurPose.checkSamePosition(currentPose))
      {
        ignorePath = true;
	path.clear();
        pathRecieved = false;
        geometry_msgs::Twist msg;
        ros::Time start = ros::Time::now();
        ros::Duration timeout(2.0);
        while(ros::Time::now() - start < timeout)
        {
          msg.linear.x =-0.1;
          velocity_pub.publish(msg);
        }
        msg.linear.x =0;
        msg.angular.z = M_PI/4;
        velocity_pub.publish(msg);
        ignorePath = false;
	goal_pub.publish(goalOfFinal);
      }
    }
    pastcurPose = currentPose;
  }
    //Callback function from "path"
    void pathCallback(const nav_msgs::Path::ConstPtr& pathmsg)
    {
	       path.clear();
          for(int i =0; i<pathmsg->poses.size();i++)
          {
            Pose2D temp;
            temp.x = pathmsg->poses[i].pose.position.x;
            temp.y = pathmsg->poses[i].pose.position.y;
            path.push_back(temp);
          }
          it = path.begin();
          if(it!=path.end())
          {//PID-controlled angular velocity
            GoalPose = (*it);
            it++;
          }
          cmd_velPublisher(GoalPose);
	         pathRecieved = true;
    }

    //Callback function for odometry from "drrobot_motor"; converts encoder movement to Twist as "twistVel"
    void motorCallback(const drrobot_jaguar4x4_player::MotorInfoArray::ConstPtr& odommsg)
    {
      //Declaring + storing previous encoder value and initializing delta
      static Encoder prev(odommsg->motorInfos[0].encoder_pos,odommsg->motorInfos[1].encoder_pos);
      Encoder Delta;

            ////ROS_INFO("Raw wheel movement: [%d] left, [%d] right", odommsg->motorInfos[0].encoder_pos, odommsg->motorInfos[1].encoder_pos);

      //Finding change in encoder position
      Delta.left = RollRemover((odommsg->motorInfos[0].encoder_pos-prev.left));
      Delta.right = -RollRemover((odommsg->motorInfos[1].encoder_pos-prev.right)); //Negative to account for right encoder being reversed

            ////ROS_INFO("Wheel movement: [%d] left, [%d] right", Delta.left, Delta.right);

      //Storing a new previous encoder value after doing calculations
      prev.left = odommsg->motorInfos[0].encoder_pos;
      prev.right = odommsg->motorInfos[1].encoder_pos;

      //Publish function for odometry; publishes to "odom"
      odomPublisher(EncToTwist(Delta));

      //Publish function for movement command; publishes to "drrobot_cmd_vel"
      cmd_velPublisher(GoalPose);
    }

//--------------------------------------PI Controller (thanks Wescott Design Services)-----------------------------------------//
    double PIDVel(const double & proErr, const double & Kp, const double & Ki, const double & Kd, const double & maxComm) //(Error, prop. gain, int. gain, deriv. gain, maximum allowable command value)
    {
      //Defining proportional term
      double pTerm = Kp*proErr;

      //Finding integral value
      static double intErr = 0;
      intErr += proErr;

      //Anti-windup
      static const double windupCoeff = 0.1;
      if (intErr > windupCoeff*maxComm/Ki)
        intErr = windupCoeff*maxComm/Ki;
      else if (intErr < -windupCoeff*maxComm/Ki)
        intErr = -windupCoeff*maxComm/Ki;

      //Defining integral term
      double iTerm = Ki*intErr;

      //Tsettle, a and previous error for bandlimited derivative term
      static const double Tsettle = 1;
      static const double sampleRate = 50;
      static double a = 1-1/(Tsettle*50);
      static double prevErr;

      //Defining the derivative term and previous error
      double dTerm = Kd*(1-a)*(proErr - prevErr);
      prevErr = a*prevErr+(1-a)*(proErr);

      ROS_INFO("pTerm: [%f], iTerm:[%f], dTerm: [%f]", pTerm, iTerm, dTerm);
      return (pTerm + iTerm);
    }
};
//--------------------------------------Main-------------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  //Initializing
  ros::init(argc, argv, "drrobot_trajectory_plotter");
  TrajectoryPlotterNode DrRobotPlotterNode;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
//By Noam and Thomas
