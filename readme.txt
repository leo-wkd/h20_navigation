This package employs UST-20LX Lidar to perceive the external environment and uses rviz to set some target points in the two-dimensional map created by gmapping package. Robot can autonomously navigate to the artificially set target points and avoid colliding with barriers on its path.

The dateset_maker and dataset_reader package are used for collecting data from Lidar and reading such collected data.Current path to save the dataset is /home/leo/h20_ws/dataset_laser.txt. You can change it at /dataset_maker/laser_listener.cpp and /dataset_reader/reader.cpp

Platform: Ubuntu16.04 Ros-kinetic H20-Robot UST-20LX Lidar

/*************************************************************/
To install package:
1. Step up catkin workspace
2. clone/download into catkin workspace
3. Run catkin_make
/*************************************************************/

Steps to run the program

/**************************
Edit the necessary files
1. Edit the /dataset_reader/reader.cpp and /dataset_maker/laser_listener.cpp to the path on your PC
2. Edit the h20.sh to the path on your PC
3. Edit the /urg_node/launch/urg_lidar.launch based on the imformation of your Lidar
eg. The ip_address of my IP-connected Lidar is 192.168.0.10
4. Edit all the yaml files in /Robotmovement according to your PC.
eg.
drrobot_player: {RobotCommMethod: Network, RobotID: drrobot1, RobotBaseIP: 192.168.0.73,
  RobotPortNum: 11311, RobotSerialPort: /dev/ttyS0, RobotType: Hawk_H20, MotorDir: 1, WheelRadius: 0.084, WheelDistance: 0.486,
  EncoderCircleCnt: 375, MinSpeed: 0.1, MaxSpeed: 1.0, Enable_IR: true, Enable_US: true}

You can get the imformation by running roscore and running ifconfig 

/*************************
Connect H20 Robot with your PC
Since you have to run the whole program on your PC and control the robot remotely, you have to set your computer as Host and set the processor on H20 as Slave
1. $ ifconfig           
run this command both on PC and H20-robot to see the ip address of PC and H20-Robot
2. $ hostname
run this command both on PC and H20-robot to see the hostname of PC and H20-Robot
3. $ sudo gedit /etc/hosts
run this command to modify the hosts file both on PC and H20-robot

For Host PC
127.0.0.1     localhost
127.0.1.1     hostname_PC

IP_A            hostname_PC
IP_B            hostname_H20-Robot

For Slave H20-Robot
127.0.0.1     localhost
127.0.1.1     hostname_H20-Robot

IP_B            hostname_H20-Robot
IP_A            hostname_PC

4. $ sudo gedit ~/.bashrc
run this command both on PC and H20-robot to change the path of ROS_MASTER
add this at the end of bashrc both on PC and H20-robot and be sure to change hostname_PC into your own hostname
export ROS_MASTER_URI=http://hostname_PC:11311

/*************************
Run the whole system
1. $ chmod +x h20.sh
2. $ ./h20.sh
3. If you run it successfully,  you can add on rviz about topics 
/Map                            to see the map created by Lidar
/Odometry                       to see the direction of the Robot 
/InteractiveMarkers             to set target for the robot by dragging the green cylinder


/*************************************************************/
If you fail to connect to or drive the Robot, try https://github.com/gayanbrahmanage/drrobot_jaguar4x4_player first. Once you can use joysticks to run the Robot, it is easier to run this package.

If you meet some other difficulties and cannot figure out the solutions, don't hesitate to contact me at leo97wang@gmail.com or find useful imformation on my blog https://blog.csdn.net/qq_42037180































