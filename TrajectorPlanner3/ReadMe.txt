Author:Thomas Vy
Email: thomas.vy@ucalgary.ca
Date: June 12, 2018

Credit to Gayan Brahmanage for the formulation of the path planner first in his python code. Thanks Gayan!

This package requires that you have ros kinetic, gmapping, rviz, drrobot_jaguar4x4_player(git@github.com:ThomasVy/drrobot_jaguar4x4_player.git), and tf packages

how to install and run:
	1. clone trajectory_planner to your catkin_workspace
	2. run "catkin_make" in the command line
	3. drrobot_jaguar4x4_player should be running (I suggest running "roslaunch drrobot_jaguar4x4_player  			   H20base_player_basic.launch")
	4. gmapping should be running (I suggest running "roslaunch drrobot_jaguar4x4_player H20gmap.launch")
	3. type in "roslaunch trajectory_planner trajectory.launch"
	4. rviz should pop up and you will be able to drag the goal to any location on the map and path will be created

The trajectory planner will determine the path from the current position of the robot to the goal mark placed by the user. If a path can be create, it will spit out the shortest path available to the topic "/path" and spit out "Found Path" to terminal, otherwise it will print "Could Not Find Path" in terminal. If the goal marker is placed in an unknown area, the program will try to find the closest possible unknown spot(relative to the goal) in map.

Change "HITBOX" constant in "include/trajectory_planner/Constants.hpp" to change the hitbox of the robot.

Change "LENGTH" constant in "include/trajectory_planner/Constants.hpp" to change the distance of the points in the path.

Change "CURVATURE" constant in " include/trajectory_planner/Constants.hpp" to change the amount the path can turn.

Change "RESOLUTION" constant in "include/trajectory_planner/Constants.hpp" to change the "cost_map" calculations gradient

Change "LOOKAHEAD" constant in "include/trajectory_planner/Constants.hpp" to change the look ahead distance from the goal and the final point on the path planner 	

	

	


Implementation:
	The implmentation of the trajectory planner is created by it recieving an occupancy grid given by gmapping over ros and turning the occupancy grid into a 2d matrix. The 2d matrix is then converted to a map that contains values for empty space, walls, and unknown parameters. The trajectory planner then uses the converted map to make a cost map. The planning of the map starts with the current position of the robot as the start and the goal on the cost map (These can be any points as you desire as long as the points are in an empty space). The planner then uses an A* algorithm to plan for the path of the robot. The planner starts with the robot's current position then calculates the next valid positions(straight, left curve, and right curve) and calculates their respective costs and places them in a priority queue. The planner then places the old position into a vector and grabs the next lowest cost from the priority queue and tries to find valid the neighbours for that position. If the priority queue becomes empty, then there is no valid path to the goal. The planner continues that process until the current position to goal is less than 5 cells. Then we grab the point that found its way to the goal first and back track it. The path is then published with the correct information along with the transform of the path to the map. 

	
