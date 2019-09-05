#ifndef TRAJECTCONST
#define TRAJECTCONST
#include <vector>
#include <queue>
#include <list>
#include <memory>
#include "geometry_msgs/PoseStamped.h"
/**
  The constants for the trajectory planner
  Author: Thomas Vy(thomas.vy@ucalgary.ca)
  Date: June 12, 2018
**/

//prototypes for classes and namespace
using namespace std;
class Image;
class Position;
struct Wall;
struct Pose;

//Constants
#define PRECISION  100 //The number of dots for the bspline curve
#define UNKNOWN -2 //The unknown space value
#define WALL -1 //The wall space value
#define WILLCOLIDE -3
#define EMPTY_SPACE 1 //The empty space value
#define RESOLUTION 4 //The distance from wall to calculate(for cost)
#define LENGTH 3//The distance between points(higher number = fast computation but inaccurate path)
#define CURVATURE 0.2 //The curvature the robot takes
#define HITBOX 9 // the hit box of the robot
#define LOOKAHEAD 1 //The distance from the goal and the final point on the path
//The typedefs
typedef vector< vector<double> > matrix; //Holds the map cells
typedef vector<geometry_msgs::PoseStamped> pathMessage;//The path message
typedef vector<Position> listOfPositions; //list of positions
typedef priority_queue<Position,vector<Position>, std::greater<Position> > positionPriorityQueue; //the priority queue
typedef vector<unique_ptr<Position> > vectorOfPointers; //vector of Pointer
typedef list<Position> positionLinkedList; //Linked list for position
typedef vector<Pose> poseVector; //vector of poses
typedef vector<double> doubleVector; //vector of doubles

//Wall class for wall pixels
typedef struct Wall{
	int x; //The x positon of the wall
	int y; //The y postion of the wall
	std::vector<bool> direction; // the free space direction
	Wall(int x, int y, vector<bool> direction):x(x), y(y), direction(direction){}; //constructor
}Wall;

//pose to hold the x, y, and angle of the postition
typedef struct Pose{
	double x; //The x postion
	double y; // y position
	double radian; // angle of the robot
	Pose():x(0),y(0),radian(0){}; //Constructor
	Pose(double x, double y, double radian):x(x),y(y),radian(radian) {}; //Constructor
	Pose(double x, double y):x(x),y(y),radian(0){}; //Constructor
	Pose(const Pose & rhs); //Copy Constructor
	Pose& operator=(const Pose & rhs); //Assignment operator
	bool operator==(const Pose &rhs); //Comparing operator
	Pose endPose(float curvature, float length); //Calculating end position for a path

}Pose;
//The position class for points on the map
class Position{
	friend class Image; //makes it so that Image can see Position's private elements
	private:
		float cost; //The cost of that position
		float total_cost; //The total cost of moving to that position
		Pose pose; //The pose of that spot
		Position * prePosition; //The previous spot of this position
	public:
		Position(Pose & pose, float total_cost, float cost, Position * prePosition);//constructor
		Position(Pose & pose, float cost, Position * prePosition);//constructor
		Position(const Position & rhs);//copy constructor
		Position():cost(0),total_cost(0){} //default constructor
		Position& operator=(const Position & rhs);//assignment operator
		bool operator>(const Position & right) const {return total_cost > right.total_cost;}// comparing for the priority queue
		listOfPositions getNeighbours (const matrix & walls, const Pose & goal);// gets the neighbours of this position


		listOfPositions lookForClosestUnknown (const matrix & walls, const Pose & goal, positionPriorityQueue & unknownQueue); //looks for neighbours(neighbours are free spaces and returning the neighbours. If an unknown is encountered then it is placed in the unknown queue with the distance away from goal.
		int checkNeighbour (const Pose & current, const Pose & next,const matrix & walls);// checks the neighbour if it is a valid spot(doesn't cross through a wall)
};

//Hold the image grids and do calculations for them
class Image{
	private:
		Pose first; //The beginning map edges
		Pose last; //The end map edges
		pathMessage path;// the message to be broadcasted
		matrix convertedImage;//the Wall, Unknown, and Empty space located
		matrix arena;// the cost grid
		void dilation(Wall & wall);//turns all the empty space cells next to a wall into a certain cost
	public:
		Image(const matrix & oriImage, const Pose & first, const Pose & last);//constructor. Turns the original image into the converted Image
		void insert_borders ();//creates the arena by locating all wall cells
		vector<bool> checkSpace (int i, int j);//checks spaces around a wall to see which direction is open space
		bool planner (Pose & start, Pose & goal);//plans the path for the map
		const pathMessage & getPath ();//gets the path message
		bool findNearestFreeSpace(Pose & finalGoal, Pose & start); //tries to find a path to the closest unknown to the goal
};
#endif
