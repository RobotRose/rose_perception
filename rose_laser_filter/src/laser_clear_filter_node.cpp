/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke
*	Date  : 8-11-2013
* 		- File created.
*
* Description:
*	Node for the laser clearing
* 
***********************************************************************************/

#include <ros/ros.h>
#include "rose_laser_filter/laser_clear_filter.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "laser_clear_filter");
	ros::NodeHandle n;
	ros::Rate r(30);

	// Create laser_clear_filter object
	LaserClearFilter laser_clear_filter(n);
	
	while(n.ok())
	{
		laser_clear_filter.process();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

