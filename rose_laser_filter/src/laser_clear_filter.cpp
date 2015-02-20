/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke
*	Date  : 8-11-2013
* 		- File created.
*
* Description:
*	Node to filter laser messages in order to clear the costmap
* 
***********************************************************************************/

#include "laser_filter/laser_clear_filter.hpp"

LaserClearFilter::LaserClearFilter(ros::NodeHandle n)
{
	n_ 							= n;
	data_filtered_ 				= true;
	sub_laser_scan_				= n_.subscribe("/scan", 10, &LaserClearFilter::CB_scanReceived, this);
	pub_laser_scan_filtered_ 	= n_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 10);
	ROS_INFO("LaserClearFilter initialized.");
}

void LaserClearFilter::CB_scanReceived(const sensor_msgs::LaserScan& input_scan)
{
	if(data_filtered_ == true)
	{
		input_scan_ 	= input_scan;
		data_filtered_ 	= false;
		ROS_DEBUG_NAMED("LaserClearFilter", "New input_scan");
	}	
	else
		ROS_DEBUG_NAMED("LaserClearFilter", "New scan received, but still filtering previous scan.");
	
}

void LaserClearFilter::process()
{
	if(!data_filtered_)
		filter_scan();
}

void LaserClearFilter::filter_scan()
{
	filtered_scan_ = input_scan_;
	for (unsigned int i=0; i < input_scan_.ranges.size(); i++) // Need to check ever reading in the current scan
    {
      //ROS_INFO("[%d]: %.3f", i, filtered_scan_.ranges[i]);
      if (filtered_scan_.ranges[i] >= filtered_scan_.range_max || filtered_scan_.ranges[i] == 0.0)                         
      {      
      	
        filtered_scan_.ranges[i] = filtered_scan_.range_max - 0.01;    // If so, then make it a value bigger a little smaller than the max range
      }
    }
    data_filtered_ = true;

    publish_filtered_scan();
}


void LaserClearFilter::publish_filtered_scan()
{
	pub_laser_scan_filtered_.publish(filtered_scan_);
}
