/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*  Author: Okke
*  Date  : 8-11-2013
*     - File created.
*
* Description:
*  Header file for the laser clearing filter
* 
***********************************************************************************/
#ifndef LASER_CLEAR_FILTER_HPP
#define LASER_CLEAR_FILTER_HPP

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

class LaserClearFilter
{
private:
  ros::NodeHandle           n_;
  ros::Subscriber           sub_laser_scan_;
  ros::Publisher            pub_laser_scan_filtered_;
  sensor_msgs::LaserScan    input_scan_;
  sensor_msgs::LaserScan    filtered_scan_;
  bool                      data_filtered_;

public:  

  LaserClearFilter(ros::NodeHandle n);

  ~LaserClearFilter(){}

  void CB_scanReceived(const sensor_msgs::LaserScan& input_scan);
  void process();
  void filter_scan();
  void publish_filtered_scan();
};

#endif // LASER_CLEAR_FILTER_HPP
