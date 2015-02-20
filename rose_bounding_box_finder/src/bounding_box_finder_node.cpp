/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Kris Piters
*	Date  : 2014/04/07
* 		- File created.
*
* Description:
*	Main function to set up ros node
* 
***********************************************************************************/

#include "rose_bounding_box_finder/bounding_box_finder_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
  ros::init(argc, argv, "bounding_box_finder");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, 10);
  private_node_handle_.param("topic", topic, string("bounding_box_finder"));

  // Create a new ScriptInteractionNode object.
  BoundingBoxFinder* bounding_box_finder = new BoundingBoxFinder("bounding_box_finder", n);
  
  //dynamic_reconfigure::Server<rose_bounding_box_finder::bounding_box_finderConfig> server_;
  //dynamic_reconfigure::Server<rose_bounding_box_finder::bounding_box_finderConfig>::CallbackType f;

  //f = boost::bind(&BoundingBoxFinder::CB_dynamicReconfigure, bounding_box_finder, _1, _2);
  //server_.setCallback(f);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  delete bounding_box_finder;

  return 0;
}
