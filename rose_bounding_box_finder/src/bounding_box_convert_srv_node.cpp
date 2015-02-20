/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Kris Piters
*	Date  : 2014/04/25
* 		- File created.
*
* Description:
*	Main function to set up ros node
* 
***********************************************************************************/

#include "bounding_box_convert_srv_node.hpp"

int main( int argc, char **argv )
{

    // Set up ROS.
    ros::init(argc, argv, "bounding_box_convert_srv");
    
    BoundingBoxConvertSrv* bounding_box_convert_srv = new BoundingBoxConvertSrv(ros::this_node::getName());
    
    ros::spin();

    delete bounding_box_convert_srv;

    return 0;
}