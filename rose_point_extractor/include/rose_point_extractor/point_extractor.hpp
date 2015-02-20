/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/08/22
* 		- File created.
*
* Description:
*	  This class provides a service that extracts a 3D-point from a selection in a point 
*   cloud. This selection is made in the 2D camera view of the camera.
* 
***********************************************************************************/
#ifndef POINT_EXTRACTOR_HPP
#define POINT_EXTRACTOR_HPP

#include <ros/ros.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rose_point_extractor/get_point.h"

#include "rose_common/common.hpp"

#define CAM_INFO_TOPIC      "/camera/rgb/camera_info"

/**
 * Class to provide the point extractor service
 */
class PointExtractor
{
  public:
    /**
     * Constructor
     */
	PointExtractor( std::string name, ros::NodeHandle n );

    /**
     * Destructor
     */
	~PointExtractor();
  
  private:
    /**
     * This function checks if a selection of points is correct.
     * x_max should be greater than x_min and the same for y.
     * @param  min_x Lower x-coordinate
     * @param  max_x Higher x-coordinate
     * @param  min_y Lower y-coordinate
     * @param  max_y Higher y-coordinate
     * @return       If the points are correct.
     */
  	bool validRectangle( const double min_x, const double max_x, 
						 const double min_y, const double max_y );

    /**
     * Gets the point cloud in a 2D rectangle.
     * It filters the current point cloud on the points that are inside the selected 
     * rectangle (min_x, max_x, min_y, max_y)   
     * @param  min_x           Lower x-coordinate
     * @param  max_x           Higher x-coordinate
     * @param  min_y           Lower y-coordinate
     * @param  max_y           Higher y-coordinate
     * @param  pcl_point_cloud The full point cloud
     * @param  object_cloud    The resulting selected point cloud
     * @return                 If the function was successful
     */
    bool getSelectedPointsInRectangle( const double min_x, const double max_x, 
                                       const double min_y, const double max_y, 
                                       const pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud, 
                                             pcl::PointCloud<pcl::PointXYZ>& object_cloud );

    /**
     * Filter outliers from the point cloud
     * @param  cloud The cloud to be filtered
     * @return       If the function was successful
     */
    bool filterOnOutliers( pcl::PointCloud<pcl::PointXYZ>& cloud );

    /**
     * Callback for the service to extract a the centroid from a selection of a point cloud
     * @param  req Request message (contains min_x, max_x, min_y, max_y)
     * @param  res Result message (coints PointStamped)
     * @return     If the function was successful
     */
  	bool CB_extract_point ( rose_point_extractor::get_point::Request  &req,	
							rose_point_extractor::get_point::Response &res);

    /**
     * Callback when a new point cloud is available
     * @param msg Pointcloud message
     */
  	void CB_new_point_cloud( const sensor_msgs::PointCloud2::ConstPtr& msg );

    void CB_camera_info_topic( const sensor_msgs::CameraInfo::ConstPtr camera_info_msg );

    void publishSelection( pcl::PointCloud<pcl::PointXYZ> selection );

  	ros::NodeHandle 	n_;        //!< NodeHandle
	std::string 		name_;     //!< Name of the node

    bool                point_cloud_initialized_; //!< Boolean whether a point cloud is stored. 
                                                  //!< Otherwise we cannot we any operation.
                                                  
    image_geometry::PinholeCameraModel cam_model_;

    int camera_height_;
    int camera_width_;

  	ros::ServiceServer  extract_service_;   //!< Service to extract the a point from a point cloud
    ros::Subscriber     point_cloud_sub_;   //!< Point cloud subscriber
    ros::Subscriber     camera_info_sub_;   //!< Point cloud subscriber

    ros::Publisher      selected_point_cloud_pub_; 

  	sensor_msgs::PointCloud2 point_cloud_;  //!< The last point cloud that was available
};

#endif //POINT_EXTRACTOR_HPP