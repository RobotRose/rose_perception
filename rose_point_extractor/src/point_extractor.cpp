/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/08/22
* 		- File created.
*
* Description:
*	This class provides a service that extracts a 3D-point from a selection in a point 
*	cloud. This selection is made in the 2D camera view of the camera.
*	 
***********************************************************************************/
#include "rose_point_extractor/point_extractor.hpp"

PointExtractor::PointExtractor( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
	, point_cloud_initialized_ ( false )
	, camera_height_ ( 480 )
	, camera_width_ ( 640 )
{
	extract_service_ = n_.advertiseService( name + "/get_point", &PointExtractor::CB_extract_point, this);

	camera_info_sub_ = n_.subscribe(CAM_INFO_TOPIC, 1, &PointExtractor::CB_camera_info_topic, this );
	// point_cloud_sub_ = n_.subscribe("/point_cloud/downsample", 5, &PointExtractor::CB_new_point_cloud, this);
	point_cloud_sub_ = n_.subscribe("/camera/depth_registered/points", 5, &PointExtractor::CB_new_point_cloud, this);

	selected_point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>( name + "/selected_points", 1);

	ROS_INFO_NAMED(ROS_NAME, "PointExtractor started");
}

PointExtractor::~PointExtractor()
{

}

void PointExtractor::CB_camera_info_topic( const sensor_msgs::CameraInfo::ConstPtr camera_info_msg )
{
	cam_model_.fromCameraInfo(camera_info_msg);
	camera_height_ = camera_info_msg->height;
	camera_width_  = camera_info_msg->width;
	ROS_DEBUG_NAMED(ROS_NAME, "Received new camera info, width: %d, height: %d", camera_width_, camera_height_);
}

void PointExtractor::CB_new_point_cloud( const sensor_msgs::PointCloud2::ConstPtr& msg )
{
	point_cloud_ 		= *msg;
	int cloud_width 	= point_cloud_.width;
	int cloud_height 	= point_cloud_.height;	

	point_cloud_initialized_ = true;
	ROS_DEBUG_NAMED(ROS_NAME, "Received new point cloud, width: %d, height: %d", cloud_width, cloud_height);
}

bool PointExtractor::validRectangle( const double x_low, const double x_high, 
									 const double y_low, const double y_high )
{
	//! @todo MdL: Can be made more generic: Flip if they are not correct. Or; Take absolute value in teh function
	//! 		   PointExtractor::getSelectedPointsInRectangle calculating the height/width.
	return ((x_high > x_low) && (y_high > y_low));
}

bool PointExtractor::getSelectedPointsInRectangle( const double min_x, const double	max_x, 
												   const double min_y, const double max_y, 
												   const pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud, 
												         pcl::PointCloud<pcl::PointXYZ>& object_cloud )
{
	double scale;
	int point_cloud_width;
	if ( pcl_point_cloud.height == 1 )
	{
		// Get point resolution
		scale =  std::sqrt((double)pcl_point_cloud.points.size() / (double)((double)camera_height_ * (double)camera_width_));

		// Convert from 1D to 2D
		point_cloud_width = camera_width_ * scale ;
		ROS_DEBUG_NAMED(ROS_NAME, "Original point cloud width: %d", point_cloud_width);
	}
	else
	{
		scale 			  = 1.0;
		point_cloud_width = pcl_point_cloud.width;
	}

	// Change min/max x/y to cope with scale
	int x_min = std::floor(min_x * scale);
	int x_max = std::floor(max_x * scale);
	int y_min = std::floor(min_y * scale);
	int y_max = std::floor(max_y * scale);

	ROS_DEBUG_NAMED(ROS_NAME, "Region (%d, %d, %d, %d)", x_min, x_max, y_min, y_max);

	// ROS_DEBUG_NAMED(ROS_NAME, "getSelectedPointsInRectangle");
	// ROS_DEBUG_NAMED(ROS_NAME, "pcl_point_cloud size: %d", (int)pcl_point_cloud.points.size());
	// ROS_DEBUG_NAMED(ROS_NAME, "pcl_point_cloud width: %d", pcl_point_cloud.width);
	int width 	= x_max - x_min; 
	int height 	= y_max - y_min;

	object_cloud.width 		= width;
	object_cloud.height 	= height;
	object_cloud.is_dense 	= false;
	object_cloud.points.resize (width * height);

	// ROS_DEBUG_NAMED(ROS_NAME, "object_cloud size: %d", (int)object_cloud.points.size());

	// Point cloud is a 1D array.
	// ROS_DEBUG_NAMED(ROS_NAME, "before loop");
	for(int i = 0; i < height; i++ )
	{				
		for(int j = 0; j < width; j++ )
		{
			// ROS_DEBUG_NAMED(ROS_NAME, "looping i:(%d/%d) j:(%d/%d)", i, height, j, width);
			int from_index = (point_cloud_width*(y_min + i) + (x_min + j));
			int to_index   = (width*i+j);
			// ROS_DEBUG_NAMED(ROS_NAME, "from %d to %d", from_index, to_index);
			object_cloud.points[to_index].x = pcl_point_cloud.points[from_index].x;
			object_cloud.points[to_index].y = pcl_point_cloud.points[from_index].y;
			object_cloud.points[to_index].z = pcl_point_cloud.points[from_index].z;
		}
	}

	publishSelection(object_cloud);

	ROS_DEBUG_NAMED(ROS_NAME, "getSelectedPointsInRectangle done");
	return true;
}

bool PointExtractor::filterOnOutliers( pcl::PointCloud<pcl::PointXYZ>& cloud )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Points in cloud before filtering: %d", (int)cloud.size());

  	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	// MakeShared, make deep copy of the point cloud. Otherwise the point cloud pointer gets deleted after this operation.
	sor.setInputCloud(cloud.makeShared());
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(cloud_filtered);

	cloud = cloud_filtered;

	ROS_DEBUG_NAMED(ROS_NAME, "Points in cloud after filtering: %d", (int)cloud.size());
	return true;
}

bool PointExtractor::CB_extract_point ( point_extractor::get_point::Request  &req,	
										point_extractor::get_point::Response &res )
{
	ROS_DEBUG_NAMED(ROS_NAME, "Request received");

	// No point cloud available
	if ( not point_cloud_initialized_ )
		return false;

	int min_x = req.x_min;
	int max_x = req.x_max;

	int min_y = req.y_min;
	int max_y = req.y_max;

	ROS_DEBUG_NAMED(ROS_NAME, "Select from region (%d, %d, %d, %d)", min_x, max_x, min_y, max_y);

	// Correct points on rectangle
	if ( not validRectangle(min_x, max_x, min_y, max_y) )
		return false;

	pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> object_cloud;

	// Get PCL point cloud from stored cloud
	pcl::fromROSMsg (point_cloud_, pcl_point_cloud);

	if ( not getSelectedPointsInRectangle(min_x, max_x, min_y, max_y, pcl_point_cloud, object_cloud))
		return false;

	if ( not filterOnOutliers(object_cloud))
		return false;

	// Get centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(object_cloud, centroid);

    // Transform vector4f to point
    pcl::PointXYZ center_point;
    center_point.getVector4fMap() = centroid;

    // Fill result message
    res.point_stamped.header  = point_cloud_.header;
	res.point_stamped.point.x = center_point.x;
	res.point_stamped.point.y = center_point.y;
	res.point_stamped.point.z = center_point.z;

    ROS_DEBUG_NAMED(ROS_NAME, "Extracted point (%2.3f, %2.3f, %2.3f) from rectangle (%i, %i) (%i, %i)",
                    center_point.x, center_point.y, center_point.z, min_x, min_y, max_x, max_y);

	return true;
}

void PointExtractor::publishSelection( pcl::PointCloud<pcl::PointXYZ> selection )
{
  	selection.header.frame_id = point_cloud_.header.frame_id;

	selected_point_cloud_pub_.publish(selection);
}