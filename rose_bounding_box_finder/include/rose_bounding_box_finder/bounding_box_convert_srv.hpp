/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Kris Piters
*	Date  : 2014/04/25
* 		- File created.
*
* Description:
*	* 
***********************************************************************************/

#ifndef BOUNDING_BOX_CONVERT_SRV_HPP
#define BOUNDING_BOX_CONVERT_SRV_HPP

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/console/time.h>

#include <pcl/common/common.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

//Service definition
#include "rose_bounding_box_finder/convert_bb_to_uv.h"

//Bounding Box msgs
#include "rose_bounding_box_finder/BoundingBox.h"
#include "rose_bounding_box_finder/BoundingBoxVector.h"
#include "rose_bounding_box_finder/uv_point.h"
#include "rose_bounding_box_finder/uv_bounding_box.h"

#include <math.h>
#include <unistd.h>


using namespace std;

class BoundingBoxConvertSrv
{
	public:
		BoundingBoxConvertSrv(string name);
		~BoundingBoxConvertSrv();

	private:

		void CB_camera_info_topic( const sensor_msgs::CameraInfo::ConstPtr camera_info_msg );
		bool
		convertService(	bounding_box_finder::convert_bb_to_uv::Request 	&req,
						bounding_box_finder::convert_bb_to_uv::Response &res);
		
		pcl::PointXYZ
		calculateFTLCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateFTRCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateFBRCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateFBLCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateBTLCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateBTRCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateBBRCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointXYZ
		calculateBBLCornerpoint(bounding_box_finder::BoundingBox bounding_box);

		pcl::PointCloud<pcl::PointXYZ>
		convertBBMsgToPointCloud(bounding_box_finder::BoundingBox bounding_box);

		bounding_box_finder::uv_point
		convertPCL3dPointToUVPoint(pcl::PointXYZ pcl_point);

		bounding_box_finder::uv_bounding_box
		createUVBoundingBoxMsg(pcl::PointCloud<pcl::PointXYZ> bb_cloud);

		ros::NodeHandle* 	n_;
		ros::ServiceServer 	convert_service_;
		ros::Subscriber		camera_info_sub_;
		ros::Publisher		debug_pub_;

		image_geometry::PinholeCameraModel cam_model_;

		tf::TransformListener *listener_;

		int camera_height_;
		int camera_width_;

};

#endif //BOUNDING_BOX_CONVERT_SRV_HPP
