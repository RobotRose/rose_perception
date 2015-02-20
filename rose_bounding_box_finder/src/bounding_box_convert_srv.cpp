/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Kris Piters
* Date  : 2014/04/25
*     - File created.
*
* Description:
* 
* 
* TODO:
*	Make CameraModel dynamic using Camerainfo topic
*	This way a boundingbox can also be visualized in other camera images (arm camera)
*
*
*
***********************************************************************************/

#include "rose_bounding_box_finder/bounding_box_convert_srv.hpp"

#define CAM_INFO_TOPIC 		"/camera/rgb/camera_info" 	//HARDCODED; Currently the rgb frame of kinect camera
#define CAM_INFO_TIMEOUT 	3.0 //seconds
//#define CAMERA_FRAME 		"/camera_rgb_optical_frame" //replaced with cam_model_.tfFrame()

BoundingBoxConvertSrv::BoundingBoxConvertSrv(string name)
	: camera_height_ ( 480 )
	, camera_width_ ( 640 )
{
	n_ = new ros::NodeHandle(name);

	convert_service_ 	= n_->advertiseService("convert_bb_to_uv", &BoundingBoxConvertSrv::convertService, this);
	debug_pub_			= n_->advertise<pcl::PointCloud<pcl::PointXYZ> > ("debug_box_cloud", 1);
	listener_        	= new(tf::TransformListener);
	camera_info_sub_	= n_->subscribe(CAM_INFO_TOPIC, 1, &BoundingBoxConvertSrv::CB_camera_info_topic, this );

	ROS_INFO("%s Service Started; Ready to receive requests.", name.c_str());
}

BoundingBoxConvertSrv::~BoundingBoxConvertSrv(){}

void BoundingBoxConvertSrv::CB_camera_info_topic( const sensor_msgs::CameraInfo::ConstPtr camera_info_msg )
{
	cam_model_.fromCameraInfo(camera_info_msg);
	camera_height_ = camera_info_msg->height;
	camera_width_ = camera_info_msg->width;
}

bool
BoundingBoxConvertSrv::convertService(	bounding_box_finder::convert_bb_to_uv::Request 	&req,	
										bounding_box_finder::convert_bb_to_uv::Response &res )
{
	//ROS_INFO("[%s] Received Request.", ros::this_node::getName().c_str());
	//ROS_DEBUG_STREAM("BoundingBox Info: " << req.bounding_box);

	if(cam_model_.initialized())
	{
		res.camera_height = camera_height_;
		res.camera_width  = camera_width_;
		for(int i =0; i < req.bounding_boxes.bounding_box_vector.size(); i++)
		{

			res.uv_bounding_box.push_back(createUVBoundingBoxMsg(convertBBMsgToPointCloud(req.bounding_boxes.bounding_box_vector[i])));
			
			debug_pub_.publish(convertBBMsgToPointCloud(req.bounding_boxes.bounding_box_vector[i]));
			//ROS_DEBUG_STREAM("UV_BoundingBox Info: " << res.uv_bounding_box);
			//ROS_INFO("[%s] Send Response.", ros::this_node::getName().c_str());
		}

		return true;
	}
	else 
	{
		ROS_ERROR("Camera model is not intialized correctly");
		return false;
	}
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateFTLCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ ftl;

	ftl.x = bounding_box.pose_stamped.pose.position.x - (bounding_box.dimensions.x/2.0);
	ftl.y = bounding_box.pose_stamped.pose.position.y + (bounding_box.dimensions.y/2.0);
	ftl.z = bounding_box.pose_stamped.pose.position.z + (bounding_box.dimensions.z/2.0);

	return ftl;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateFTRCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ ftr;

	ftr.x = bounding_box.pose_stamped.pose.position.x - (bounding_box.dimensions.x/2.0);
	ftr.y = bounding_box.pose_stamped.pose.position.y - (bounding_box.dimensions.y/2.0);
	ftr.z = bounding_box.pose_stamped.pose.position.z + (bounding_box.dimensions.z/2.0);

	return ftr;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateFBRCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ fbr;

	fbr.x = bounding_box.pose_stamped.pose.position.x - (bounding_box.dimensions.x/2.0);
	fbr.y = bounding_box.pose_stamped.pose.position.y - (bounding_box.dimensions.y/2.0);
	fbr.z = bounding_box.pose_stamped.pose.position.z - (bounding_box.dimensions.z/2.0);

	return fbr;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateFBLCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ fbl;

	fbl.x = bounding_box.pose_stamped.pose.position.x - (bounding_box.dimensions.x/2.0);
	fbl.y = bounding_box.pose_stamped.pose.position.y + (bounding_box.dimensions.y/2.0);
	fbl.z = bounding_box.pose_stamped.pose.position.z - (bounding_box.dimensions.z/2.0);

	return fbl;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateBTLCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ btl;

	btl.x = bounding_box.pose_stamped.pose.position.x + (bounding_box.dimensions.x/2.0);
	btl.y = bounding_box.pose_stamped.pose.position.y + (bounding_box.dimensions.y/2.0);
	btl.z = bounding_box.pose_stamped.pose.position.z + (bounding_box.dimensions.z/2.0);

	return btl;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateBTRCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ btr;

	btr.x = bounding_box.pose_stamped.pose.position.x + (bounding_box.dimensions.x/2.0);
	btr.y = bounding_box.pose_stamped.pose.position.y - (bounding_box.dimensions.y/2.0);
	btr.z = bounding_box.pose_stamped.pose.position.z + (bounding_box.dimensions.z/2.0);

	return btr;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateBBRCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ bbr;

	bbr.x = bounding_box.pose_stamped.pose.position.x + (bounding_box.dimensions.x/2.0);
	bbr.y = bounding_box.pose_stamped.pose.position.y - (bounding_box.dimensions.y/2.0);
	bbr.z = bounding_box.pose_stamped.pose.position.z - (bounding_box.dimensions.z/2.0);

	return bbr;
}

pcl::PointXYZ
BoundingBoxConvertSrv::calculateBBLCornerpoint(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointXYZ bbl;

	bbl.x = bounding_box.pose_stamped.pose.position.x + (bounding_box.dimensions.x/2.0);
	bbl.y = bounding_box.pose_stamped.pose.position.y + (bounding_box.dimensions.y/2.0);
	bbl.z = bounding_box.pose_stamped.pose.position.z - (bounding_box.dimensions.z/2.0);

	return bbl;
}

//Creates a PCl Pointcloud using the Pose and Dimensions of the BoundingBox
pcl::PointCloud<pcl::PointXYZ>
BoundingBoxConvertSrv::convertBBMsgToPointCloud(bounding_box_finder::BoundingBox bounding_box)
{
	pcl::PointCloud<pcl::PointXYZ> bb_cloud;
	
	//static double tf::getYaw	(	const geometry_msgs::Quaternion & 	msg_q	 ) 	 [inline, static]
	double 			yaw 			= tf::getYaw(bounding_box.pose_stamped.pose.orientation);
	Eigen::Affine3f transformation 	= pcl::getTransformation(-bounding_box.pose_stamped.pose.position.x,-bounding_box.pose_stamped.pose.position.y,-bounding_box.pose_stamped.pose.position.z,0,0,0);

	bb_cloud.points.push_back(calculateFTLCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateFTRCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateFBRCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateFBLCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateBTLCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateBTRCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateBBRCornerpoint(bounding_box));
	bb_cloud.points.push_back(calculateBBLCornerpoint(bounding_box));

	//rotate points to match orientation of the bounding box
	pcl::transformPointCloud(bb_cloud, bb_cloud, transformation);

	transformation 	= pcl::getTransformation(0,0,0,0,0,yaw);
	pcl::transformPointCloud(bb_cloud, bb_cloud, transformation);

	transformation 	= pcl::getTransformation(	bounding_box.pose_stamped.pose.position.x,
												bounding_box.pose_stamped.pose.position.y,
												bounding_box.pose_stamped.pose.position.z,0,0,0);

	pcl::transformPointCloud(bb_cloud, bb_cloud, transformation);

	bb_cloud.header.seq = bounding_box.pose_stamped.header.seq;
	//bb_cloud.header.stamp = bounding_box.pose_stamped.header.stamp; //Stamps of different type incompatible
	bb_cloud.header.frame_id = bounding_box.pose_stamped.header.frame_id;

	//transform the cloud to the camera frame
	int retry_counter = 0;
    while(!pcl_ros::transformPointCloud(cam_model_.tfFrame(), bb_cloud, bb_cloud, *listener_))
    {
        ROS_INFO("TF_listener not ready. Retrying %d/50.", (retry_counter+1));
        
        if(retry_counter++ == 50)
        	break;
    }

    bb_cloud.header.frame_id = cam_model_.tfFrame();

	return bb_cloud;    
}

bounding_box_finder::uv_point
BoundingBoxConvertSrv::convertPCL3dPointToUVPoint(pcl::PointXYZ pcl_point)
{
	bounding_box_finder::uv_point uv_point;
	cv::Point3d cv_3d_point;
	cv::Point2i cv_2d_point;

	cv_3d_point.x = pcl_point.x;
	cv_3d_point.y = pcl_point.y;
	cv_3d_point.z = pcl_point.z;

	cv_2d_point = cam_model_.project3dToPixel(cv_3d_point);

	uv_point.u = cv_2d_point.x;
	uv_point.v = cv_2d_point.y;

	return uv_point;
}

bounding_box_finder::uv_bounding_box
BoundingBoxConvertSrv::createUVBoundingBoxMsg(pcl::PointCloud<pcl::PointXYZ> bb_cloud)
{
	bounding_box_finder::uv_bounding_box uv_bounding_box;

	for(int i = 0; i < bb_cloud.points.size(); i++)
	{
		uv_bounding_box.corners.push_back(convertPCL3dPointToUVPoint(bb_cloud.points[i]));
	}

	uv_bounding_box.header.seq = bb_cloud.header.seq;
	//uv_bounding_box.header.stamp = bb_cloud.header.stamp; //Stamps of different type incompatible
	uv_bounding_box.header.frame_id = bb_cloud.header.frame_id;

	return uv_bounding_box;
}
