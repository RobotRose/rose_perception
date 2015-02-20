/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Kris Piters
*	Date  : 2014/04/07
* 		- File created.
*
* Description:
*	* 
***********************************************************************************/

#ifndef BOUNDING_BOX_FINDER_HPP
#define BOUNDING_BOX_FINDER_HPP

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/console/time.h>

#include <pcl/common/common.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/surface/concave_hull.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "server_multiple_client/server_multiple_client.hpp"

#include "std_msgs/String.h"

//Action msgs
#include "bounding_box_finder/getboundingboxAction.h"
#include "bounding_box_finder/getboundingboxGoal.h"
#include "bounding_box_finder/getboundingboxFeedback.h"
#include "bounding_box_finder/getboundingboxResult.h"

//Servive msgs
#include "bounding_box_finder/toggle.h"

//Bounding Box msgs
#include "bounding_box_finder/BoundingBox.h"
#include "bounding_box_finder/BoundingBoxVector.h"

//Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <bounding_box_finder/bounding_box_finderConfig.h>

#include <math.h>
#include <unistd.h>

#include "rose_common/common.hpp"
#include "rose_transformations/transformations.hpp"

using namespace std;

typedef ServerMultipleClient<bounding_box_finder::getboundingboxAction> SMC;

struct BoundingBoxFinderConfig
{
	/*
	gen.add("leaf_size_param", double_t, 0, "Leaf Size for VoxelGrid (m)", 0.01, 0.1, 0.001)
	gen.add("depth_limit_param", double_t, 0, "Depth limit every point beyond this distance (m) is discarded", 1.0, 0, 4.0)
	gen.add("planar_threshold_param", double_t, 0, "Plane padding for planar segmentation (m)", 0.02, 0.0, 0.05)
	gen.add("plane_filter_limit_param", double_t, 0, "Planar segmentation keeps filtering planes until this part of the original cloud is left", 0.3, 0.0, 1.0)
	gen.add("min_cluster_size_param", int_t, 0, "Minimal Number of Points defining a cluster", 100, 1, 25000)
	gen.add("max_cluster_size_param", int_t, 0, "Maximum Number of Points defining a cluster", 25000, 1, 25000)
	gen.add("cluster_tolerance_param", double_t, 0, "Furthest distance between points to be seen as one cluster (m)", 0.02, 0.0, 0.10)
	gen.add("object_frame_param", str_t, 0, "Frame to which the cluster points are transformed", "/map")
	gen.add("radian_increment_param", double_t, 0, "Size of the steps taken when rotating the pointcluster to determine the orientation (radian)", 0.02, 0.01, 1.57)
	gen.add("enable_continuous_operation_param", bool_t, 0, "When true the algoritm keeps operating without the need for a Goal from SMC", True)
	*/

	double 	leaf_size;
	double 	depth_limit;
	double 	planar_threshold;
	double 	plane_filter_limit;
	int 	min_cluster_size;
	int 	max_cluster_size;
	double 	cluster_tolerance;
	string 	object_frame;
	double 	radian_increment;
	bool 	enable_continuous_operation;
};


class BoundingBoxFinder
{

	public:
		BoundingBoxFinder(string name, ros::NodeHandle n);
		~BoundingBoxFinder();

		void 
	    CB_dynamicReconfigure(	bounding_box_finder::bounding_box_finderConfig &config, 
	    						uint32_t level);

	    
	private:
		bool 
		toggle( bounding_box_finder::toggle::Request  &req, 
                bounding_box_finder::toggle::Response &res );

	    void 
	    CB_serverWork( const bounding_box_finder::getboundingboxGoalConstPtr &goal, 
	    			   SMC* smc);
	    
	    void 
	    CB_serverCancel( SMC* smc);

	    

	    void 
	    CB_depthPoints( const sensor_msgs::PointCloud2ConstPtr& input);
	    

	    void 
	    voxelGrid( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
	               float                                  leaf_size, 
	               float                                  depth_limit);

	    void 
	    planarSegmentation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
	                        float                                   plane_filter_limit,
	                        float                                   planar_threshold);

	    bool 
	    euclideanClusterExtraction( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
	                                std::vector<pcl::PointIndices>*         clusters_indices, 
	                                int                                     min_cluster_size, 
	                                int                                     max_cluster_size,
	                                float                                   cluster_tolerance);

	    /*
	    pcl::PointCloud<pcl::PointXYZRGB> convexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
	                              					 pcl::PointIndices                       indices);
		*/

	    pcl::PointCloud<pcl::PointXYZRGB> 
	    convexHull( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in);

	    Eigen::Vector4f 
	    getCenterpointOfPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

	    bounding_box_finder::BoundingBox 
	    calculateOrientedBoundingBoxByRotation( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in);

	    bounding_box_finder::BoundingBoxVector 
	    createBoundingBoxVector( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
	     						 const std::vector<pcl::PointIndices>& cluster_indices);

	    void 
	    colorCloudBasedOnIndices( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
	    						  const std::vector<pcl::PointIndices>& cluster_indices);

	    visualization_msgs::MarkerArray 
	    createMarkerArrayFromBoundingBoxes( bounding_box_finder::BoundingBoxVector bounding_boxes);

	    void 
	    publishBoundingBoxes( bounding_box_finder::BoundingBoxVector bounding_boxes);

	    void 
	    publishColoredObjectClusterCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
	     							      const std::vector<pcl::PointIndices>& cluster_indices);

	    void 
	    publishBoundingBoxesAsMarkers( bounding_box_finder::BoundingBoxVector bounding_boxes);

	    bool 					config_initialized_;

	    ros::NodeHandle     	n_;
	    string              	name_;
	    SMC                 	smc_;

	    ros::Publisher      	marker_pub_;
	    ros::Publisher      	bounding_boxes_pub_;
	    ros::Publisher      	clusters_pub_;
	    
	    ros::Subscriber     	depth_points_sub_;

	    ros::ServiceServer		toggle_service_;

	    tf::TransformListener 	tf_;

	    bounding_box_finder::BoundingBoxVector bounding_boxes_;

	    BoundingBoxFinderConfig config_;

	    dynamic_reconfigure::Server<bounding_box_finder::bounding_box_finderConfig> reconfigure_server_;
  		dynamic_reconfigure::Server<bounding_box_finder::bounding_box_finderConfig>::CallbackType reconfigure_callback_ptr;

	      	    
};

#endif //BOUNDING_BOX_FINDER_HPP

