/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
* Author: Kris Piters
* Date  : 2014/04/07
*     - File created.
*
* Description:
* 
* 
* TODO:
* - Add pointcloud templates
* - MAKE CUSTOM ACTION GOAL ETC for serverWork callback
* - Create separate Classes
* - Add dynamic_reconfigure functionality
* - TREADING?
* - 
*
*
***********************************************************************************/

#include "rose_bounding_box_finder/bounding_box_finder.hpp"

//Defines for VoxelGrid
#define LEAF_SIZE           config_.leaf_size     // 0.01          //Meter
#define DEPTH_LIMIT         config_.depth_limit   // 1.0           //Meter

//Defines for planar segmentation
#define PLANAR_THRESHOLD    config_.planar_threshold  // 0.02          //Meter
#define PLANE_FILTER_LIMIT  config_.plane_filter_limit  // 0.3           //Part of total cloud

//Defines for Clustering
#define MIN_CLUSTER_SIZE    config_.min_cluster_size  // 100           //Minimal Number of Points in Cluster
#define MAX_CLUSTER_SIZE    config_.max_cluster_size  // 25000         //Maximum Number of Points in Cluster
#define CLUSTER_TOLERANCE   config_.cluster_tolerance   // 0.02          //CM Furthest distance between point to be seen as one cluster

//Defines for calculateOrientedBoundingBoxByRotation
#define OBJECT_FRAME        config_.object_frame          //"/base_link"  //Frame to which the cluster points are transformed
#define INCREMENT           config_.radian_increment      // 0.02          //Radian

//! @todo MdL: Should be configurable, or given with the request of bounding boxes.
#define ROTATION_FRAME      "base_link"           // The frame which defines the z-axis to rotate around

BoundingBoxFinder::BoundingBoxFinder(string name, ros::NodeHandle  n)
    : n_ ( n )
    , name_ ( name )
    , config_initialized_ ( false )
    , smc_ (n_, name_, boost::bind(&BoundingBoxFinder::CB_serverWork, this, _1, _2),
                       boost::bind(&BoundingBoxFinder::CB_serverCancel, this, _1))
{  
    marker_pub_         = n_.advertise<visualization_msgs::MarkerArray> (name + "/markers", 1);
    clusters_pub_       = n_.advertise<sensor_msgs::PointCloud2> (name + "/clusters", 1);
    bounding_boxes_pub_ = n_.advertise<rose_bounding_box_finder::BoundingBoxVector> (name + "/bounding_box_vector", 1);
    //depth_points_sub_   = n_.subscribe ("/camera/depth_registered/points", 1, &BoundingBoxFinder::CB_depthPoints, this);
    //
    toggle_service_     = n_.advertiseService(name + "/toggle", &BoundingBoxFinder::toggle, this);


    reconfigure_callback_ptr = boost::bind(&BoundingBoxFinder::CB_dynamicReconfigure, this, _1, _2);
    reconfigure_server_.setCallback(reconfigure_callback_ptr);  
    
    smc_.startServer();
}

BoundingBoxFinder::~BoundingBoxFinder()
{

}

bool BoundingBoxFinder::toggle( rose_bounding_box_finder::toggle::Request  &req, 
                                rose_bounding_box_finder::toggle::Response &res )
{
    ROS_DEBUG_NAMED(ROS_NAME, "BoundingBoxFinder::toggle-ing");
    if ( req.on )
    {
        ROS_DEBUG_NAMED(ROS_NAME, "BoundingBoxFinder::toggle: turning ON");
        depth_points_sub_ = n_.subscribe ("/camera/depth_registered/points", 1, &BoundingBoxFinder::CB_depthPoints, this);
        ROS_INFO_NAMED(ROS_NAME, "BoundingBoxFinder::toggle: turned ON");
    }
    else
    {
      ROS_DEBUG_NAMED(ROS_NAME, "BoundingBoxFinder::toggle: turning OFF");
      depth_points_sub_.shutdown();

      // Publish an empty list of bounding boxes
      rose_bounding_box_finder::BoundingBoxVector bounding_boxes;
      publishBoundingBoxes(bounding_boxes);
      ROS_INFO_NAMED(ROS_NAME, "BoundingBoxFinder::toggle: turned OFF");
    }

    return true;
}

//TODO add check for object_frame_name
//causes bug in calculateOrientedBoundingBoxByRotation
void BoundingBoxFinder::CB_dynamicReconfigure(rose_bounding_box_finder::rose_bounding_box_finderConfig &config, uint32_t level)
{
    ROS_INFO_NAMED(ROS_NAME, "Reconfigure Request: %f %f %f %f %d %d %f %s %f %s", 
              config.leaf_size_param, 
              config.depth_limit_param,
              config.planar_threshold_param,
              config.plane_filter_limit_param,
              config.min_cluster_size_param,
              config.max_cluster_size_param,
              config.cluster_tolerance_param,
              config.object_frame_param.c_str(),
              config.radian_increment_param,
              config.enable_continuous_operation_param?"True":"False");

    config_.leaf_size                     = config.leaf_size_param;
    config_.depth_limit                   = config.depth_limit_param;
    config_.planar_threshold              = config.planar_threshold_param;
    config_.plane_filter_limit            = config.plane_filter_limit_param;
    config_.min_cluster_size              = config.min_cluster_size_param;
    config_.max_cluster_size              = config.max_cluster_size_param;
    config_.cluster_tolerance             = config.cluster_tolerance_param;
    config_.radian_increment              = config.radian_increment_param;
    config_.enable_continuous_operation   = config.enable_continuous_operation_param;

    // This line assumes that the initial frame (in the .cfg file) exists in the TF tree.
    // This line only executes (while on object frame) when the bounding box finder it created for the first time.
    while ( not config_initialized_ && not tf_.frameExists(config.object_frame_param))
    {
        ROS_INFO_NAMED(ROS_NAME, "Waiting for frame %s to come up...", config.object_frame_param.c_str() );
    }

    config_initialized_ = true;

    if(tf_.frameExists(config.object_frame_param))
        config_.object_frame                = config.object_frame_param;
    else
    {
        config_.object_frame                = "/map";

        ROS_ERROR("Invalid object_frame \"%s\", defaulting to \"%s\".", config.object_frame_param.c_str(), config_.object_frame.c_str());
    }
    ROS_DEBUG_NAMED(ROS_NAME, "Reconfigure completed");
}


void 
BoundingBoxFinder::voxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                             float                                  leaf_size, 
                             float                                  depth_limit)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_voxel (new pcl::PointCloud<pcl::PointXYZRGB>);

    //filter cloud using voxelgrid
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setLeafSize (leaf_size,leaf_size,leaf_size);
    sor.setFilterFieldName ("z");
    sor.setFilterLimits(0.0, depth_limit);
    sor.filter (*cloud_voxel);

    //ROS_INFO("cloud_voxel size: %lu", cloud_voxel->points.size());    

    *cloud_in = *cloud_voxel;
    ROS_INFO("voxelGrid time: %.2f", tt.toc());
}

void 
BoundingBoxFinder::planarSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
                                      float                                   plane_filter_limit,
                                      float                                   planar_threshold)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::SACSegmentation<pcl::PointXYZRGB>  seg;
    pcl::PointIndices::Ptr                  inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr             coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (planar_threshold); 

    int i=0, nr_points = (int) cloud_in->points.size ();
    //ROS_INFO("planarSegmentation cloud_in size: %.lu", cloud_in->points.size ());
    while (cloud_in->points.size () > plane_filter_limit * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_in);
      seg.segment (*inliers, *coefficients);
      
      if (inliers->indices.size () == 0)
      {
        ROS_DEBUG_STREAM("planarSegmentation: Could not estimate a planar model for the given dataset.");
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud (cloud_in);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      
      //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
      ROS_DEBUG_STREAM("planarSegmentation: PointCloud representing the planar component: " << cloud_plane->points.size () << " data points.");

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_in = *cloud_f;
    }

    ROS_INFO("PlanarSegmentation time: %.2f", tt.toc());

}


bool
BoundingBoxFinder::euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
                                              std::vector<pcl::PointIndices>*         clusters_indices, 
                                              int                                     min_cluster_size, 
                                              int                                     max_cluster_size,
                                              float                                   cluster_tolerance)
{
    if(cloud_in->points.size() < min_cluster_size)
        return false;

    pcl::console::TicToc tt;
    tt.tic();

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr        tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr            cloud_clusters;
    std::vector<pcl::PointIndices>                    cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    
    tree->setInputCloud (cloud_in);

    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);

    ROS_DEBUG_STREAM("euclideanClusterExtraction: Found " << cluster_indices.size () << " Clusters");
    
    *clusters_indices = cluster_indices;

    ROS_INFO("euclideanClusterExtraction time: %.2f", tt.toc());

    return true;
}

/*
//TODO
//Returns a point cloud containing the Convex Hull points of a cloud with indices.
pcl::PointCloud<pcl::PointXYZRGB>
BoundingBoxFinder::convexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, 
                              pcl::PointIndices                       indices)
{
  pcl::console::TicToc tt;
  tt.tic();

  pcl::ConvexHull<pcl::PointXYZRGB>       chull;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  chull_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  chull.setInputCloud(cloud_in);
  chull.setIndices(&indices);
  chull.reconstruct (*chull_points);

  ROS_INFO("convexHull time: %.2f", tt.toc());
  return *chull_points;  
}
*/

pcl::PointCloud<pcl::PointXYZRGB>
BoundingBoxFinder::convexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in)
{
    //pcl::console::TicToc tt;
    //tt.tic();

    pcl::ConvexHull<pcl::PointXYZRGB>       chull;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  chull_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    chull.setInputCloud(cloud_in);
    chull.reconstruct (*chull_points);

    //ROS_INFO("convexHull time: %.2f", tt.toc());

    return *chull_points;
}

Eigen::Vector4f
BoundingBoxFinder::getCenterpointOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    Eigen::Vector4f min, max, center;
    
    pcl::getMinMax3D(*cloud_in, min, max);
    
    center[0] = min[0] + ((max[0]-min[0])/2);
    center[1] = min[1] + ((max[1]-min[1])/2);
    center[2] = min[2] + ((max[2]-min[2])/2);
    
    return center;
}

//TODO transform whole clpoud instead of each cluster
rose_bounding_box_finder::BoundingBox 
BoundingBoxFinder::calculateOrientedBoundingBoxByRotation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in)
{
    //Timer
    pcl::console::TicToc tt;
    tt.tic();

    rose_bounding_box_finder::BoundingBox bounding_box;
        
    //pcl compatible cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    //Copy iput cloud to local cloud and convert To convex Hull to save computation
    *cluster_cloud = convexHull(cloud_in);

    //transform cloud to base_link frame 
    //BUG! changing object frame to non existing frame will cause the infinite loop
    //USED QUICK FIX WITH COUNTER
    int counter = 0;
    while(!pcl_ros::transformPointCloud(ROTATION_FRAME, *cluster_cloud, *cluster_cloud, tf_))
    {
        ROS_INFO("TF_listener not ready. Retrying %d/50.", (counter+1));
        
        if(counter++ == 50)
          break;
    }
        
    //get centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloud, centroid);
    
    //get center
    Eigen::Vector4f center;
    center = getCenterpointOfPointCloud(cluster_cloud);
    
    //Compute original min and max values
    Eigen::Vector4f org_min, org_max;
    pcl::getMinMax3D(*cluster_cloud, org_min, org_max);
    
    //translate cloud to world frame origin TODO: HACK!
    Eigen::Affine3f transform = pcl::getTransformation(-center[0],-center[1],-center[2],0,0,0); 
    pcl::transformPointCloud(*cluster_cloud, *cluster_cloud, transform);
    
    //rotation angle
    float theta = 0, min_theta = 0;
    float min_vol_bb, cur_vol_bb;
    Eigen::Vector4f min_min, min_max;
    float min_theta_ar[2] = {0,0};
    Eigen::Vector4f min_min_ar[2];
    Eigen::Vector4f min_max_ar[2];
    
    //add min_vol_bb
    pcl::getMinMax3D(*cluster_cloud, min_min, min_max);
    min_vol_bb = ((min_max[0]-min_min[0])*(min_max[1]-min_min[1])*(min_max[2]-min_min[2]));
    
    //for loop angle 0 - 2 pi rad
      //rotate cloud around centroid?
      //get min max
      //calculate volume from min max
      //check if minimal volume
        //yes?
          //save used rotation angle
          //save minimal volume
          //save min - max
    for(theta = 0; theta <= (M_PI); theta += INCREMENT)
    {
      Eigen::Vector4f min, max;
      Eigen::Affine3f transformation = pcl::getTransformation(0,0,0,0,0,INCREMENT);
      
      pcl::transformPointCloud(*cluster_cloud, *cluster_cloud, transformation);
      pcl::getMinMax3D(*cluster_cloud, min, max);
      
      cur_vol_bb = ((max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]));
      
      if(cur_vol_bb < min_vol_bb)
      {
        min_theta = theta;
        min_min = min;
        min_max = max;
        min_vol_bb = cur_vol_bb;
        
        min_theta_ar[0] = min_theta_ar[1];
        min_theta_ar[1] = min_theta;
        
        min_min_ar[0] = min_min_ar[1];
        min_min_ar[1] = min_min;
        
        min_max_ar[0] = min_max_ar[1];
        min_max_ar[1] = min_max;
       }
    }

    bounding_box.pose_stamped.header.stamp     = ros::Time(0);
    bounding_box.pose_stamped.header.frame_id  = ROTATION_FRAME;

    bounding_box.pose_stamped.pose.position.x  = center[0];
    bounding_box.pose_stamped.pose.position.y  = center[1];
    bounding_box.pose_stamped.pose.position.z  = center[2];

    bounding_box.pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(-min_theta);

    bounding_box.dimensions.x = (min_max[0]-min_min[0]);
    bounding_box.dimensions.y = (min_max[1]-min_min[1]);
    bounding_box.dimensions.z = (min_max[2]-min_min[2]);
    
    //! @todo MdL: Transform to output object frame.
    rose_transformations::transformToFrame(tf_, OBJECT_FRAME, bounding_box.pose_stamped);

    //ROS_INFO("calculateOrientedBoundingBoxByRotation time: %.2f", tt.toc());

    return bounding_box;
}

//TODO transform cloud before making calculating seperate bouding boxes
rose_bounding_box_finder::BoundingBoxVector
BoundingBoxFinder::createBoundingBoxVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, const std::vector<pcl::PointIndices>& cluster_indices)
{
    pcl::console::TicToc tt;
    tt.tic();

    rose_bounding_box_finder::BoundingBoxVector bounding_boxes;
    //ROS_INFO_STREAM("createBoundingBoxVector cloud_in frame ID: " << cloud_in->header.frame_id);
    for(int i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

        cluster->header.frame_id = cloud_in->header.frame_id;
        
        for(int k = 0; k < (cluster_indices)[i].indices.size(); k++)
        {
            cluster->points.push_back(cloud_in->points[(cluster_indices)[i].indices[k]]);
        }

        bounding_boxes.bounding_box_vector.push_back(calculateOrientedBoundingBoxByRotation(cluster));
    }

    ROS_INFO("createBoundingBoxVector time: %.2f", tt.toc());
    
    return bounding_boxes;
}

//TODO
void BoundingBoxFinder::colorCloudBasedOnIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, const std::vector<pcl::PointIndices>& cluster_indices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    *colored_cloud = *cloud_in;

    for(int i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        int r = (1000 *(i+1)) % 255;
        int g = (2000 *(i+10)) % 255;
        int b = (3000 *(i+100)) % 255;

        for(int k = 0; k < (cluster_indices)[i].indices.size(); k++)
        {
            colored_cloud->points[(cluster_indices)[i].indices[k]].r = r;
            colored_cloud->points[(cluster_indices)[i].indices[k]].g = g;
            colored_cloud->points[(cluster_indices)[i].indices[k]].b = b;
        }      
    }

    *cloud_in = *colored_cloud;
}

visualization_msgs::MarkerArray
BoundingBoxFinder::createMarkerArrayFromBoundingBoxes(rose_bounding_box_finder::BoundingBoxVector bounding_boxes)
{
    visualization_msgs::MarkerArray markers;

    for(int i = 0; i < bounding_boxes.bounding_box_vector.size(); i++)
    {
        visualization_msgs::Marker marker;
    
        marker.ns               = "BB";
        marker.id               = i;
        marker.type             = visualization_msgs::Marker::CUBE;
        marker.action           = visualization_msgs::Marker::ADD;

        marker.header.frame_id  = bounding_boxes.bounding_box_vector[i].pose_stamped.header.frame_id;//frame_id;
        marker.header.stamp     = ros::Time::now();

        marker.pose             = bounding_boxes.bounding_box_vector[i].pose_stamped.pose;
        
        marker.scale.x          = bounding_boxes.bounding_box_vector[i].dimensions.x;
        marker.scale.y          = bounding_boxes.bounding_box_vector[i].dimensions.y;
        marker.scale.z          = bounding_boxes.bounding_box_vector[i].dimensions.z;
       
        if (marker.scale.x ==0)
            marker.scale.x=0.1;

        if (marker.scale.y ==0)
          marker.scale.y=0.1;

        if (marker.scale.z ==0)
            marker.scale.z=0.1;
        
        marker.color.r = float((int)((1000 *(i+1)) % 255))/255;
        marker.color.g = float((int)((2000 *(i+10)) % 255))/255;
        marker.color.b = float((int)((3000 *(i+100)) % 255))/255;
        marker.color.a = 0.5;
                
        marker.lifetime = ros::Duration(10);

        markers.markers.push_back(marker);
    }

    return markers;
}

void BoundingBoxFinder::publishBoundingBoxes (rose_bounding_box_finder::BoundingBoxVector bounding_boxes)
{
    bounding_boxes_pub_.publish(bounding_boxes);
}

//TODO
void BoundingBoxFinder::publishColoredObjectClusterCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_in, const std::vector<pcl::PointIndices>& cluster_indices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    *colored_clusters = *cloud_in;
    
    colorCloudBasedOnIndices(colored_clusters, cluster_indices);
    
    clusters_pub_.publish(colored_clusters);
}

void BoundingBoxFinder::publishBoundingBoxesAsMarkers (rose_bounding_box_finder::BoundingBoxVector bounding_boxes)
{
    marker_pub_.publish(createMarkerArrayFromBoundingBoxes(bounding_boxes));
}

void 
BoundingBoxFinder::CB_depthPoints (const sensor_msgs::PointCloud2ConstPtr& input)
{
    //if (not smc_.hasActiveGoal() )
    //      return;
  
    pcl::console::TicToc tt;
    tt.tic();
    //ROS_INFO("Input Cloud: %d", (input->width * input->height));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<pcl::PointIndices>          clusters_indices;
  
    //Convert from ROS Cloud to pcl Cloud
    pcl::fromROSMsg(*input, *cloud_pcl);

    cloud_pcl->header.frame_id = input->header.frame_id;

    //ROS_INFO("Cloud_pcl BEFORE Voxel:: %lu", cloud_pcl->points.size());
    //Perform Downsampling using VoxelGrid
    
    //ROS_INFO("config_.leaf_size: %f - config_.depth_limit: %f", config_.leaf_size, config_.depth_limit);

    voxelGrid(cloud_pcl, LEAF_SIZE, DEPTH_LIMIT);//LEAF_SIZE, DEPTH_LIMIT);

    //ROS_INFO("Cloud_pcl after Voxel:: %lu", cloud_pcl->points.size());

    //Perform PlanarSegmentation; resulting cloud contains object clusters
    planarSegmentation(cloud_pcl, PLANE_FILTER_LIMIT, PLANAR_THRESHOLD);

    //TODO: Return feedback
    if(!euclideanClusterExtraction(cloud_pcl, &clusters_indices, MIN_CLUSTER_SIZE, MAX_CLUSTER_SIZE, CLUSTER_TOLERANCE))
    {
        ROS_ERROR("CB_depthPoints: -NO CLUSTERS FOUND- , Points left in Cloud: %lu", cloud_pcl->points.size());
        publishColoredObjectClusterCloud(cloud_pcl, clusters_indices);
        ROS_INFO("CB_depthPoints time: %.2f", tt.toc());
        return;
    }

    bounding_boxes_ = createBoundingBoxVector(cloud_pcl, clusters_indices);

    publishBoundingBoxesAsMarkers(bounding_boxes_);
    
    publishBoundingBoxes(bounding_boxes_);

    publishColoredObjectClusterCloud(cloud_pcl, clusters_indices);
  
    ROS_INFO("CB_depthPoints time: %.2f", tt.toc());
}



void BoundingBoxFinder::CB_serverWork( const rose_bounding_box_finder::getboundingboxGoalConstPtr &goal, SMC* smc ){}
void BoundingBoxFinder::CB_serverCancel( SMC* smc ){}


/*
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//NOTES:
//[bounding_box_finder/BoundingBox]:
//geometry_msgs/PoseStamped pose_stamped
//  std_msgs/Header header
//    uint32 seq
//    time stamp
//    string frame_id
//  geometry_msgs/Pose pose
//    geometry_msgs/Point position
//      float64 x
//      float64 y
//      float64 z
//    geometry_msgs/Quaternion orientation
//      float64 x
//      float64 y
//      float64 z
//      float64 w
//geometry_msgs/Vector3 dimensions
//  float64 x
//  float64 y
//  float64 z
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
