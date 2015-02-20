#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <rose_ui_overview_camera/selection.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//SERVICE
#include "rose_ui_overview_camera/extract_from_cloud.h"

sensor_msgs::PointCloud2 cloud2;

geometry_msgs::PointStamped sendPointS;
bool received_rect = false;

ros::Publisher SPoint_pub;
ros::Publisher point_sel_stat_pub;
ros::Publisher speaker_pub;


int rec_x1,rec_y1,rec_x2,rec_y2;
float lx,ly,lz, rx, ry, rz;
float wlx,wly,wlz, wrx, wry, wrz;

void rectCallback(const rose_ui_overview_camera::selection::ConstPtr& msg)
{
	ROS_INFO("RECV : %d, %d, %d, %d",msg->x1,msg->y1,msg->x2,msg->y2);
	rec_x1 = msg->x1;
	rec_x2 = msg->x2;
	rec_y1 = msg->y1;
	rec_y2 = msg->y2;
	received_rect = true;
}


// Testing PointCloud2 - Works! Use this if you want to eliminate tracking.
void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

	ROS_INFO("PCD Received!! - points2Callback");
	cloud2 = *msg;
	int cloud_width = cloud2.width;
	int cloud_height = cloud2.height;	
	ROS_INFO("Width: %d 	Height: %d", cloud_width, cloud_height);
	pcl::PointCloud<pcl::PointXYZ> cloudPt;
	pcl::fromROSMsg (cloud2, cloudPt);

	if(false)
	{
		pcl::PointCloud<pcl::PointXYZ> object_cloud;
		ROS_INFO("Check 1");
		if(rec_x2 <= rec_x1 || rec_y2 <= rec_y1) ROS_INFO("Invalid rect");
		else
		{
			ROS_INFO("Check 2");
			int wide = rec_x2 - rec_x1; 
			int high = rec_y2 - rec_y1;

			object_cloud.width = wide;
			object_cloud.height = high;
			object_cloud.is_dense = false;
			object_cloud.points.resize (object_cloud.width * object_cloud.height);

			float minX, maxX, minY, maxY, x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_4, y_4, z_4;
			minX = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].x; 
			maxX = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].x; 
			minY = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].y; 
			maxY = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].y;

			for(int i = 0; i < high; i++)
			{				
				for(int j = 0; j < wide; j++ )
				{
					float tempx = cloudPt.points[640*((rec_y1 + i)) + (rec_x1 + j)].x;
					float tempy = cloudPt.points[640*((rec_y1 + i)) + (rec_x1 + j)].y;
					
					if(tempx > -2.0 && 
						tempx < 2.0){ minX = tempx; maxX = minX;}
					if(tempy > -2.0 && 
						tempy < 2.0){ minY = tempy; maxY = minY;}
				}
			}
			ROS_INFO("Check 2");
			ROS_INFO("INIT VAL (%f,%f,%f,%f)",minX,maxX,minY,maxY);

			for(int i = 0; i < high; i++)
			{				
				for(int j = 0; j < wide; j++ )
				{
					object_cloud.points[wide*i+j].x = cloudPt.points[cloud_width*((rec_y1 + i)) + (rec_x1 + j)].x; //cloud_width was 640
					object_cloud.points[wide*i+j].y = cloudPt.points[cloud_width*((rec_y1 + i)) + (rec_x1 + j)].y;
					object_cloud.points[wide*i+j].z = cloudPt.points[cloud_width*((rec_y1 + i)) + (rec_x1 + j)].z;
					
				}
			}
			ROS_INFO("Check 3");
			//pcl::io::savePCDFileASCII ("test_pcd.pcd", object_cloud);
			//ROS_INFO ("Saved %d data points to test_pcd.pcd.", (int)object_cloud.points.size ());

			int centerx = (wide/2.0);
			int centery = (high/2.0);
			bool left = true;
			bool right = true;
			for(int lindex = 1; lindex < wide/2.0; lindex++)
			{
				int ind = (wide * high) / 2.0;
				if(left && object_cloud.points[ind - lindex].z > 0.15)// && object_cloud.points[ind - lindex].z < 2.0 )
				{
					x_1 = object_cloud.points[ind - lindex].x;
					y_1 = object_cloud.points[ind - lindex].y;
					z_1 = object_cloud.points[ind - lindex].z;
					ROS_INFO("hey left");
					left = false;
				}

				if(right && object_cloud.points[ind + lindex].z > 0.15)// && object_cloud.points[ind + lindex].z < 2.0)
				{
					x_2 = object_cloud.points[ind + lindex].x;
					y_2 = object_cloud.points[ind + lindex].y;
					z_2 = object_cloud.points[ind + lindex].z;
					ROS_INFO("hey right");
					right = false;
				}
			}
			ROS_INFO("Check 4");
			////////////////////////////// ADDING /////////////////////////////
			ROS_INFO("--------------UPPER POINT DETECTION---------");
			bool uleft = true;
			bool bright = true;
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide/2.0);
				if(uleft && object_cloud.points[ind + lindex].z > 0.15)
				{
					lx = object_cloud.points[ind + lindex].x;
					ly = object_cloud.points[ind + lindex].y;
					lz = object_cloud.points[ind + lindex].z;
					ROS_INFO("good point @ top");
					uleft = false;
				}
			}
			ROS_INFO("--------------LOWER POINT DETECTION---------");
			ROS_INFO("Check 5");
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide * (high-1)) + (wide/2.0);

				if(bright && object_cloud.points[ind - lindex].z > 0.15)
				{
					rx = object_cloud.points[ind - lindex].x;
					ry = object_cloud.points[ind - lindex].y;
					rz = object_cloud.points[ind - lindex].z;
					ROS_INFO("good point @ bottom");
					bright = false;
				}

			}
			ROS_INFO("--------------LEFT POINT DETECTION---------");
			bool wleft = true;
			bool wright = true;
			ROS_INFO("Check 6");
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide*(high/2.0)) + 1;
				if(wleft && object_cloud.points[ind + lindex].z > 0.15)
				{
					wlx = object_cloud.points[ind + lindex].x;
					wly = object_cloud.points[ind + lindex].y;
					wlz = object_cloud.points[ind + lindex].z;
					ROS_INFO("good point @ left");
					wleft = false;
				}
			}
			ROS_INFO("--------------RIGHT POINT DETECTION---------");
			ROS_INFO("Check 6");
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide*(high/2.0)) + (wide - 1);

				if(wright && object_cloud.points[ind - lindex].z > 0.15)
				{
					wrx = object_cloud.points[ind - lindex].x;
					wry = object_cloud.points[ind - lindex].y;
					wrz = object_cloud.points[ind - lindex].z;
					ROS_INFO("good point @ right");
					wright = false;
				}

			}
			///////////////////////////////////////////////////////////////////
			ROS_INFO("Check 7");
			// Added IF construct remove for old state
			if(left || right || uleft || bright || wleft || wright)
			{		
				std::stringstream mode_str;
				mode_str<<"Please select again..";
				std_msgs::String msgs;
				msgs.data = mode_str.str();
				point_sel_stat_pub.publish(msgs);
				ROS_INFO("Told user to select another point");

			      std_msgs::Int32 comm;
			      comm.data = 50;
			      // Announce bad point
			      speaker_pub.publish(comm);
			}
			else
			{
				ROS_INFO("Check 8");
				std::stringstream mode_str;
				mode_str<<"";
				std_msgs::String msgs;
				msgs.data = mode_str.str();
				point_sel_stat_pub.publish(msgs);
				ROS_INFO("Told user the point is good");

				ROS_INFO("MIN_X: %f, MAX_X: %f, MIN_Y: %f, MAX_Y: %f", minX,maxX,minY,maxY);
				ROS_INFO("left points (%f, %f, %f)", x_1, y_1, z_1);
				ROS_INFO("right points (%f, %f, %f)", x_2, y_2, z_2);
				//ROS_INFO("upper points (%f, %f, %f)", x_3, y_3, z_3);
				//ROS_INFO("lower points (%f, %f, %f)", x_4, y_4, z_4);
				ROS_INFO("AVERAGE DIST: %f",(z_1 + z_2)/2.0);
				ROS_INFO("New point: (%f, %f, %f)",(x_1 + x_2)/2.0, (y_1 + y_2)/2.0,(z_1 + z_2)/2.0);


				ROS_INFO("upper corner points (%f, %f, %f)", lx,ly,lz);
				ROS_INFO("lower corner points (%f, %f, %f)", rx,ry,rz);
				ROS_INFO("left corner points (%f, %f, %f)", wlx,wly,wlz);
				ROS_INFO("right corner points (%f, %f, %f)", wrx,wry,wrz);

				float obj_height = sqrt(pow(lx - rx,2.0) + pow(ly - ry,2.0) + pow(lz - rz,2.0));
				ROS_INFO("Object Height: %f",obj_height);
				float obj_width = sqrt(pow(wlx - wrx,2.0) + pow(wly - wry,2.0) + pow(wlz - wrz,2.0));
				ROS_INFO("Object width: %f",obj_width);

			        std_msgs::Int32 comm;
				comm.data = 51;
				// Announce good point
				speaker_pub.publish(comm);

				sendPointS.header.stamp = ros::Time::now();
				sendPointS.point.x = (x_1 + x_2)/2.0;
				sendPointS.point.y = (y_1 + y_2)/2.0;
				sendPointS.point.z = (z_1 + z_2)/2.0;
				SPoint_pub.publish(sendPointS);

				// obj.cx = (x_1 + x_2)/2.0;
				// obj.cx = (y_1 + y_2)/2.0;
				// obj.cz = (z_1 + z_2)/2.0;

				// obj.ux = lx;				
				// obj.uy = ly;
				// obj.uz = lz;

				// obj.bx = rx;				
				// obj.by = ry;
				// obj.bz = rz;

				// obj.lx = wlx;				
				// obj.ly = wly;
				// obj.lz = wlz;

				// obj.rx = wrx;				
				// obj.ry = wry;
				// obj.rz = wrz;

				// obj.width = obj_width;
				// obj.height = obj_height;

				//object_pub.publish(obj);
			}
		}
	}

	received_rect = false;
	//ROS_INFO("Cloud size: %d",cloudPt.points.size());
	//ROS_INFO("Test: %f",cloud_width,cloud_height);
	//ROS_INFO("Test data X : %f, Y : %f, Z : %f",cloudPt.points[(640*239)+320].x, cloudPt.points[(640*239)+320].y, cloudPt.points[(640*239)+320].z);
}

//SERVICE --------------------------------->
bool extractService(rose_ui_overview_camera::extract_from_cloud::Request &req, rose_ui_overview_camera::extract_from_cloud::Response &res)
{

	bool success = false;

	rec_x1 = req.selection.x1;
	rec_x2 = req.selection.x2;

	rec_y1 = req.selection.y1;
	rec_y2 = req.selection.y2;

	int cloud_width = cloud2.width;
	int cloud_height = cloud2.height;	
	ROS_INFO("SERVICE CALL: Width: %d 	Height: %d", cloud_width, cloud_height);
	pcl::PointCloud<pcl::PointXYZ> cloudPt;
	pcl::fromROSMsg (cloud2, cloudPt);

	if(true) //if(received_rect)
	{
		pcl::PointCloud<pcl::PointXYZ> object_cloud;
		ROS_INFO("Check 1");
		if(rec_x2 <= rec_x1 || rec_y2 <= rec_y1) ROS_INFO("Invalid rect");
		else
		{
			ROS_INFO("Check 2");
			int wide = rec_x2 - rec_x1; 
			int high = rec_y2 - rec_y1;

			object_cloud.width = wide;
			object_cloud.height = high;
			object_cloud.is_dense = false;
			object_cloud.points.resize (object_cloud.width * object_cloud.height);

			float minX, maxX, minY, maxY, x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_4, y_4, z_4;
			minX = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].x; 
			maxX = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].x; 
			minY = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].y; 
			maxY = 0.0; //cloudPt.points[640*((rec_y1)) + (rec_x1)].y;

			for(int i = 0; i < high; i++)
			{				
				for(int j = 0; j < wide; j++ )
				{
					float tempx = cloudPt.points[640*((rec_y1 + i)) + (rec_x1 + j)].x;
					float tempy = cloudPt.points[640*((rec_y1 + i)) + (rec_x1 + j)].y;
					
					if(tempx > -2.0 && 
						tempx < 2.0){ minX = tempx; maxX = minX;}
					if(tempy > -2.0 && 
						tempy < 2.0){ minY = tempy; maxY = minY;}
				}
			}
			ROS_INFO("Check 2");
			ROS_INFO("INIT VAL (%f,%f,%f,%f)",minX,maxX,minY,maxY);

			for(int i = 0; i < high; i++)
			{				
				for(int j = 0; j < wide; j++ )
				{
					object_cloud.points[wide*i+j].x = cloudPt.points[cloud_width*((rec_y1 + i)) + (rec_x1 + j)].x; //cloud_width was 640
					object_cloud.points[wide*i+j].y = cloudPt.points[cloud_width*((rec_y1 + i)) + (rec_x1 + j)].y;
					object_cloud.points[wide*i+j].z = cloudPt.points[cloud_width*((rec_y1 + i)) + (rec_x1 + j)].z;
					
				}
			}
			ROS_INFO("Check 3");
			//pcl::io::savePCDFileASCII ("test_pcd.pcd", object_cloud);
			//ROS_INFO ("Saved %d data points to test_pcd.pcd.", (int)object_cloud.points.size ());

			int centerx = (wide/2.0);
			int centery = (high/2.0);
			bool left = true;
			bool right = true;
			for(int lindex = 1; lindex < wide/2.0; lindex++)
			{
				int ind = (wide * high) / 2.0;
				if(left && object_cloud.points[ind - lindex].z > 0.15)// && object_cloud.points[ind - lindex].z < 2.0 )
				{
					x_1 = object_cloud.points[ind - lindex].x;
					y_1 = object_cloud.points[ind - lindex].y;
					z_1 = object_cloud.points[ind - lindex].z;
					ROS_INFO("hey left");
					left = false;
				}

				if(right && object_cloud.points[ind + lindex].z > 0.15)// && object_cloud.points[ind + lindex].z < 2.0)
				{
					x_2 = object_cloud.points[ind + lindex].x;
					y_2 = object_cloud.points[ind + lindex].y;
					z_2 = object_cloud.points[ind + lindex].z;
					ROS_INFO("hey right");
					right = false;
				}
			}
			ROS_INFO("Check 4");
			////////////////////////////// ADDING /////////////////////////////
			ROS_INFO("--------------UPPER POINT DETECTION---------");
			bool uleft = true;
			bool bright = true;
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide/2.0);
				if(uleft && object_cloud.points[ind + lindex].z > 0.15)
				{
					lx = object_cloud.points[ind + lindex].x;
					ly = object_cloud.points[ind + lindex].y;
					lz = object_cloud.points[ind + lindex].z;
					ROS_INFO("good point @ top");
					uleft = false;
				}
			}
			ROS_INFO("--------------LOWER POINT DETECTION---------");
			ROS_INFO("Check 5");
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide * (high-1)) + (wide/2.0);

				if(bright && object_cloud.points[ind - lindex].z > 0.15)
				{
					rx = object_cloud.points[ind - lindex].x;
					ry = object_cloud.points[ind - lindex].y;
					rz = object_cloud.points[ind - lindex].z;
					ROS_INFO("good point @ bottom");
					bright = false;
				}

			}
			ROS_INFO("--------------LEFT POINT DETECTION---------");
			bool wleft = true;
			bool wright = true;
			ROS_INFO("Check 6");
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide*(high/2.0)) + 1;
				if(wleft && object_cloud.points[ind + lindex].z > 0.15)
				{
					wlx = object_cloud.points[ind + lindex].x;
					wly = object_cloud.points[ind + lindex].y;
					wlz = object_cloud.points[ind + lindex].z;
					ROS_INFO("good point @ left");
					wleft = false;
				}
			}
			ROS_INFO("--------------RIGHT POINT DETECTION---------");
			ROS_INFO("Check 6");
			for(int lindex = 1; lindex < wide/4.0; lindex++)
			{
				int ind = (wide*(high/2.0)) + (wide - 1);

				if(wright && object_cloud.points[ind - lindex].z > 0.15)
				{
					wrx = object_cloud.points[ind - lindex].x;
					wry = object_cloud.points[ind - lindex].y;
					wrz = object_cloud.points[ind - lindex].z;
					ROS_INFO("good point @ right");
					wright = false;
				}

			}
			///////////////////////////////////////////////////////////////////
			ROS_INFO("Check 7");
			// Added IF construct remove for old state
			if(left || right || uleft || bright || wleft || wright)
			{		
				std::stringstream mode_str;
				mode_str<<"Please select again..";
				std_msgs::String msgs;
				msgs.data = mode_str.str();
				point_sel_stat_pub.publish(msgs);
				ROS_INFO("Told user to select another point");
				success = false;

			      //std_msgs::Int32 comm;
			      //comm.data = 50;
			      // Announce bad point
			      //speaker_pub.publish(comm);
			}
			else
			{
				ROS_INFO("Check 8");
				std::stringstream mode_str;
				mode_str<<"";
				std_msgs::String msgs;
				msgs.data = mode_str.str();
				point_sel_stat_pub.publish(msgs);
				ROS_INFO("Told user the point is good");

				ROS_INFO("MIN_X: %f, MAX_X: %f, MIN_Y: %f, MAX_Y: %f", minX,maxX,minY,maxY);
				ROS_INFO("left points (%f, %f, %f)", x_1, y_1, z_1);
				ROS_INFO("right points (%f, %f, %f)", x_2, y_2, z_2);
				//ROS_INFO("upper points (%f, %f, %f)", x_3, y_3, z_3);
				//ROS_INFO("lower points (%f, %f, %f)", x_4, y_4, z_4);
				ROS_INFO("AVERAGE DIST: %f",(z_1 + z_2)/2.0);
				ROS_INFO("New point: (%f, %f, %f)",(x_1 + x_2)/2.0, (y_1 + y_2)/2.0,(z_1 + z_2)/2.0);

				ROS_INFO("upper corner points (%f, %f, %f)", lx,ly,lz);
				ROS_INFO("lower corner points (%f, %f, %f)", rx,ry,rz);
				ROS_INFO("left corner points (%f, %f, %f)", wlx,wly,wlz);
				ROS_INFO("right corner points (%f, %f, %f)", wrx,wry,wrz);

				float obj_height = sqrt(pow(lx - rx,2.0) + pow(ly - ry,2.0) + pow(lz - rz,2.0));
				ROS_INFO("Object Height: %f",obj_height);
				float obj_width = sqrt(pow(wlx - wrx,2.0) + pow(wly - wry,2.0) + pow(wlz - wrz,2.0));
				ROS_INFO("Object width: %f",obj_width);

			        //std_msgs::Int32 comm;
				//comm.data = 51;
				// Announce good point
				//speaker_pub.publish(comm);

				sendPointS.header.stamp = ros::Time::now();
				sendPointS.point.x = (x_1 + x_2)/2.0;
				sendPointS.point.y = (y_1 + y_2)/2.0;
				sendPointS.point.z = (z_1 + z_2)/2.0;
				SPoint_pub.publish(sendPointS);

				res.objpose.position.x = (x_1 + x_2)/2.0;
				res.objpose.position.y = (y_1 + y_2)/2.0;
				res.objpose.position.z = (z_1 + z_2)/2.0;
				//res.pose.yaw = 0.0;
				//res.pose.pitch = 0.0;
				//res.pose.roll = 0.0;
				//res.pose.gripper = 0.0;

				// obj.cx = (x_1 + x_2)/2.0;
				// obj.cx = (y_1 + y_2)/2.0;
				// obj.cz = (z_1 + z_2)/2.0;

				// obj.ux = lx;				
				// obj.uy = ly;
				// obj.uz = lz;

				// obj.bx = rx;				
				// obj.by = ry;
				// obj.bz = rz;

				// obj.lx = wlx;				
				// obj.ly = wly;
				// obj.lz = wlz;

				// obj.rx = wrx;				
				// obj.ry = wry;
				// obj.rz = wrz;

				// obj.width = obj_width;
				// obj.height = obj_height;
				//object_pub.publish(obj);
				success = true;
			}
		}
	}

	received_rect = false;
	//ROS_INFO("Cloud size: %d",cloudPt.points.size());
	//ROS_INFO("Test: %f",cloud_width,cloud_height);
	//ROS_INFO("Test data X : %f, Y : %f, Z : %f",cloudPt.points[(640*239)+320].x, cloudPt.points[(640*239)+320].y, cloudPt.points[(640*239)+320].z);

	return success;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"rect_tracker");
	ros::NodeHandle n;


  	// Loop rate
  	ros::Rate r(20);

	//////////////////////////////////UNCOMMENT///////////////////////////////////////////////////////////
	// Point selection status publisher -- Publishes to LUI the status of the selected point
	point_sel_stat_pub = n.advertise<std_msgs::String>("point_selection_status", 100);

	//ros::Subscriber sub_points2 = n.subscribe("/stereoVidere/points2",100,points2Callback);
	ros::Subscriber sub_points2 = n.subscribe("/camera/depth_registered/points",100,points2Callback);
	ros::Subscriber rect_sub = n.subscribe("/overview_camera/selection",100,rectCallback);
	
	// Speaker publisher
	//speaker_pub = n.advertise<std_msgs::Int32>("speak_command",10);

	// This is the X , Y, Z coordinates of the point selected in the point cloud.
	SPoint_pub = n.advertise<geometry_msgs::PointStamped>("img_point_goal",10);

	//object_pub = n.advertise<usb_cam::object>("object_feature",10);

	sendPointS.header.frame_id = "openni_rgb_optical_frame";


	ros::ServiceServer extract_service = n.advertiseService("extract_from_cloud", extractService);
	//////////////////////////////////UNCOMMENT///////////////////////////////////////////////////////////

        while (n.ok())
	{ 
		ros::spinOnce();
		r.sleep();
	}

	return 0;

}


/*
					if(object_cloud.points[wide*i+j].x < minX && object_cloud.points[wide*i+j].x != NULL
						&& object_cloud.points[wide*i+j].z != 0.0 && object_cloud.points[wide*i+j].z < 2.0)
					{ 
						minX = object_cloud.points[wide*i+j].x; 
						x_1 = object_cloud.points[wide*i+j].x;
						y_1 = object_cloud.points[wide*i+j].y;
						z_1 = object_cloud.points[wide*i+j].z;
					}
					if(object_cloud.points[wide*i+j].x > maxX && object_cloud.points[wide*i+j].x != NULL
						&& object_cloud.points[wide*i+j].z != 0.0 && object_cloud.points[wide*i+j].z < 2.0) 
					{
						maxX = object_cloud.points[wide*i+j].x;
						x_2 = object_cloud.points[wide*i+j].x;
						y_2 = object_cloud.points[wide*i+j].y;
						z_2 = object_cloud.points[wide*i+j].z;
					}
					if(object_cloud.points[wide*i+j].y < minY && object_cloud.points[wide*i+j].y != NULL
						&& object_cloud.points[wide*i+j].z != 0.0 && object_cloud.points[wide*i+j].z < 2.0) 
					{ 
						minY = object_cloud.points[wide*i+j].y; 
						x_3 = object_cloud.points[wide*i+j].x;
						y_3 = object_cloud.points[wide*i+j].y;
						z_3 = object_cloud.points[wide*i+j].z;

					}
					if(object_cloud.points[wide*i+j].y > maxY && object_cloud.points[wide*i+j].y != NULL
						&& object_cloud.points[wide*i+j].z != 0.0 && object_cloud.points[wide*i+j].z < 2.0) 
					{
						maxY = object_cloud.points[wide*i+j].y;
						x_4 = object_cloud.points[wide*i+j].x;
						y_4 = object_cloud.points[wide*i+j].y;
						z_4 = object_cloud.points[wide*i+j].z;

					}

*/
