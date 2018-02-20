/**
    main.cpp

    Purpose: a ROS node to downsample a pointcloud for integrating Kinect One with the TUB EC Grasp Planner
    This code downsamples a generic resolution point cloud to a any lower resolution (x_res * y_res) keeping
    the PointCloud2 organized and pubishes it to a topic.

    Input Topic: 	/kinect2/sd/points
    Output Topic: 	/down_sampled_points

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>

// PCL INCLUDES
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OTHER INCLUDES
#include <cmath>

#define DEBUG			0										// if DEBUG 1 prints additional couts

// GLOBAL VARIABLES
ros::Publisher pub_down_points; 								// publisher for downsampled point cloud
int x_res = 320;
int y_res = 240;

// CALLBACK FUNCTION
void downsample_and_publish_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud){

  	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

	// Convert to PCL data type
	pcl_conversions::toPCL(*input_cloud, *cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> down_cloud;
    pcl::fromPCLPointCloud2(*cloud,*temp_cloud);

	// Checking if downsamplable
    if(temp_cloud->width < x_res || temp_cloud->height < y_res){
    	ROS_ERROR_STREAM("Desired resolution is bigger than original pointcloud dimensions!");
    }

    // Filtering using iterator
	int x_skip = std::floor(temp_cloud->width / x_res);			// x skip while row downsampling
	int y_skip = std::floor(temp_cloud->height / y_res);		// y skip while column downsampling

	int x_margin = (temp_cloud->width - x_skip * x_res) / 2;	// number of points to skip from left and right
	int y_margin = (temp_cloud->height - y_skip * y_res) / 2;	// number of points to skip from up and down

	// First push all columns (x / width) in a row (y / height) then change row
	for(int j = y_margin; j < (y_margin + y_skip * y_res); j+=y_skip){
	    for(int i = x_margin; i < (x_margin + x_skip * x_res); i+=x_skip){
	        down_cloud.push_back(temp_cloud->at(i,j));
	    }
	}

	if(DEBUG){
		std::cout << "Cloud just sampled has width " << down_cloud.width << " and height " << down_cloud.height << "!" << std::endl;
	}

	// Setting correctly width and height
	//down_cloud.resize(x_res * y_res); 						// not necessary now
	down_cloud.width = x_res;
	down_cloud.height = y_res;
	
  	// Convert to ROS data type
  	pcl::PCLPointCloud2 out_cloud;
  	pcl::toPCLPointCloud2(down_cloud, out_cloud);
	sensor_msgs::PointCloud2 output_pc;
	pcl_conversions::fromPCL(out_cloud, output_pc);

	// Write frame info to msg
	output_pc.width = down_cloud.width;
	output_pc.height = down_cloud.height;

	if(DEBUG){
		std::cout << "Cloud to be published has width " << down_cloud.width << " and height " << down_cloud.height << "!" << std::endl;
	}

	// Setting other message fields
	output_pc.header.stamp = input_cloud->header.stamp;
	output_pc.header.frame_id = input_cloud->header.frame_id;
	output_pc.is_dense = input_cloud->is_dense;

	// Publish the data
	pub_down_points.publish(output_pc);

	delete cloud;

}

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "pc_downsampler");
	ros::NodeHandle pc_nh;

	// Creating a ROS subscriber for the input point cloud
	ros::Subscriber sub = pc_nh.subscribe("input_topic", 1, downsample_and_publish_cb);

	// Creating a ROS publisher for the output point cloud
	pub_down_points = pc_nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1);

	// Success message
	std::cout << "Downscaled points (to " << x_res << "x" << y_res << ") are being published from input_topic to output_topic!" << std::endl;

	// Spin
	ros::spin ();

}
