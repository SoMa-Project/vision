/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 


#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/search/search.h>
#include <pcl/search/organized.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/region_growing_rgb.h>

namespace ecto_rbo_pcl
{

struct RegionGrowingRGB
{
  ecto::spore< ::pcl::PointIndices> indices_;
  ecto::spore<ecto::pcl::Clusters> output_;

  ecto::spore<double> distance_threshold_;
  ecto::spore<double> point_color_threshold_;
  ecto::spore<double> region_color_threshold_;
  ecto::spore<int> min_cluster_size_;

  static void declare_params(ecto::tendrils& params) {
      params.declare<double>("distance_threshold", "Used to determine whether the point is neighbouring or not. If the point is located at a distance less than the given threshold, then it is considered to be neighbouring. Used for clusters neighbours search.", 10.0);
      params.declare<double>("point_color_threshold", "Color threshold. Just as angle threshold is used for testing points normals in pcl::RegionGrowing to determine if the point belongs to cluster, this value is used for testing points colors.", 6.0);
      params.declare<double>("region_color_threshold", "Color threshold for clusters. This value is similar to the previous, but is used when the merging process takes place.", 5.0);
      params.declare<int>("min_cluster_size", "This value is similar to that which was used in the Region growing segmentation tutorial. In addition to that, it is used for merging process. If cluster has less points than was set through setMinClusterSize method, then it will be merged with the nearest neighbour.", 10);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
      inputs.declare< ::pcl::PointIndices >("indices", "Point indices to consider (usually used to avoid NaNs in an organized pointcloud.");
      outputs.declare< ecto::pcl::Clusters >("output", "Clusters indices.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
      distance_threshold_ = params["distance_threshold"];
      point_color_threshold_ = params["point_color_threshold"];
      region_color_threshold_ = params["region_color_threshold"];
      min_cluster_size_ = params["min_cluster_size"];

      indices_ = inputs["indices"];
      output_ = outputs["output"];
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs, boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
        ::pcl::PointCloud< ::pcl::PointXYZRGB >::Ptr new_input(new ::pcl::PointCloud< ::pcl::PointXYZRGB >);
        ::pcl::copyPointCloud(*input, *new_input);

        //::pcl::search::Search < ::pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr< ::pcl::search::Search< ::pcl::PointXYZRGB> > (new ::pcl::search::KdTree< ::pcl::PointXYZRGB>());
//        ::pcl::search::Search < ::pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr< ::pcl::search::OrganizedNeighbor< ::pcl::PointXYZRGB> > (new ::pcl::search::OrganizedNeighbor< ::pcl::PointXYZRGB>());

        ::pcl::RegionGrowingRGB< ::pcl::PointXYZRGB > rg;
        rg.setInputCloud(new_input);
        if (indices_.user_supplied())
            rg.setIndices(boost::shared_ptr< std::vector<int> >(new std::vector<int>(indices_->indices)));
//        rg.setSearchMethod(tree);
        rg.setDistanceThreshold(*distance_threshold_);
        rg.setPointColorThreshold(*point_color_threshold_);
        rg.setRegionColorThreshold(*region_color_threshold_);
        rg.setMinClusterSize(*min_cluster_size_);
        std::vector < ::pcl::PointIndices> clusters;
        rg.extract (clusters);

//      //::pcl::RegionGrowingRGB< ::pcl::PointXYZ, ::pcl::Normal> region_growing;
//        ::pcl::RegionGrowingRGB< ::pcl::PointXYZ> region_growing;
//      region_growing.setSmoothMode(true);
//      region_growing.setSmoothnessThreshold(10.0f / 180.0f * static_cast<float>(M_PI));
//      //  rg.setCurvatureTest(true);
//      //  rg.setCurvatureThreshold(0.02);
//      //  rg.setResidualTest(true);
//      //  rg.setResidualThreshold(0.02);

//      ::pcl::PointCloud< ::pcl::PointXYZ>::Ptr new_input(new ::pcl::PointCloud< ::pcl::PointXYZ>);
//      ::pcl::copyPointCloud(*input, *new_input);
//      region_growing.setCloud(new_input);
//      region_growing.setNormals(normals);
////      boost::shared_ptr< ::pcl::search::Search< ::pcl::PointXYZ> > search(new ::pcl::search::KdTree< ::pcl::PointXYZ>());
////      search = boost::shared_ptr< ::pcl::search::Search< ::pcl::PointXYZ> > ;
////      region_growing.setNeighbourSearchMethod(search);

//      int segments = region_growing.segmentPoints();

      ROS_INFO_STREAM("RegionGrowingRGB found " << clusters.size() << " segments.");
      *output_ = clusters;

//      ::ros::NodeHandle handle_;
//      sensor_msgs::PointCloud2Ptr color_msg(new sensor_msgs::PointCloud2);
//      ::pcl::toROSMsg(*(region_growing.getColoredCloud()), *color_msg);
//      color_msg->header = pcl_conversions::fromPCL(input->header);
//      color_msg->header.frame_id = "/base_link";

//      static ros::NodeHandle nh;
//      static ::ros::Publisher segmentation_publisher = nh.advertise<sensor_msgs::PointCloud2>("/rgdbtest/segments", 1);

//      segmentation_publisher.publish(*color_msg);

//      *clusters2_ = region_growing.getSegments();

      return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell< ecto_rbo_pcl::RegionGrowingRGB >, "RegionGrowingRGB", "Do the region growing segmentation algorithm.");

