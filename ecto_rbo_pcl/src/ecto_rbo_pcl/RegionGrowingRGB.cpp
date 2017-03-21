
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
      params.declare<double>("distance_threshold", "ddd", 10.0);
      params.declare<double>("point_color_threshold", "ddd", 6.0);
      params.declare<double>("region_color_threshold", "ddd", 5.0);
      params.declare<int>("min_cluster_size", "", 10);
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

