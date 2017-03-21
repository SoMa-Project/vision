#include <pcl/segmentation/organized_connected_component_segmentation.h>

#include "ecto_rbo_pcl/curvature_comparator.h"
#include "ecto_rbo_pcl/edge_comparator.h"
#include "ecto_rbo_pcl/convex_edge_comparator.h"
#include "ecto_rbo_pcl/intensity_comparator.h"

//#include "organized_connected_component_segmentation.h"
//#include "organized_connected_component_segmentation.hpp"

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/PointIndices.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

struct FloodFill
{
  spore<int> min_inliers_;
  spore<double> angular_threshold_;
  spore<double> distance_threshold_;
  spore<double> normal_distance_threshold_;
  spore<double> curvature_threshold_;
  spore<double> curvature_distance_threshold_;

  spore<int> comparator_type_;
  
  spore<bool> publish_rviz_markers_;

  spore< std::vector< ::pcl::PointIndices > > clusters_;
  spore< ::pcl::PointIndices > inliers_;
  spore< ::pcl::PointIndices > unclustered_;

  static void declare_params(tendrils& params) {
      params.declare<double> ("distance_threshold", "Maximum distance between two neighboring points in the same cluster.", 0.05); //0.04
      params.declare<double> ("normal_distance_threshold", "Distance between two neighboring points along their normals.", 0.12); //0.02
      params.declare<double> ("angular_threshold", "Angular difference (in rad) between the normals of two neighboring points.", ::pcl::deg2rad(30.0)); //15
      params.declare<double> ("curvature_threshold", "Maximum curvature of a point.", 0.07); // 0.04
      params.declare<double> ("curvature_distance_threshold", "Difference in curvature between two neighboring points.", 0.07); //0.04
      params.declare<int> ("min_inliers", "Number of minimum points a cluster must contain.", 100);
      params.declare<int> ("comparator_type", "0 = curvature_comparator, 1 = edge_comparator, 2 = curvature_comparator", 0);
      params.declare<bool>("publish_rviz_markers", "Should the output be published for visualization?", false);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    outputs.declare< std::vector< ::pcl::PointIndices > >("clusters", "Clusters indices.");
    outputs.declare< ::pcl::PointIndices >("inliers", "Points that belong to some cluster.");
    outputs.declare< ::pcl::PointIndices >("unclustered", "Point indices that do not belong to any cluster.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
    clusters_ = outputs["clusters"];
    inliers_ = outputs["inliers"];
    unclustered_ = outputs["unclustered"];

    angular_threshold_ = params["angular_threshold"];
    distance_threshold_ = params["distance_threshold"];
    normal_distance_threshold_ = params["normal_distance_threshold"];
    curvature_threshold_ = params["curvature_threshold"];
    curvature_distance_threshold_ = params["curvature_distance_threshold"];
    min_inliers_ = params["min_inliers"];
    comparator_type_ = params["comparator_type"];
    publish_rviz_markers_ = params["publish_rviz_markers"];
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals) 
    {
    
      ROS_INFO_STREAM_NAMED("FloodFill", "Computing clusters ...");

      typename ::ecto_rbo_pcl::CurvatureComparator<Point, ::pcl::Normal>::Ptr comparator(new ::ecto_rbo_pcl::CurvatureComparator<Point, ::pcl::Normal>);
      comparator->setAngularThreshold(*angular_threshold_);
      comparator->setDistanceThreshold(*distance_threshold_);
      comparator->setNormalDistanceThreshold(*normal_distance_threshold_);
      comparator->setCurvatureThreshold(*curvature_threshold_);
      comparator->setCurvatureDistanceThreshold(*curvature_distance_threshold_);

      comparator->setInputCloud(input);
      comparator->setInputNormals(normals);

      ::pcl::OrganizedConnectedComponentSegmentation<Point, ::pcl::Label> connected_component_labeling(comparator);

      if (*comparator_type_ == 1) {
        typename ::ecto_rbo_pcl::EdgeComparator<Point, ::pcl::Normal>::Ptr edge_comparator(new ::ecto_rbo_pcl::EdgeComparator<Point, ::pcl::Normal>);
        edge_comparator->setDistanceThreshold(*distance_threshold_);
        edge_comparator->setCurvatureThreshold(*curvature_threshold_);
        edge_comparator->setCurvatureDistanceThreshold(*curvature_distance_threshold_);

        edge_comparator->setInputCloud(input);
        edge_comparator->setInputNormals(normals);

        connected_component_labeling.setComparator(edge_comparator);
      }
      else if (*comparator_type_ == 2) {
        typename ::ecto_rbo_pcl::ConvexEdgeComparator<Point, ::pcl::Normal>::Ptr convex_edge_comparator(new ::ecto_rbo_pcl::ConvexEdgeComparator<Point, ::pcl::Normal>);
        convex_edge_comparator->setDistanceThreshold(*distance_threshold_);
        convex_edge_comparator->setCurvatureThreshold(*curvature_threshold_);

        convex_edge_comparator->setInputCloud(input);
        convex_edge_comparator->setInputNormals(normals);

        connected_component_labeling.setComparator(convex_edge_comparator);
      }
      else if (*comparator_type_ == 3) {
        typename ::ecto_rbo_pcl::IntensityComparator<Point, ::pcl::Normal>::Ptr intensity_comparator(new ::ecto_rbo_pcl::IntensityComparator<Point, ::pcl::Normal>);
        intensity_comparator->setDistanceThreshold(*distance_threshold_);
        intensity_comparator->setCurvatureThreshold(*curvature_threshold_);
        intensity_comparator->setCurvatureDistanceThreshold(*curvature_distance_threshold_);

        intensity_comparator->setInputCloud(input);
        intensity_comparator->setInputNormals(normals);

        connected_component_labeling.setComparator(intensity_comparator);
      }
      connected_component_labeling.setInputCloud(input);

      ::pcl::PointCloud< ::pcl::Label> labels;
      std::vector< ::pcl::PointIndices> label_indices;

      connected_component_labeling.segment(labels, label_indices);

//      int segments = region_growing.segmentPoints();
      size_t total = 0;
      for (size_t i = 0; i < label_indices.size(); ++i) {
//        ROS_STREAM_NAMED("FloodFill", "     %zu points", label_indices[i].indices.size());
        total += label_indices[i].indices.size();
      }
      //ROS_INFO("segments: %zu out of %zu points with ", label_indices.size(), input->size());
      //ROS_INFO("in total: %zu", total);
//      ROS_INFO("First point finite: %i (%f %f %f)", pcl::isFinite(input->points[0]), input->points[0].x, input->points[0].y, input->points[0].z);
      ROS_DEBUG_NAMED("FloodFill", "segments: %zu out of %zu points with ", label_indices.size(), input->size());
      ROS_DEBUG_NAMED("FloodFill", "in total: %zu", total);
//      ROS_STREAM_NAMED("FloodFill", "First point finite: %i (%f %f %f)", pcl::isFinite(input->points[0]), input->points[0].x, input->points[0].y, input->points[0].z);

      ::pcl::PointCloud< ::pcl::PointXYZRGB>::Ptr colored_cloud(new ::pcl::PointCloud< ::pcl::PointXYZRGB>);
      ::pcl::copyPointCloud<Point, ::pcl::PointXYZRGB>(*input, *colored_cloud);

      unsigned char red [12] = {255,   0,   0, 255, 255,   0, 127,   0,   0, 127, 127,   0};
      unsigned char grn [12] = {  0, 255,   0, 255,   0, 255,   0, 127,   0, 127,   0, 127};
      unsigned char blu [12] = {  0,   0, 255,   0, 255, 255,   0,   0, 127,   0, 127, 127};

      /*
      for (int h = 0; h < colored_cloud->height; ++h) {
        for (int w = 0; w < colored_cloud->width; ++w) {
          int index = h * colored_cloud->width + w;
          colored_cloud->at(index).r = (colored_cloud->width - w) * 255.0 / colored_cloud->width;
          colored_cloud->at(index).g = h * 255.0 / colored_cloud->height;
          colored_cloud->at(index).b = 0;
        }
      }
      */

      for (size_t i = 0; i < labels.size(); ++i) {
            int color = labels.at(i).label;
            colored_cloud->at(i).r = red[color % 12];
            colored_cloud->at(i).g = grn[color % 12];
            colored_cloud->at(i).b = blu[color % 12];
      }

      clusters_->clear();
      unclustered_->indices.clear();
      inliers_->indices.clear();
      for (std::vector< ::pcl::PointIndices>::const_iterator it = label_indices.begin(); it != label_indices.end(); ++it) {
    	  if (it->indices.size() > *min_inliers_) {
//                  clusters2_->push_back(it->indices);
                  //std::cout << " " << it->indices.size();
    		  ::pcl::PointIndices ind;
    		  ind.header = it->header;
    		  ind.indices = it->indices;
    		  clusters_->push_back(ind);
    		  inliers_->indices.insert(inliers_->indices.end(), it->indices.begin(), it->indices.end());
    		  //std::copy(it->indices.begin(), it->indices.end(), )
    	  }
    	  else {
            unclustered_->header = it->header;
    	    unclustered_->indices.insert(unclustered_->indices.end(), it->indices.begin(), it->indices.end());
    	  }
      }

      //ROS_INFO("Big clusters: %zu    Points in clusters: %zu / %zu", clusters_->size(), inliers_->indices.size(), input->size());
      ROS_INFO_NAMED("FloodFill", "Big clusters: %zu    Points in clusters: %zu / %zu", clusters_->size(), inliers_->indices.size(), input->size());

      if (*publish_rviz_markers_)
      {
          ::pcl::ExtractIndices< ::pcl::PointXYZRGB> filter;
          ::pcl::IndicesPtr indices_to_be_displayed = boost::make_shared<std::vector<int> >(inliers_->indices);
          filter.setIndices(indices_to_be_displayed);
          filter.filterDirectly(colored_cloud);

          sensor_msgs::PointCloud2Ptr color_msg(new sensor_msgs::PointCloud2);
          ::pcl::toROSMsg(*colored_cloud, *color_msg);

          color_msg->header.frame_id = input->header.frame_id;
          color_msg->header.stamp = ::ros::Time::now();
          
          static ::ros::NodeHandle nh;
          static ::ros::Publisher segmentation_publisher = nh.advertise<sensor_msgs::PointCloud2>("flood_fill_segmentation", 1);
          
          segmentation_publisher.publish(*color_msg);
      }
      
      return OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCellWithNormals< ecto_rbo_pcl::FloodFill >, "FloodFill", "Does the flood fill segmentation algorithm.");

