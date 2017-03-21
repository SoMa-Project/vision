#include <ros/console.h>

#include <ecto_rbo_pcl/common.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <ros/ros.h>

using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

struct ExtractClosestCluster
{
  static void declare_params(tendrils& params)
  {
    params.declare<double>("size_weight", "If set, distance will be influenced by the cluster size by this factor (size_weight < 0: prefer larger clusters that still are further away)").required(false);
  }
  
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare<Clusters>("clusters", "Clusters indices.");
    inputs.declare<UnalignedAffine3f>("transform", "Pose to which the closest cluster (centroid) is selected.").required(false);
    inputs.declare<UnalignedAffine3f>("transform_2", "Another pose. If this is given the closest cluster is defined as the one which minimizes: |Pose1 - X| + |Pose2 - x|").required(false);
    outputs.declare<PointCloud>("output", "Filtered Cloud.");
    outputs.declare<Clusters>("closest_cluster", "Closest cluster indices.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
    clusters_ = inputs["clusters"];
    transform_ = inputs["transform"];
    transform_2_ = inputs["transform_2"];
    
    output_ = outputs["output"];
    closest_cluster_ = outputs["closest_cluster"];
    
    size_weight_ = params["size_weight"];
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
      ROS_DEBUG("ExtractClosestCluster.cpp: No. of clusters to choose from: %zu", clusters_->size());
      if (clusters_->empty()) {
        ROS_WARN("ExtractClosestCluster.cpp: No clusters to choose from!");

        /*typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        cloud->header = input->header;
        cloud->resize(0);
        *output_ = xyz_cloud_variant_t(cloud);*/

        return ecto::OK;
      }

      ::pcl::ExtractIndices<Point> filter;
      int closest = 0;
      float closest_distance = FLT_MAX;
      for (size_t i = 0; i < clusters_->size(); i++)
      {
//        std::cerr << "Cluster " << i << " has " << (*clusters_)[i].indices.size() << "  points." << std::endl;

        Eigen::Vector4f centroid;
        ::pcl::compute3DCentroid(*input, (*clusters_)[i], centroid);
        float distance_i;

        if (transform_.user_supplied())
        {
            distance_i = (centroid.head<3>() - transform_->translation()).squaredNorm(); // before: norm()
            if (transform_2_.user_supplied())
                distance_i += (centroid.head<3>() - transform_2_->translation()).squaredNorm();
        }
        else {
            distance_i = centroid.norm();
        }
        
        if (size_weight_.user_supplied())
        {
            // e.g. if size_weight_ == 0.0001f: 1000 points count as much as 10 centimeters
            //std::cout << "Distance before: " << distance_i << " and after ";
            distance_i += *size_weight_ * (*clusters_)[i].indices.size();
            //std::cout << distance_i << std::endl;
        }
        
        if (distance_i < closest_distance)
        {
          closest = i;
          closest_distance = distance_i;
        }
      }
      filter.setIndices(::pcl::PointIndicesPtr(new ::pcl::PointIndices((*clusters_)[closest])));
      filter.setInputCloud(input);

      typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
      filter.filter(*cloud);
      cloud->header = input->header;
      *output_ = xyz_cloud_variant_t(cloud);

      closest_cluster_->clear();
      closest_cluster_->push_back((*clusters_)[closest]);

      return OK;
    }

  spore<Clusters> clusters_;
  spore<PointCloud> output_;
  spore<Clusters> closest_cluster_;
  spore<UnalignedAffine3f> transform_;
  spore<UnalignedAffine3f> transform_2_;
  spore<double> size_weight_;
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::ExtractClosestCluster>, "ExtractClosestCluster", "Extract a point cloud corresponding to the closest cluster.");

