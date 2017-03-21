/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <iostream>
#include <fstream>

#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>

#include <pcl/PointIndices.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct PointFeatureHistogram
{
  spore<pregrasp_msgs::GraspStrategyArray> pregrasp_messages_;
  spore<std::vector< ::pcl::PointIndices> > point_indices_;
  //spore<std::vector< ::pcl::PFHSignature125> point_feature_histograms_;

  static void declare_params(ecto::tendrils& params)
  {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<pregrasp_msgs::GraspStrategyArray>("pregrasp_messages", "The pregrasps define the bounding boxes which are used for cropping.");
    inputs.declare<std::vector< ::pcl::PointIndices> >("point_indices", "Indices of all points that fall within the bounding box of the corresponding pregrasp.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    pregrasp_messages_ = inputs["pregrasp_messages"];
    point_indices_ = inputs["point_indices"];
  }

  template<typename Point>
  void computeCEDescriptor(
      ::tf::Transform query,
      boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
      boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals,
      const std::vector<int>& indices,
      Eigen::VectorXf& histogram)
  {
    float query_x = query.getOrigin().x();
    float query_y = query.getOrigin().y();
    float query_z = query.getOrigin().z();

    float query_approach_x = -query.getBasis().getColumn(2).x();
    float query_approach_y = -query.getBasis().getColumn(2).y();
    float query_approach_z = -query.getBasis().getColumn(2).z();

    float query_up_x = query.getBasis().getColumn(1).x();
    float query_up_y = query.getBasis().getColumn(1).y();
    float query_up_z = query.getBasis().getColumn(1).z();

    int h_index, h_p;
    int nr_split = 5;
    int feature_index[3];

    histogram.setZero ();
    float hist_incr = 1.0;//100.0f / static_cast<float> (indices.size () * (indices.size () - 1) / 2);

    for (size_t i = 0; i < indices.size(); ++i)
    {
      // calculate d
      float dx = input->at(indices[i]).x - query_x;
      float dy = input->at(indices[i]).y - query_y;
      float dz = input->at(indices[i]).z - query_z;
      float d = sqrt(dx*dx + dy*dy + dz*dz);

      // calculate alpha
      float alpha = query_approach_x*normals->at(indices[i]).normal_x + query_approach_y*normals->at(indices[i]).normal_y + query_approach_z*normals->at(indices[i]).normal_z;

      // calculate beta
      float beta = query_up_x*normals->at(indices[i]).normal_x + query_up_y*normals->at(indices[i]).normal_y + query_up_z*normals->at(indices[i]).normal_z;

      // normalize features
      feature_index[0] = static_cast<int> (floor (nr_split * ((alpha + M_PI_2) * 1.0f / M_PI)));
      if (feature_index[0] < 0)         feature_index[0] = 0;
      if (feature_index[0] >= nr_split) feature_index[0] = nr_split - 1;

      feature_index[1] = static_cast<int> (floor (nr_split * ((beta + M_PI_2) * 1.0f / M_PI)));
      if (feature_index[1] < 0)         feature_index[1] = 0;
      if (feature_index[1] >= nr_split) feature_index[1] = nr_split - 1;

//      f_index_[2] = static_cast<int> (floor (nr_split * ((pfh_tuple_[2] + 1.0) * 0.5)));

      feature_index[2] = static_cast<int> (floor (nr_split * (d / 0.1)));
      if (feature_index[2] < 0)         feature_index[2] = 0;
      if (feature_index[2] >= nr_split) feature_index[2] = nr_split - 1;

      // copy into the appropriate histogram bin
      h_index = 0;
      h_p     = 1;
      for (int d = 0; d < 3; ++d)
      {
        h_index += h_p * feature_index[d];
        h_p     *= nr_split;
      }
      histogram[h_index] += hist_incr;
    }
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
  {
    std::cout << "hallo" << std::endl;
    assert(point_indices_->size() == pregrasp_messages_->strategies.size());

    std::cout << "there are " << pregrasp_messages_->strategies.size() << " things" << std::endl;

    for (size_t i = 0; i < pregrasp_messages_->strategies.size(); ++i)
    {
      ::pcl::PointCloud< ::pcl::PFHSignature125>::Ptr pfhs (new ::pcl::PointCloud< ::pcl::PFHSignature125> ());

      typename ::pcl::PFHEstimation< Point, ::pcl::Normal, ::pcl::PFHSignature125> pfh;
      //typename ::pcl::FPFHEstimation< Point, ::pcl::Normal, ::pcl::FPFHSignature33> fpfh;
      //typename ::pcl::VFHEstimation< Point, ::pcl::Normal, ::pcl::VFHSignature308> vfh;

      std::cout << "before the histogram " << std::endl;

      Eigen::VectorXf histogram(125);

      ::tf::Transform hand_pose;
      ::tf::poseMsgToTF(pregrasp_messages_->strategies[i].pregrasp_pose.pose.pose, hand_pose);
      computeCEDescriptor(hand_pose, input, normals, point_indices_->at(i).indices, histogram);
//      pfh.computePointPFHSignature(*input, *normals, point_indices_->at(i).indices, 5, histogram);

      std::ofstream gnuplot("test.dat");

      for (size_t j = 0; j < histogram.size(); ++j)
      {
        float alpha_res = M_PI / 5.0;
        float d_res = 0.1f / 5.0;
        //gnuplot << histogram[j] << std::endl;
        int a = j % 5;
        int b = (j / 5) % 5;
        int d = j / 25;
        gnuplot << a * alpha_res - M_PI_2 + alpha_res/2.0 << " " << b * alpha_res - M_PI_2 + alpha_res/2.0 << " " << d * d_res << " " << histogram[j] << std::endl;
      }

      gnuplot.close();
    }

    return OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals< ecto_rbo_grasping::PointFeatureHistogram >, "PointFeatureHistogram", "Compute the point feature histogram.");
