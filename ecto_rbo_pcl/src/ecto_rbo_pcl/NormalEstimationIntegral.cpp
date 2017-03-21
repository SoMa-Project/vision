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

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace ecto;

namespace ecto_rbo_pcl
{
    struct NormalEstimationIntegral
    {
      static void declare_params(ecto::tendrils& params)
      {
        params.declare<int> (&NormalEstimationIntegral::estimation_method_, "estimation_method", "The estimation method to use: COVARIANCE_MATRIX(0), AVERAGE_3D_GRADIENT(1), AVERAGE_DEPTH_CHANGE(2).", 0);
        params.declare<double> (&NormalEstimationIntegral::max_depth_change_factor_, "max_depth_change_factor", "The max depth change factor.", 0.01f);
        params.declare<double> (&NormalEstimationIntegral::smoothing_size_, "smoothing_size", "The smoothing size.", 15.0f);
        params.declare<bool>(&NormalEstimationIntegral::publish_rviz_markers_, "publish_rviz_markers", "Should the output be published for visualization?", false);
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<ecto::pcl::FeatureCloud> ("output", "Cloud of features.");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        output_ = outputs["output"];
      }

      template <typename Point>
      int process(const tendrils& inputs, const tendrils& outputs,
                  boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
      {
        ::pcl::IntegralImageNormalEstimation<Point, ::pcl::Normal> impl;
        typename ::pcl::PointCloud< ::pcl::Normal >::Ptr normals(new typename ::pcl::PointCloud< ::pcl::Normal >);

        switch (*estimation_method_)
        {
          case 0:
          {
            impl.setNormalEstimationMethod(impl.COVARIANCE_MATRIX);
            break;
          }
          case 1:
          {
            impl.setNormalEstimationMethod(impl.AVERAGE_3D_GRADIENT);
            break;
          }
          case 2:
          {
            impl.setNormalEstimationMethod(impl.AVERAGE_DEPTH_CHANGE);
            break;
          }
        }

        impl.setMaxDepthChangeFactor(*max_depth_change_factor_);
        impl.setNormalSmoothingSize(*smoothing_size_);
        impl.setInputCloud(input);
        impl.compute(*normals);

        normals->header = input->header;
        *output_ = ecto::pcl::feature_cloud_variant_t(normals);

        if (*publish_rviz_markers_)
        {
            static ros::NodeHandle nh;
            static ros::Publisher normal_pub = nh.advertise<sensor_msgs::PointCloud2>("/normals", 10);
            
            ::pcl::PointCloud< ::pcl::PointXYZRGB>::Ptr normal_cloud(new ::pcl::PointCloud< ::pcl::PointXYZRGB>);
            normal_cloud->points.resize(normals->size());
            
            // publish inliers to debug
            for (size_t i = 0; i < normals->size(); ++i)
            {
              ::pcl::PointXYZRGB& p = normal_cloud->at(i);
              p.x = input->points[i].x;
              p.y = input->points[i].y;
              p.z = input->points[i].z;
              p.r = (acos(normals->points[i].normal_z) / M_PI + 1.0f) * 255.0;
              p.g = (acos(normals->points[i].normal_y) / M_PI + 1.0f) * 255.0;
              p.b = 0.0f;
            }

            sensor_msgs::PointCloud2Ptr color_msg(new sensor_msgs::PointCloud2);
            ::pcl::toROSMsg(*normal_cloud, *color_msg);
            color_msg->header = pcl_conversions::fromPCL(input->header);
            normal_pub.publish(*color_msg);
        }

        return ecto::OK;
      }

      ecto::spore<double> max_depth_change_factor_, smoothing_size_;
      ecto::spore<int> estimation_method_;
      ecto::spore<ecto::pcl::FeatureCloud> output_;
      ecto::spore<bool> publish_rviz_markers_;
    };

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::NormalEstimationIntegral>, "NormalEstimationIntegral", "Normal estimation using integral images");
