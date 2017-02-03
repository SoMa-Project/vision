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
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

using namespace ecto;

namespace ecto_rbo_pcl
{

struct Centroid
{
  typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
  typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;

  spore<std::vector< ::pcl::PointIndices> > clusters_;
  spore<std::vector<UnalignedVector4f> > centroids_;

  static void declare_params(tendrils& params)
  {
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector< ::pcl::PointIndices> >("clusters", "Clusters indices.");
    outputs.declare<std::vector<UnalignedVector4f> >("centroids", "Clusters centroids");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    clusters_ = inputs["clusters"];
    centroids_ = outputs["centroids"];
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
      centroids_->clear();

      // iterate over all clusters
      for (std::vector< ::pcl::PointIndices>::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
      {
        // calculate mean point
        Eigen::Vector4f mean;
        ::pcl::compute3DCentroid(*input, *it, mean);
//    		  std::cout << "mean " << mean[0] << " " << mean[1] << " " << mean[2] << " " << mean[3]<< std::endl;

        centroids_->push_back(mean);
      }

      return ecto::OK;
    }
};
}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::Centroid>, "Centroid", "Calculate 3D centroids for point clusters.");
