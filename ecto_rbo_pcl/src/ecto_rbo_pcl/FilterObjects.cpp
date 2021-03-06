/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 
#include <ecto_rbo_pcl/common.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>


#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <visualization_msgs/MarkerArray.h>



using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;


// ======================================================================================================================
// ======================================================================================================================
struct FilterObjects
{
  ros::NodeHandle nh_;

  // inputs
  spore<std::vector<UnalignedAffine3f> > object_poses_;
  spore<std::vector<UnalignedVector3f> > object_sizes_;
  spore<std::vector<UnalignedVector4f> > centroids_;

  // parameters
  spore<bool> filter_method_;

  // outputs
  spore<std::vector<UnalignedAffine3f> > object_poses__;
  spore<std::vector<UnalignedVector3f> > object_sizes__;
  spore<std::vector<UnalignedVector4f> > centroids__;


  // ======================================================================================================================
  static void declare_params(tendrils& params)
  {

    params.declare<bool>("filter_method", "0: first object, 1: biggest object,...", 0);

  }

  // ======================================================================================================================
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector<UnalignedAffine3f> >("transforms", "A vector of 4x4 affine transformations for the objects.");
    inputs.declare<std::vector<UnalignedVector3f> >("sizes", "A vector of 3d sizes for the bounding boxes.");
    inputs.declare<std::vector<UnalignedVector4f> >("centroids", "A vector of object centroids");

    outputs.declare<std::vector<UnalignedAffine3f> >("transforms", "A vector of 4x4 affine transformations for the objects.");
    outputs.declare<std::vector<UnalignedVector3f> >("sizes", "A vector of 3d sizes for the bounding boxes.");
    outputs.declare<std::vector<UnalignedVector4f> >("centroids", "A vector of object centroids");
  }

  // ======================================================================================================================
  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    // inputs
    object_poses_ = inputs["transforms"];
    object_sizes_ = inputs["sizes"];
    centroids_ = inputs["centroids"];

    //prams
    filter_method_ = params["filter_method"];

    // outputs
    object_poses__ = outputs["transforms"];
    object_sizes__ = outputs["sizes"];
    centroids__ = outputs["centroids"];
  }





  // ==========================================================================================
  int process(const tendrils& inputs, const tendrils& outputs)
  {

    // here can be implemented any filter method that returns just one of the objects from the input tendrils

    object_poses__->clear();
    object_sizes__->clear();
    centroids__->clear();

    switch (*filter_method_)
    {
      case 0: // keep only 1 object -> first in list
      {
        object_poses__->push_back(object_poses_->front());
        object_sizes__->push_back(object_sizes_->front());
        centroids__->push_back(centroids_->front());
        break;
      }
      case 1: // keep only 1 object -> biggest volume
      {
        // compute object sizes
        std::vector<float> volumes;
        for(std::vector<UnalignedVector3f>::iterator it = object_sizes_->begin(); it != object_sizes_->end(); ++it) 
        {
          float size_x = (*it)(0);
          float size_y = (*it)(1);
          float size_z = (*it)(2);

          float obj_volume = size_x * size_y * size_z;
          volumes.push_back(obj_volume);
        }

        std::vector<float>::iterator result = std::max_element(volumes.begin(), volumes.end());
        
        object_poses__->push_back(object_poses_->at(std::distance(volumes.begin(), result)));
        object_sizes__->push_back(object_sizes_->at(std::distance(volumes.begin(), result)));
        centroids__->push_back(centroids_->at(std::distance(volumes.begin(), result)));
        break;
      }
    }

    


    return ecto::OK;
  }

};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::FilterObjects, "FilterObjects", "Returns the first object in a list of objects");
