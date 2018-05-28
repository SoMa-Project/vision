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

#include "object_segmentation/object_pose.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
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


// ======================================================================================================================
// ======================================================================================================================
struct MultiObjectDetection
{
  ros::NodeHandle nh_;

  // TODO: check if those are required or can be deleted
  geometry_msgs::PoseArray object_poses;
  std_msgs::Float32MultiArray bounding_boxes;
  tf::Transform cam_to_ifco_convention;
  visualization_msgs::MarkerArray bbox_markers;

  // needed for object detection
  tf::TransformBroadcaster br;
  ros::ServiceClient client = nh_.serviceClient<object_segmentation::object_pose>("object_pose");
  object_segmentation::object_pose object_srv;


  // outputs
  ecto::spore<std::vector<UnalignedAffine3f> > object_poses_;
  ecto::spore<std::vector<UnalignedVector3f> > object_sizes_;


  // ======================================================================================================================
  static void declare_params(ecto::tendrils& params)
  {

  }

  // ======================================================================================================================
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    outputs.declare<std::vector<UnalignedAffine3f> >(&MultiObjectDetection::object_poses_, "transforms", "A vector of 4x4 affine transformations for the objects.");
    outputs.declare<std::vector<UnalignedVector3f> >(&MultiObjectDetection::object_sizes_, "sizes", "A vector of 3d sizes for the bounding boxes.");
  }

  // ======================================================================================================================
  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {

    // outputs
    object_poses_ = outputs["transforms"];
    object_sizes_ = outputs["sizes"];

  }


  void visualize_bboxes(){

      visualization_msgs::MarkerArray markers;

      for(int i=0; i < bounding_boxes.data.size(); i+=3)
      {
          visualization_msgs::Marker marker;
          marker.header.frame_id = "camera";
          marker.header.stamp = ros::Time();
          marker.id = i;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose = object_poses.poses[i/3];
          marker.scale.x = bounding_boxes.data[i];
          marker.scale.y = bounding_boxes.data[i+1];
          marker.scale.z = bounding_boxes.data[i+2];
          marker.color.a = 0.5; // Don't forget to set the alpha!
          marker.color.r = 0.4;
          marker.color.g = 0.0;
          marker.color.b = 0.2;

          markers.markers.push_back(marker);
      }
      bbox_markers = markers;
  }



  // ==========================================================================================
  int process(const tendrils& inputs, const tendrils& outputs)
  {

    if (client.call(object_srv))
    {
        ROS_INFO("Object service was called");
        object_poses.poses = object_srv.response.object_poses;

        std_msgs::Float32MultiArray bounding_boxes_temp;
        ROS_INFO("Detected %lu objects", object_srv.response.bounding_boxes.size());

        for(int i=0; i < object_srv.response.bounding_boxes.size(); i++)
        {
            bounding_boxes_temp.data.push_back(object_srv.response.bounding_boxes[i].x);
            bounding_boxes_temp.data.push_back(object_srv.response.bounding_boxes[i].y);
            bounding_boxes_temp.data.push_back(object_srv.response.bounding_boxes[i].z);

        }
        bounding_boxes = bounding_boxes_temp;


        for(int i=0; i < bounding_boxes.data.size(); i+=3)
        {
          // add object posed
            geometry_msgs::Pose pose = object_poses.poses[i/3];

            Eigen::Matrix3f rotation;
            Eigen::Quaterniond objRotation_eigen;
            tf::quaternionMsgToEigen (pose.orientation, objRotation_eigen);

            Eigen::Matrix3d objRotation_ = objRotation_eigen.toRotationMatrix();
            rotation = objRotation_.cast<float>();

            UnalignedAffine3f transform = Eigen::Translation3f(pose.position.x,
                                                               pose.position.y,
                                                               pose.position.z) * rotation;
            object_poses_->push_back(transform);


            // add bounding box dimensions
            UnalignedVector3f size (bounding_boxes.data[i], bounding_boxes.data[i+1], bounding_boxes.data[i+2]);
            object_sizes_->push_back(size);
        }

        visualize_bboxes();

    }
    else
    {
        ROS_ERROR("Failed to call object_pose service");
        return false;
    }



  }

};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::MultiObjectDetection, "MultiObjectDetection", "Detect Multiple objects.");
