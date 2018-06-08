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
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread/mutex.hpp>
#include <visualization_msgs/MarkerArray.h>
#include "ifco_pose_estimator/ifco_pose.h"

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

  geometry_msgs::PoseArray object_poses;
  std_msgs::Float32MultiArray bounding_boxes;
  tf::Transform cam_to_ifco_convention;
  visualization_msgs::MarkerArray bbox_markers;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster br;



  // inputs
  spore<UnalignedAffine3f> ifco_transform_;

  // outputs
  spore<std::vector<UnalignedAffine3f> > object_poses_;
  spore<std::vector<UnalignedVector3f> > object_sizes_;


  ::ros::Time last_marker_message_;


  // ======================================================================================================================
  static void declare_params(tendrils& params)
  {

  }

  // ======================================================================================================================
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {

    inputs.declare<UnalignedAffine3f>("ifco_transform", "Transform of the IFCO.");


    outputs.declare<std::vector<UnalignedAffine3f> >("transforms", "A vector of 4x4 affine transformations for the objects.");
    outputs.declare<std::vector<UnalignedVector3f> >("sizes", "A vector of 3d sizes for the bounding boxes.");
  }

  // ======================================================================================================================
  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    // inputs
    ifco_transform_ = inputs["ifco_transform"];

    // outputs
    object_poses_ = outputs["transforms"];
    object_sizes_ = outputs["sizes"];

    ros::Time::init();
    last_marker_message_ = ::ros::Time::now();

  }


  void visualize_bboxes(){

      visualization_msgs::MarkerArray markers;

      for(int i=0; i < bounding_boxes.data.size(); i+=3)
      {
          ROS_INFO("Marker Box %i", i);
          visualization_msgs::Marker marker;
          marker.header.frame_id = "camera";
          marker.header.stamp = ros::Time(0);
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
    //(*object_poses_)
    //(*object_sizes_)

    geometry_msgs::Pose ifcoPose, ifcoPose_rotated;
    UnalignedVector3f trans;
    trans = ifco_transform_->translation();
    ifcoPose.position.x = trans(0);
    ifcoPose.position.y = trans(1);
    ifcoPose.position.z = trans(2);

    Eigen::Matrix3f rot;
    rot = ifco_transform_->linear();
    Eigen::Quaternionf q(rot);
    ifcoPose.orientation.x = q.x();
    ifcoPose.orientation.y = q.y();
    ifcoPose.orientation.z = q.z();
    ifcoPose.orientation.w = q.w();

    ros::ServiceClient object_pose_client = nh_.serviceClient<object_segmentation::object_pose>("object_pose");

    object_segmentation::object_pose object_srv;

    tf::Transform cam_to_ifco, correction;
    tf::poseMsgToTF(ifcoPose, cam_to_ifco);

    correction.setRotation( tf::createQuaternionFromRPY(-M_PI,0,0));
    cam_to_ifco_convention = cam_to_ifco * correction;

    geometry_msgs::Transform ifcoPose_rot;

    tf::transformTFToMsg (cam_to_ifco_convention, ifcoPose_rot);

    ifcoPose_rotated.position.x = ifcoPose_rot.translation.x;
    ifcoPose_rotated.position.y = ifcoPose_rot.translation.x;
    ifcoPose_rotated.position.z = ifcoPose_rot.translation.x;

    ifcoPose_rotated.orientation.x = ifcoPose_rot.rotation.x;
    ifcoPose_rotated.orientation.y = ifcoPose_rot.rotation.y;
    ifcoPose_rotated.orientation.z = ifcoPose_rot.rotation.z;
    ifcoPose_rotated.orientation.w = ifcoPose_rot.rotation.w;

    object_srv.request.ifco_pose = ifcoPose_rotated; //srv.response.pose;

    if (object_pose_client.call(object_srv))
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

            ROS_INFO_STREAM("bounding boxes: " << object_srv.response.bounding_boxes[i].x << object_srv.response.bounding_boxes[i].y << object_srv.response.bounding_boxes[i].z);
            ROS_INFO_STREAM("bounding boxes names: " << object_srv.response.object_names[i]);
        }
        bounding_boxes= bounding_boxes_temp;

        visualize_bboxes();

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


    }
    else
    {
        ROS_ERROR("Failed to call object_pose service");
        return QUIT;
    }



    //ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseArray>("object_poses", 100);
    //ros::Publisher bbox_pub = nh_.advertise<std_msgs::Float32MultiArray>("bounding_boxes", 100);
    static ros::Publisher vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("multiObjectBoxes", 1, true);

    //object_poses.header.frame_id = "camera";

    tf::Transform blub2;
    tf::poseMsgToTF(ifcoPose_rotated, blub2);

    br.sendTransform(tf::StampedTransform(blub2, ros::Time::now(), "camera", "ifco_ocadoStyle"));
    //pose_pub.publish(object_poses);
    //bbox_pub.publish(bounding_boxes);
    vis_pub.publish(bbox_markers);

    return ecto::OK;
  }

};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::MultiObjectDetection, "MultiObjectDetection", "Detect Multiple objects.");
