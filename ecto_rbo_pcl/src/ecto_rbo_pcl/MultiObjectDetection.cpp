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
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;


// ======================================================================================================================
// ======================================================================================================================
struct MultiObjectDetection
{
  ros::NodeHandle nh_;

  geometry_msgs::PoseArray object_poses;
  std_msgs::Float32MultiArray bounding_boxes;
  tf::Transform ifcoPose_corrected_tf;
  visualization_msgs::MarkerArray bbox_markers;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster br;



  // inputs
  spore<UnalignedAffine3f> ifco_transform_;
  spore< std::vector<float> > ec_wall_offset_;
  spore< std::vector<int> > ec_installed_on_wall_;
  spore<double> ifco_length_;
  spore<double> ifco_width_;

  // parameters
  spore<bool> ifco_alignment_;

  // outputs
  spore<std::vector<UnalignedAffine3f> > object_poses_;
  spore<std::vector<UnalignedVector3f> > object_sizes_;
  spore<std::vector<UnalignedVector4f> > centroids_;



  // ======================================================================================================================
  static void declare_params(tendrils& params)
  {

    params.declare<bool>("ifco_alignment", "Use the z-Axis of the ifco transform also as object normal.", false);

  }

  // ======================================================================================================================
  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<UnalignedAffine3f>("ifco_transform", "Transform of the IFCO.");
    inputs.declare< std::vector<float> >("ec_wall_offset", "The space that is occupied by the ec.");
    inputs.declare< std::vector<int> >("ec_installed_on_wall", "The wall on which the ec is isntalled inside the ifco. 0 if no ec is installed. Walls defined counter-clockwise.");
    inputs.declare<double>("ifco_length", "Size of the long IFCO edge", 0.0);
    inputs.declare<double>("ifco_width", "Size of the short IFCO edge", 0.0);

    outputs.declare<std::vector<UnalignedAffine3f> >("transforms", "A vector of 4x4 affine transformations for the objects.");
    outputs.declare<std::vector<UnalignedVector3f> >("sizes", "A vector of 3d sizes for the bounding boxes.");
    outputs.declare<std::vector<UnalignedVector4f> >("centroids", "Object centers");
  }

  // ======================================================================================================================
  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    // inputs
    ifco_transform_         = inputs["ifco_transform"];
    ec_wall_offset_         = inputs["ec_wall_offset"];
    ec_installed_on_wall_   = inputs["ec_installed_on_wall"];
    ifco_length_ = inputs["ifco_length"];
    ifco_width_ = inputs["ifco_width"];


    //params
    ifco_alignment_ = params["ifco_alignment"];

    // outputs
    object_poses_ = outputs["transforms"];
    object_sizes_ = outputs["sizes"];
    centroids_    = outputs["centroids"];
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



  void translateIFCO_ec(Eigen::Translation3f& ifco_pose)
  {
      float ifco_length_new = (*ifco_length_);
      float ifco_width_new = (*ifco_width_);

      for (std::vector< int >::iterator it = ec_installed_on_wall_->begin(); it != ec_installed_on_wall_->end(); ++it) 
      {
              int ec = std::distance(ec_installed_on_wall_->begin(), it);
              float offset = ec_wall_offset_->at(ec); // needs to be removed from ifco free space
              // in which dimension?
              switch (*it) 
              {
                  case 0:
                       break; // not installed
                  case 1:
                  {
                      ifco_pose.x() -= offset;
                      ifco_length_new -= offset;
                      break;
                  }
                  case 2:
                  {
                      ifco_pose.y() -= offset;
                      ifco_width_new -= offset;
                      break;
                  }
                  case 3:
                  {
                      ifco_pose.x() += offset;
                      ifco_length_new -= offset;
                      break;
                  }
                  case 4:
                  {
                      ifco_pose.y() += offset;
                      ifco_width_new -= offset;
                      break;
                  }

              }

              nh_.setParam("/ifco/length", ifco_length_new);
              nh_.setParam("/ifco/width", ifco_width_new);
      }
      return;
    }


  // ==========================================================================================
  int process(const tendrils& inputs, const tendrils& outputs)
  {
    // the multi-object service needs the ifco transform but rotated by 90 degrees as provided originally by ocado ICP.

    object_poses_->clear();
    object_sizes_->clear();
    centroids_->clear();

    // convert input tendril ifco transform to geometry_msg
    geometry_msgs::Pose ifcoPose_msg;

    UnalignedVector3f trans;
    trans = ifco_transform_->translation();
    ifcoPose_msg.position.x = trans(0);
    ifcoPose_msg.position.y = trans(1);
    ifcoPose_msg.position.z = trans(2);

    Eigen::Matrix3f rot;
    rot = ifco_transform_->linear();
    Eigen::Quaternionf q(rot);
    ifcoPose_msg.orientation.x = q.x();
    ifcoPose_msg.orientation.y = q.y();
    ifcoPose_msg.orientation.z = q.z();
    ifcoPose_msg.orientation.w = q.w();


    // convert ifco transform geometry_msg to tf and correct by -90 degree rotation around x.
    tf::Transform ifcoPose_tf, correction;
    tf::poseMsgToTF(ifcoPose_msg, ifcoPose_tf);
    correction.setRotation( tf::createQuaternionFromRPY(-M_PI,0,0));
    ifcoPose_corrected_tf = ifcoPose_tf * correction;

    geometry_msgs::Transform ifcoPose_corrected_msg;
    tf::transformTFToMsg (ifcoPose_corrected_tf, ifcoPose_corrected_msg);

    // the final ifco frame that gets handed over to the multi-object service
    geometry_msgs::Pose ifcoPose_final;
    ifcoPose_final.position.x = ifcoPose_msg.position.x;
    ifcoPose_final.position.y = ifcoPose_msg.position.y;
    ifcoPose_final.position.z = ifcoPose_msg.position.z;

    ifcoPose_final.orientation.x = ifcoPose_corrected_msg.rotation.x;
    ifcoPose_final.orientation.y = ifcoPose_corrected_msg.rotation.y;
    ifcoPose_final.orientation.z = ifcoPose_corrected_msg.rotation.z;
    ifcoPose_final.orientation.w = ifcoPose_corrected_msg.rotation.w;

    ros::ServiceClient object_pose_client = nh_.serviceClient<object_segmentation::object_pose>("object_pose");
    object_segmentation::object_pose object_srv;

    object_srv.request.ifco_pose = ifcoPose_final;

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
      }
      bounding_boxes = bounding_boxes_temp;

      visualize_bboxes();

      // generate output tendrils
      for(int i=0; i < bounding_boxes.data.size(); i+=3)
      {
        // object poses
        geometry_msgs::Pose pose = object_poses.poses[i/3];

        Eigen::Matrix3f rotation;
        Eigen::Quaterniond objRotation_eigen;
        tf::quaternionMsgToEigen (pose.orientation, objRotation_eigen);

        Eigen::Matrix3d objRotation_ = objRotation_eigen.toRotationMatrix();
        rotation = objRotation_.cast<float>();

        if (*ifco_alignment_)
        {
          // ensure, that object normal is aligned with ifco normal (for ocado use case)
          // we extract the normal of the ifco rotation (rot) and set it as the new object normal
          Eigen::Matrix3f rotation_obj_ifco_normal;
          tf::Vector3 obj_z = tf::Vector3(rot(0,2), rot(1,2), rot(2,2));
          // project object x axis on ifco plane and use as new object x asis
          tf::Vector3 obj_x = (tf::Vector3(rotation(0,0), rotation(1,0), rotation(2,0)) - ( tf::Vector3(rotation(0,0), rotation(1,0), rotation(2,0)).dot(obj_z)) * obj_z).normalized();

          // Compute the orientation
          tf::Vector3 obj_y = obj_z.cross(obj_x);
          rotation_obj_ifco_normal << obj_x.x(), obj_y.x(), obj_z.x(),
                                      obj_x.y(), obj_y.y(), obj_z.y(),
                                      obj_x.z(), obj_y.z(), obj_z.z();


          rotation = rotation_obj_ifco_normal;
        }

        UnalignedAffine3f transform = Eigen::Translation3f(pose.position.x,
                                                           pose.position.y,
                                                           pose.position.z) * rotation;

        object_poses_->push_back(transform);

        // add bounding box dimensions
        UnalignedVector3f size (bounding_boxes.data[i], bounding_boxes.data[i+1], bounding_boxes.data[i+2]);
        object_sizes_->push_back(size);

        UnalignedVector4f center(pose.position.x, pose.position.y, pose.position.z, 0);
        centroids_->push_back(center);
      }
    }
    else
    {
      ROS_ERROR("Failed to call object_pose service");
      return QUIT;
    }

    // visualize object detection result as boxes and poses
    static ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseArray>("object_poses", 1, true);
    static ros::Publisher vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("multiObjectBoxes", 1, true);
    object_poses.header.frame_id = "camera";
    pose_pub.publish(object_poses);
    vis_pub.publish(bbox_markers);

    // un-comment to see which pose is used for the multi-object detection:
    //br.sendTransform(tf::StampedTransform(ifcoPose_corrected_tf, ros::Time::now(), "camera", "ifco_multiObject"));

    return ecto::OK;
  }

};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::MultiObjectDetection, "MultiObjectDetection", "Detect Multiple objects.");
