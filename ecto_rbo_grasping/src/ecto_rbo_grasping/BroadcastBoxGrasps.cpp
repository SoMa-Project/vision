/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <limits>

#include <ecto/ecto.hpp>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

struct BroadcastBoxGrasps
{

  typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;

  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_;
  spore<std::vector<UnalignedVector4f> > plane_centroids_;

  ::ros::NodeHandle nh_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > boxes_; // x, y, z, normal_x, normal_y, normal_z, width, height
  spore<std::vector< double> > box_qualities_;
  spore<std::vector<int> > box_ids_;

  spore<std_msgs::Header> header_;

  spore<double> min_size_;
  spore<double> max_size_;

  spore< ::pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;
  spore< ::posesets::PoseSetArrayConstPtr> boxgrasp_manifolds_;

  ::ros::Time last_marker_message_;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<double>("max_size", "", 0.24);
    params.declare<double>("min_size", "", 0.08);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> >("models", "Bounded box coefficients");
    inputs.declare< ::std::vector< double> >("model_qualities", "").required(false);
    inputs.declare< ::std::vector<int> >("model_ids", "Box ids").required(false);
    inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> >("planes", "planes that should be avoided").required(false);
    inputs.declare< ::std::vector<UnalignedVector4f> >("plane_centroids", "planes that should be avoided").required(false);
    inputs.declare<std_msgs::Header>("header", "header of the sensed data that was used to create the grasps");

    outputs.declare< ::pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "");
    outputs.declare< ::posesets::PoseSetArrayConstPtr>("manifolds", "All the grasps that should be used.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    boxes_ = inputs["models"];
    box_ids_ = inputs["model_ids"];
    box_qualities_ = inputs["model_qualities"];
    plane_coefficients_ = inputs["planes"];
    plane_centroids_ = inputs["plane_centroids"];
    header_ = inputs["header"];

    pregrasp_messages_ = outputs["pregrasp_messages"];
    boxgrasp_manifolds_ = outputs["manifolds"];

    min_size_ = params["min_size"];
    max_size_ = params["max_size"];

    last_marker_message_ = ::ros::Time::now();
  }

  bool isColliding(Eigen::Vector3f& centroid, Eigen::Matrix3f& orientation, pregrasp_msgs::GraspStrategy& grasp)
  {
//		float height_2 = height * 0.5;
    return false;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < plane_coefficients_->size(); ++i)
    {
      ::pcl::ModelCoefficientsConstPtr p = plane_coefficients_->at(i);
      Eigen::Vector3f obstacle_normal(p->values[0], p->values[1], p->values[2]);

      // check free space along disk axis
      // distance between cylinder and plane
      double distance = centroid[0] * p->values[0] + centroid[1] * p->values[1] + centroid[2] * p->values[2]
          + p->values[3];

      min_distance = std::min(min_distance, distance);

      if (fabs(distance) < 0.05)
      {
        return true;
      }

      // compute distance to hand along all three dimensions
      double dot_normal = (-obstacle_normal).dot(orientation.col(2));
      if (fabs(dot_normal) > 1e-3)
      {
        double distance_along_normal = distance / dot_normal;

        if (distance_along_normal < 0.1)
          return true;
      }

      double dot_yaws = (-obstacle_normal).dot(orientation.col(1));
      if (fabs(dot_yaws) > 1e-3)
      {
        double distance_along_yaws = distance / dot_yaws;

        if (fabs(distance_along_yaws) < 0.2)
          return true;
      }

      double dot_camera = (-obstacle_normal).dot(orientation.col(0));
      if (fabs(dot_camera) > 1e-3)
      {
        double distance_along_camera = distance / dot_camera;

        if (distance_along_camera > 0 && distance_along_camera < 0.1)
          return true;
      }

      // check that camera fits (prefer orientation which keeps camera away from plane)
      if (orientation.col(0).dot(obstacle_normal) > 0.0)
      {
        // flip
        ROS_INFO("FLIPPING!");
        orientation.col(0) *= -1.0;
        orientation.col(1) *= -1.0;
      }

      // check along approach direction (maybe: make approach comply with plane normal)
    }

    return false;
  }

  void publishRVizPoses(pregrasp_msgs::GraspStrategyArray& grasps)
  {
    static ros::Publisher pose_publisher = nh_.advertise< ::geometry_msgs::PoseStamped>("/grasps", 10);

    ::geometry_msgs::PoseStamped msg;
    msg.header = grasps.header;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps.strategies.begin();
        it != grasps.strategies.end(); ++it)
    {
      msg.pose = it->pregrasp_pose.pose.pose;

      pose_publisher.publish(msg);
    }
  }

  void publishRVizMarkers(pregrasp_msgs::GraspStrategyArray& grasps)
  {
    static ros::Publisher marker_publisher = nh_.advertise< ::visualization_msgs::MarkerArray>("/box_grasps_marker", 10);

    ::visualization_msgs::MarkerArray msgs;
    ::visualization_msgs::Marker msg;
    msg.header = grasps.header;
    msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;
    msg.action = ::visualization_msgs::Marker::ADD;
    msg.lifetime = ::ros::Time::now() - last_marker_message_;

    msg.scale.x = 0.01;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;

    msg.color.g = 1.0;
    msg.color.r = msg.color.b = 0;
    msg.color.a = 0.7;

    msg.mesh_resource = "package://barrett_hand_262/data/barrett_cylindrical.dae";

    static int id = 0;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps.strategies.begin();
        it != grasps.strategies.end(); ++it)
    {
      msg.id = id++;
      msg.pose = it->pregrasp_pose.pose.pose;

      msgs.markers.push_back(msg);
    }

    last_marker_message_ = ::ros::Time::now();
    marker_publisher.publish(msgs);
  }

  int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
  {
    // create ros grasp messages
    pregrasp_msgs::GraspStrategyArrayPtr box_grasps(new pregrasp_msgs::GraspStrategyArray());
    //box_grasps->header = *header_;

    ::posesets::PoseSetArrayPtr boxgrasp_manifolds(new ::posesets::PoseSetArray());

    // iterate over all hypotheses
    for (std::vector< ::pcl::ModelCoefficientsConstPtr>::const_iterator it = boxes_->begin(); it != boxes_->end(); ++it)
//    for (std::vector<int>::const_iterator id = box_ids_->begin(); id != box_ids_->end(); ++id)
    {
//      if (*id < 0)
//        continue;
//
//      ::pcl::ModelCoefficientsConstPtr* it = &(boxes_->at(*id));

      Eigen::Vector3f centroid((*it)->values[0], (*it)->values[1], (*it)->values[2]);
      Eigen::Vector3f centroid_normalized = centroid.normalized();
      Eigen::Vector3f normal((*it)->values[3], (*it)->values[4], (*it)->values[5]);
      normal.normalize();
      Eigen::Vector3f principal_axis((*it)->values[6], (*it)->values[7], (*it)->values[8]);
      float height = principal_axis.norm();
      principal_axis.normalize();
      float width = (*it)->values[9];

      float offset = 0.0; //0.065; // that's just for 3d visualization; better: change origin in model to palm!

      // create an approach vector plus roll angle: cyl_axis X view_y
      pregrasp_msgs::GraspStrategy grasp;
      grasp.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_BOX;
      grasp.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE;
      static int cnt = 0;

      Eigen::Vector3f approach_x = principal_axis;
      Eigen::Vector3f approach_y = normal.cross(principal_axis);
      Eigen::Vector3f approach_z = normal;

      Eigen::Matrix3f rotation;
      rotation << approach_x, approach_y, approach_z;

      Eigen::Vector3f center_offset = centroid - offset * approach_z.normalized();
      grasp.pregrasp_pose.pose.pose.position.x = center_offset[0];
      grasp.pregrasp_pose.pose.pose.position.y = center_offset[1];
      grasp.pregrasp_pose.pose.pose.position.z = center_offset[2];

      grasp.pregrasp_pose.pose.header = *header_;

      if (box_qualities_.user_supplied())
        grasp.quality_grasp = (*box_qualities_)[it - boxes_->begin()];
//      grasp.quality_grasp = (*box_qualities_)[id - box_ids_->begin()];

      if (!isColliding(centroid, rotation, grasp) && width <= *max_size_ && height >= *min_size_)
      {
        grasp.id = cnt++; //box_ids_->at(it - boxes_->begin());

        Eigen::Quaternionf q(rotation);
        grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
        grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
        grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
        grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

        grasp.pregrasp_pose.center = grasp.pregrasp_pose.pose;

        box_grasps->strategies.push_back(grasp);

        // add the corresponding manifold
        tf::Transform poseset_origin;
        tf::poseMsgToTF(grasp.pregrasp_pose.pose.pose, poseset_origin);
        ::posesets::PoseSet ps(poseset_origin);
        ps.setPositions(tf::Vector3(width, height, 0.02));
        ps.getOrientations().addFuzzy(poseset_origin.getRotation());
        boxgrasp_manifolds->push_back(ps);
      }
      else
        ROS_WARN("Box Grasp ignored due to collisions.");

      // add the other box direction
      approach_x = principal_axis.cross(normal);
      approach_y = principal_axis;
      approach_z = normal;
      rotation << approach_x, approach_y, approach_z;

      if (!isColliding(centroid, rotation, grasp) && height <= *max_size_ && width >= *min_size_)
      {
        grasp.id = cnt++; //box_ids_->at(it - boxes_->begin());

        Eigen::Quaternionf q(rotation);
        grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
        grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
        grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
        grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

        grasp.pregrasp_pose.center = grasp.pregrasp_pose.pose;

        box_grasps->strategies.push_back(grasp);

        // add the corresponding manifold
        tf::Transform poseset_origin;
        tf::poseMsgToTF(grasp.pregrasp_pose.pose.pose, poseset_origin);
        ::posesets::PoseSet ps(poseset_origin);
        ps.setPositions(tf::Vector3(height, width, 0.02));
        ps.getOrientations().addFuzzy(poseset_origin.getRotation());
        boxgrasp_manifolds->push_back(ps);
      }
      else
        ROS_WARN("Box Grasp ignored due to collisions.");
    }

    publishRVizPoses(*box_grasps);
    publishRVizMarkers(*box_grasps);

    static ros::Publisher box_grasps_publisher = nh_.advertise<pregrasp_msgs::GraspStrategyArray>("/box_grasps", 10);
    box_grasps_publisher.publish(*box_grasps);

    (*pregrasp_messages_) = box_grasps;
    (*boxgrasp_manifolds_) = boxgrasp_manifolds;

    return OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::BroadcastBoxGrasps, "BroadcastBoxGrasps", "Broadcast some disk grasps.");
