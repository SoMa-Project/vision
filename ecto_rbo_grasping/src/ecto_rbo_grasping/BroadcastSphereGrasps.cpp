#include <ecto/ecto.hpp>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

#include "ecto_rbo_grasping/SphericalManifold.h"


using namespace ecto;

namespace ecto_rbo_grasping
{

struct BroadcastSphereGrasps
{

  typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;

  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_;
  spore<std::vector<UnalignedVector4f> > plane_centroids_;

  ::ros::NodeHandle nh_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > spheres_; // x, y, z, normal_x, normal_y, normal_z, radius
  spore<std::vector< double> > sphere_qualities_;
  spore<std::vector<int> > disk_ids_;

  spore<std_msgs::Header> header_;

  spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;

  ::ros::Time last_marker_message_;

  static void declare_params(ecto::tendrils& params)
  {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> >("models", "Sphere coefficients: x, y, z, normal_x, normal_y, normal_z, radius");
    inputs.declare< ::std::vector< double> >("model_qualities", "");
    inputs.declare< ::std::vector<int> >("model_ids", "Sphere ids").required(false);
    inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> >("planes", "planes that should be avoided").required(false);
    inputs.declare< ::std::vector<UnalignedVector4f> >("plane_centroids", "planes that should be avoided").required(false);
    inputs.declare< std_msgs::Header>("header", "header of the sensed data that was used to create the grasps");

    outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    spheres_ = inputs["models"];
    sphere_qualities_ = inputs["model_qualities"];
    disk_ids_ = inputs["model_ids"];
    plane_coefficients_ = inputs["planes"];
    plane_centroids_ = inputs["plane_centroids"];
    header_ = inputs["header"];

    pregrasp_messages_ = outputs["pregrasp_messages"];

    last_marker_message_ = ::ros::Time::now();
  }

  void considerPlanarStructures(Eigen::Vector3f& center, Eigen::Matrix3f& rotation, double radius)
  {
    if (!plane_coefficients_->empty())
    {
      SphericalManifold manifold(radius, M_PI_2);

      Eigen::Vector3f middle_finger = -rotation.col(1);
      Eigen::Vector3f retract = -(rotation.col(2));
      manifold.setOrigin(center, middle_finger, retract);

//      tf::Vector3 btcenter(center(0), center(1), center(2));
//      tf::Vector3 btx_axis(-rotation(0,2), -rotation(1,2), -rotation(2,2));
//      tf::Vector3 btcylinder_axis(rotation(0,0), rotation(1,0), rotation(2,0));
//      manifold.setOrigin(btcenter, btcylinder_axis, btx_axis);

      ::pcl::ModelCoefficientsConstPtr p = plane_coefficients_->at(0);
      Eigen::Vector3f normal_z(p->values[0], p->values[1], p->values[2]);

      Eigen::Vector3f optimal_centroid, resulting_normal;
      manifold.getCartesianCoordinate(normal_z, resulting_normal, optimal_centroid);

      center = optimal_centroid;
      rotation << rotation.col(0), resulting_normal.cross(rotation.col(0)), resulting_normal;
    }
  }

  bool considerObstacles(Eigen::Vector3f& centroid, Eigen::Matrix3f& orientation, Eigen::Vector3f cyl_axis,
                         float height, bool changes)
  {
    changes = false;
    float height_2 = height * 0.5;

    for (size_t i = 0; i < plane_coefficients_->size(); ++i)
    {
      ::pcl::ModelCoefficientsConstPtr p = plane_coefficients_->at(i);
      Eigen::Vector3f normal(p->values[0], p->values[1], p->values[2]);

      // check free space along disk axis
      // distance between cylinder and plane
      double distance = centroid[0] * p->values[0] + centroid[1] * p->values[1] + centroid[2] * p->values[2]
          + p->values[3];
      if (distance > 0.0 && distance < 0.1)
      {
        // calculate point on the cylinder axis which obeys distance threshold
        double denominator = cyl_axis[0] * p->values[0] + cyl_axis[1] * p->values[1] + cyl_axis[2] * p->values[2];

        // if plane is parallel to cylinder axis do something
        if (denominator < 1e-3)
          return false;

        double alpha = -(distance - 0.1) / denominator;

        if (std::fabs(alpha) < height_2)
        {

          ROS_INFO("WOULD CHANGE!");
          changes = true;
//					centroid = centroid + alpha * cyl_axis;
        }
      }

      // check that camera fits (prefer orientation which keeps camera away from plane)
      if (orientation.col(0).dot(normal) > 0.0)
      {
        // flip
        ROS_INFO("FLIPPING!");
        changes = true;
        orientation.col(0) *= -1.0;
        orientation.col(1) *= -1.0;
      }

      // check along approach direction (maybe: make approach comply with plane normal)
    }

    return true;
  }

  void publishRVizPoses(pregrasp_msgs::GraspStrategyArrayPtr& grasps)
  {
    static ros::Publisher pose_publisher = nh_.advertise< ::geometry_msgs::PoseStamped>("/sphere_grasps", 10);

    ::geometry_msgs::PoseStamped msg;
    msg.header = grasps->header;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps->strategies.begin(); it != grasps->strategies.end(); ++it)
    {
      msg.pose = it->pregrasp_pose.pose.pose;

      pose_publisher.publish(msg);
    }
  }

  void publishRVizMarkers(pregrasp_msgs::GraspStrategyArrayPtr& grasps)
  {
    static ros::Publisher marker_publisher = nh_.advertise< ::visualization_msgs::MarkerArray>("/sphere_grasps_marker", 10);

    ::visualization_msgs::MarkerArray msgs;
    ::visualization_msgs::Marker msg;
    msg.header = grasps->header;
    msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;
    msg.action = ::visualization_msgs::Marker::ADD;
    msg.lifetime = ::ros::Time::now() - last_marker_message_;

    msg.scale.x = msg.scale.y = msg.scale.z = 1;

    // pink
    msg.color.b = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 0;
    msg.color.a = 0.7;

    msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.obj";

    static int id = 0;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps->strategies.begin(); it != grasps->strategies.end(); ++it)
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
    pregrasp_msgs::GraspStrategyArrayPtr sphere_grasps(new ::pregrasp_msgs::GraspStrategyArray);
    sphere_grasps->header = *header_;

    // iterate over all hypotheses
    for (std::vector< ::pcl::ModelCoefficientsConstPtr>::const_iterator it = spheres_->begin(); it != spheres_->end(); ++it)
    {
      Eigen::Vector3f centroid((*it)->values[0], (*it)->values[1], (*it)->values[2]);
      Eigen::Vector3f centroid_normalized = centroid.normalized();
      Eigen::Vector3f direction((*it)->values[3], (*it)->values[4], (*it)->values[5]);
      Eigen::Vector3f direction_normalized = direction.normalized();
      float radius = (*it)->values[3];

      float offset = radius;//0.065; // that's just for 3d visualization; better: change origin in model to palm!

      // create an approach vector plus roll angle: cyl_axis X view_y
      pregrasp_msgs::GraspStrategy grasp;
      grasp.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE;
      grasp.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE;
      static int cnt = 0;
      grasp.id = cnt++; //sphere_ids_->at(it - spheres_->begin());

      Eigen::Quaternionf q;
      q.setFromTwoVectors(Eigen::Vector3f::UnitZ(), centroid_normalized);

      Eigen::Matrix3f rotation = q.toRotationMatrix();
      considerPlanarStructures(centroid, rotation, offset);

      q = Eigen::Quaternionf(rotation);

      bool changes = false;
//      bool collision = considerObstacles(centroid, rotation, direction_normalized, height, changes);
      bool collision = false;

      if (changes)
      {
        ROS_INFO("Sphere Grasps were changed!");
      }

      if (collision)
      {
        ROS_WARN("Sphere Grasp ignored due to collisions.");
      }
      else
      {
        grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
        grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
        grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
        grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

        Eigen::Vector3f center_offset = centroid - offset * centroid_normalized;
        grasp.pregrasp_pose.pose.pose.position.x = center_offset[0];
        grasp.pregrasp_pose.pose.pose.position.y = center_offset[1];
        grasp.pregrasp_pose.pose.pose.position.z = center_offset[2];

        grasp.pregrasp_pose.pose.header = *header_;

        grasp.quality_grasp = (*sphere_qualities_)[it - spheres_->begin()];

        sphere_grasps->strategies.push_back(grasp);
      }
    }

//    publishRVizPoses(sphere_grasps);
    publishRVizMarkers(sphere_grasps);

    static ros::Publisher sphere_grasps_publisher = nh_.advertise<pregrasp_msgs::GraspStrategyArray>("/sphere_grasps", 10);
    sphere_grasps_publisher.publish(sphere_grasps);

    (*pregrasp_messages_) = sphere_grasps;

    return OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::BroadcastSphereGrasps, "BroadcastSphereGrasps", "Broadcast some spherical grasps.");
