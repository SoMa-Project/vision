#include <limits>

#include <ecto/ecto.hpp>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

#include <ecto_rbo_grasping/CylindricalManifold.h>

#ifdef USE_BULLET_COLLISIONS
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btBox2dShape.h>
#include <BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#endif

using namespace ecto;

namespace ecto_rbo_grasping
{

struct BroadcastCylinderGrasps
{

  typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;

  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_;
  spore<std::vector<UnalignedVector4f> > plane_centroids_;

  ::ros::NodeHandle nh_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_cylinders_;
  spore<std::vector< double> > cylinder_qualities_;
  spore<std::vector<int> > cylinder_ids_;

  spore<std_msgs::Header> header_;

  //spore<pregrasp_msgs::GraspStrategyArray> pregrasp_messages_;
  spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;

  ::ros::Time last_marker_message_;

  static void declare_params(ecto::tendrils& params)
  {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> >("models", "Bounded cylinder coefficients");
    inputs.declare< ::std::vector<int> >("model_ids", "Bounded cylinder ids").required(false);
    inputs.declare< ::std::vector< double> >("model_qualities", "");
    inputs.declare< ::std::vector< ::pcl::ModelCoefficientsConstPtr> >("planes", "planes that should be avoided");
    inputs.declare< ::std::vector<UnalignedVector4f> >("plane_centroids", "planes that should be avoided");
    inputs.declare< std_msgs::Header>("header", "header of the sensed data that was used to create the grasps");

    outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    bounded_cylinders_ = inputs["models"];
    cylinder_qualities_ = inputs["model_qualities"];
    cylinder_ids_ = inputs["model_ids"];
    plane_coefficients_ = inputs["planes"];
    plane_centroids_ = inputs["plane_centroids"];
    header_ = inputs["header"];

    pregrasp_messages_ = outputs["pregrasp_messages"];

    last_marker_message_ = ::ros::Time::now();
  }

//  bool isColliding()
//  {
//    // returns wether that cylindrical grasp would collide with any of the obstacles
//    return false;
//  }

#ifdef USE_BULLET_COLLISIONS
  double shortestDistance(const btTransform& grasp_pose, const btConvexShape& obstacle, const btTransform& obstacle_pose)
  {
    // returns the shortest distance between that cylindrical grasp and all obstacles
    // negative means penetration
    btVoronoiSimplexSolver sGjkSimplexSolver;
    btGjkEpaPenetrationDepthSolver epa;

    // create bounding shape for BarrettHand and cylindrical grasp
//    btCylinderShapeZ hand(btVector3(0.01, 0.01, 0.1));
//    btSphereShape hand(0.05);
    int num_spheres = 5;
    btVector3 centers[num_spheres];
    centers[0].setValue(0,     0, 0);
    centers[1].setValue(0, -0.10, 0.025);
    centers[2].setValue(0, -0.05, 0.1);
    centers[3].setValue(0, +0.05, 0.1);
    centers[4].setValue(0, +0.10, 0.025);
    btScalar radi[num_spheres];
    radi[0] = 0.05;
    radi[1] = 0.025;
    radi[2] = 0.025;
    radi[3] = 0.025;
    radi[4] = 0.025;
    btMultiSphereShape hand(&centers[0], &radi[0], num_spheres);

    btGjkPairDetector convexConvex(&obstacle, &hand, &sGjkSimplexSolver, &epa);
    sGjkSimplexSolver.reset();

    btPointCollector gjkOutput;
    btGjkPairDetector::ClosestPointInput input;
    input.m_transformA = obstacle_pose;
    input.m_transformB = grasp_pose;

    convexConvex.getClosestPoints(input, gjkOutput, 0);

    ROS_DEBUG("min distance = %f", gjkOutput.m_distance);

#ifndef SUPPRESS_RVIZ_DEBUG_DRAWINGS
    static ros::Publisher collision_publisher = nh_.advertise<visualization_msgs::Marker>("/collision", 10);

    visualization_msgs::Marker marker;
    marker.header.frame_id =  header_->frame_id;
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration(0.2);
    marker.id = 0;
    marker.ns = "colja";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.color.r = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.05;

    marker.points.resize(2);
    marker.points[0].x = gjkOutput.m_pointInWorld.x();
    marker.points[0].y = gjkOutput.m_pointInWorld.y();
    marker.points[0].z = gjkOutput.m_pointInWorld.z();

    btVector3 endPt = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld * gjkOutput.m_distance;

    marker.points[1].x = endPt.x();
    marker.points[1].y = endPt.y();
    marker.points[1].z = endPt.z();

    collision_publisher.publish(marker);
#endif

    return gjkOutput.m_distance;
  }
#endif

  void considerPlanarStructures(Eigen::Vector3f& center, Eigen::Matrix3f& rotation, double radius, double height)
  {
    if (!plane_coefficients_->empty())
    {
      CylindricalManifold manifold(radius, height, M_PI_2);

      Eigen::Vector3f cyl_axis = rotation.col(0);
      Eigen::Vector3f retract = -(rotation.col(2));
      manifold.setOrigin(center, cyl_axis, retract);

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
                         float height, bool changes, pregrasp_msgs::GraspStrategy& grasp)
  {
    changes = false;
    float height_2 = height * 0.5;
    double min_distance = 1;//std::numeric_limits<double>::max();

    for (size_t i = 0; i < plane_coefficients_->size(); ++i)
    {
      ::pcl::ModelCoefficientsConstPtr p = plane_coefficients_->at(i);

      tf::Vector3 hand_approach(orientation(0,2), orientation(1,2), orientation(2,2));
      tf::Vector3 normal_z(p->values[0], p->values[1], p->values[2]);
      min_distance = std::min(min_distance, fabs(normal_z.normalize().dot(hand_approach)));

/*
      // compute distance
//      min_distance = std::min(
//          min_distance,
//          static_cast<double>(centroid[0] * p->values[0] + centroid[1] * p->values[1] + centroid[2] * p->values[2]
//              + p->values[3]));
      btBox2dShape table(btVector3(5.0, 5.0, 0));
      btVector3 unit_z(0, 0, 1);
      Eigen::Vector3f normal(p->values[0], p->values[1], p->values[2]);
      normal_z.normalize();
      btTransform table_pose(btQuaternion(normal_z.cross(unit_z).normalized(), -btAcos(normal_z.dot(unit_z))), btVector3(plane_centroids_->at(i)[0], plane_centroids_->at(i)[1], plane_centroids_->at(i)[2]));
      Eigen::Quaternionf q(orientation);
      btTransform grasp_pose(btQuaternion(q.x(), q.y(), q.z(), q.w()), btVector3(centroid[0], centroid[1], centroid[2]));
      min_distance = std::min(min_distance, shortestDistance(grasp_pose, table, table_pose));

      int flip = -1;
      // check that camera fits (prefer orientation which keeps camera away from plane)
      if (orientation.col(0).dot(normal) > 0.0)
      {
        // flip
        ROS_INFO("FLIPPING!");
        changes = true;
        orientation.col(0) *= -1.0;
        orientation.col(1) *= -1.0;
        flip = -1;
      }

      // check free space along cylindrical axis
      // distance between cylinder and plane
//			double distance = centroid[0] * p->values[0] + centroid[1] * p->values[1] + centroid[2] * p->values[2] + p->values[3];
//			ROS_INFO("distance = %f", distance);
//				// calculate point on the cylinder axis which obeys distance threshold
//			if (distance > 0.0 && distance < 0.2) {
//				double denominator = cyl_axis[0] * p->values[0] + cyl_axis[1] * p->values[1] + cyl_axis[2] * p->values[2];
//
//				// if plane is parallel to cylinder axis do something
//				if (fabs(denominator) < 1e-3)
//					return false;
//
//				double alpha = -(distance - 0.15) / fabs(denominator);
//
//                                ROS_INFO("alpha = %f", alpha);
//
////				if (std::fabs(alpha) < height_2) {
//
//					ROS_INFO("WOULD CHANGE!");
//					changes = true;
//					centroid = centroid + flip * alpha * cyl_axis;
////				}
//			}
      // check along approach direction (maybe: make approach comply with plane normal)
*/
    }
    grasp.quality_grasp = min_distance;

    ROS_INFO("Minimal Distance: %f", min_distance);

    return true;
  }

#ifndef SUPPRESS_RVIZ_DEBUG_DRAWINGS
  void publishRVizPoses(pregrasp_msgs::GraspStrategyArrayPtr& grasps)
  {
    static ros::Publisher pose_publisher = nh_.advertise< ::geometry_msgs::PoseStamped>("/grasps", 10);

    ::geometry_msgs::PoseStamped msg;
    msg.header = grasps->header;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps->strategies.begin();
        it != grasps->strategies.end(); ++it)
    {
      msg.pose = it->pregrasp_pose.pose.pose;

      pose_publisher.publish(msg);
    }
  }

  void publishRVizMarkers(pregrasp_msgs::GraspStrategyArrayPtr& grasps)
  {
    static ros::Publisher marker_publisher = nh_.advertise< ::visualization_msgs::MarkerArray>("/cylinder_grasps_marker", 10);

    ::visualization_msgs::MarkerArray msgs;
    ::visualization_msgs::Marker msg;
    msg.header = grasps->header;
    msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;
    msg.action = ::visualization_msgs::Marker::ADD;
    msg.lifetime = ::ros::Time::now() - last_marker_message_;

    msg.scale.x = msg.scale.y = msg.scale.z = 1;

    msg.color.r = 1.0;
    msg.color.g = msg.color.b = 0;
    msg.color.a = 0.7;

    msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.obj";

    static int id = 0;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps->strategies.begin();
        it != grasps->strategies.end(); ++it)
    {
      msg.id = id++;
      msg.pose = it->pregrasp_pose.pose.pose;

      // just for debugging
      // msg.pose.position.x = msg.pose.position.y = msg.pose.position.z = 0.0;
      // msg.pose.orientation.x = msg.pose.orientation.y = msg.pose.orientation.z = 0.0;
      // msg.pose.orientation.w = 1.0;

      msgs.markers.push_back(msg);
    }

    last_marker_message_ = ::ros::Time::now();
    marker_publisher.publish(msgs);

  }
#endif

  int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/)
  {
    // create ros grasp messages
    pregrasp_msgs::GraspStrategyArrayPtr cylindrical_grasps(new ::pregrasp_msgs::GraspStrategyArray);

    cylindrical_grasps->header = *header_;

    // iterate over all hypotheses
    for (std::vector< ::pcl::ModelCoefficientsConstPtr>::const_iterator it = bounded_cylinders_->begin();
        it != bounded_cylinders_->end(); ++it)
    {
      Eigen::Vector3f centroid((*it)->values[0], (*it)->values[1], (*it)->values[2]);
      Eigen::Vector3f centroid_normalized = centroid.normalized();
      Eigen::Vector3f direction((*it)->values[3], (*it)->values[4], (*it)->values[5]);
      Eigen::Vector3f direction_normalized = direction.normalized();
      float height = 2.0 * direction.norm();
      float radius = (*it)->values[6];

      // this is a rough filter that represents our 'task': "get the things from the table!"
//      if (!plane_centroids_->empty())
//      {
//        Eigen::Vector3f table_center = plane_centroids_->at(0).topRows(3);
//        if ((table_center - centroid).norm() > 0.35)
//          continue;
//      }

//      float offset = radius; // that's just for 3d visualization; better: change origin in model to palm!

      // create an approach vector plus roll angle: cyl_axis X view_y
      pregrasp_msgs::GraspStrategy grasp;
      grasp.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER;
      grasp.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE;
      if (cylinder_ids_.user_supplied())
        grasp.id = cylinder_ids_->at(it - bounded_cylinders_->begin());

      grasp.pregrasp_pose.size.clear();
      grasp.pregrasp_pose.size.push_back(height);
      grasp.pregrasp_pose.size.push_back(radius);
      grasp.pregrasp_pose.size.push_back(M_PI_2);

      Eigen::Vector3f approach_y = centroid_normalized.cross(direction_normalized);
      Eigen::Vector3f approach_z = direction_normalized.cross(approach_y);
      Eigen::Matrix3f rotation;
      rotation << direction_normalized, approach_y, approach_z;

      considerPlanarStructures(centroid, rotation, radius, height / 2.0);

//      bool collision = considerObstacles(centroid, rotation, direction_normalized, height, changes, grasp);

//        rotation = rotation * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
      Eigen::Quaternionf q(rotation);
      grasp.pregrasp_pose.pose.pose.orientation.x = q.x();
      grasp.pregrasp_pose.pose.pose.orientation.y = q.y();
      grasp.pregrasp_pose.pose.pose.orientation.z = q.z();
      grasp.pregrasp_pose.pose.pose.orientation.w = q.w();

      Eigen::Vector3f center_offset = centroid - radius * rotation.col(2); //approach_z.normalized();
      grasp.pregrasp_pose.pose.pose.position.x = center_offset[0];
      grasp.pregrasp_pose.pose.pose.position.y = center_offset[1];
      grasp.pregrasp_pose.pose.pose.position.z = center_offset[2];

      grasp.pregrasp_pose.pose.header = *header_;

      grasp.quality_grasp = (*cylinder_qualities_)[it - bounded_cylinders_->begin()];

      cylindrical_grasps->strategies.push_back(grasp);
    }

#ifndef SUPPRESS_RVIZ_DEBUG_DRAWINGS
    publishRVizPoses(cylindrical_grasps);
    publishRVizMarkers(cylindrical_grasps);
#endif

    static ros::Publisher cylinder_grasps_publisher = nh_.advertise<pregrasp_msgs::GraspStrategyArray>("/cylinder_grasps", 10);
    cylinder_grasps_publisher.publish(cylindrical_grasps);

    (*pregrasp_messages_) = cylindrical_grasps;

    return OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::BroadcastCylinderGrasps, "BroadcastCylinderGrasps", "Broadcast some cylindrical grasps.");
