#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/eigen.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <pregrasp_msgs/PreGrasp.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

struct BroadcastGrasps
{
  typedef Eigen::Matrix<float,3,1,Eigen::DontAlign> UnalignedVector3f;
  typedef Eigen::Matrix<float,4,1,Eigen::DontAlign> UnalignedVector4f;
  typedef Eigen::Matrix<float,3,3,Eigen::DontAlign> UnalignedMatrix3f;

  static void declare_params(tendrils& params) {
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
    inputs.declare< UnalignedMatrix3f> (&BroadcastGrasps::eigen_vectors_, "eigenvectors", "The eigenvectors of the input point cloud.");
    inputs.declare< UnalignedVector3f> (&BroadcastGrasps::eigen_values_, "eigenvalues", "The eigenvalues of the input point cloud.");
    inputs.declare< UnalignedVector4f> (&BroadcastGrasps::xyz_centroid_, "mean", "The centroid of the input point cloud.");
    inputs.declare< std_msgs::Header>  (&BroadcastGrasps::header_, "header", "header of the sensed data that was used to create the grasps");

    inputs.declare< ::pcl::ModelCoefficientsConstPtr> (&BroadcastGrasps::bounded_cylinder_coefficients_, "bounded_cylinder_model", "A cylinder.");
    inputs.declare< ::pcl::ModelCoefficientsConstPtr> (&BroadcastGrasps::obstacle_plane_coefficients_, "obstacle_plane_model", "A plane that should not be grasped but taken into account for grasps.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
  }

  void transformEigenToMsg(const Eigen::Affine3f& transform, geometry_msgs::Pose& msg) {
    Eigen::Vector3f translation = transform.translation();
    msg.position.x = translation.x();
    msg.position.y = translation.y();
    msg.position.z = translation.z();

    Eigen::Quaternionf rotation(transform.rotation());
    msg.orientation.x = rotation.x();
    msg.orientation.y = rotation.y();
    msg.orientation.z = rotation.z();
    msg.orientation.w = rotation.w();

//    ROS_DEBUG("geometry_msgs::Pose %f %f %f", msg.position.x, msg.position.y, msg.position.z);

//    if (fabs(bt.length2() - 1 ) > QUATERNION_TOLERANCE)
//        {
//          ROS_WARN("TF to MSG: Quaternion Not Properly Normalized");
//          Quaternion bt_temp = bt;
//          bt_temp.normalize();
//          msg.x = bt_temp.x(); msg.y = bt_temp.y(); msg.z = bt_temp.z();  msg.w = bt_temp.w();
//        }
//      else
//      {
//        msg.x = bt.x(); msg.y = bt.y(); msg.z = bt.z();  msg.w = bt.w();
//      }
  }

  void broadcastRVizMarkers(ros::NodeHandle& nh, pregrasp_msgs::PreGrasp& pregrasps) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = header_->frame_id;
    marker.header.stamp = ros::Time();
//    marker.ns = "my_namespace";
//    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = 1;
//    marker.pose.position.y = 1;
//    marker.pose.position.z = 1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    double max_quality = pregrasps.quality[0];
    double min_quality = pregrasps.quality[0];
    // get maximum quality value
    for (size_t i = 0; i < pregrasps.quality.size(); ++i) {
      if (max_quality > pregrasps.quality[i]) {
        max_quality = pregrasps.quality[i];
      }
      else if (min_quality < pregrasps.quality[i]) {
        min_quality = pregrasps.quality[i];
      }
    }

    for (size_t i = 0; i < pregrasps.poses.size(); ++i) {
      tf::Vector3 position;
      tf::Quaternion orientation;
      tf::pointMsgToTF(pregrasps.poses[i].position, position);
      tf::quaternionMsgToTF(pregrasps.poses[i].orientation, orientation);

      tf::Vector3 end = tf::quatRotate(orientation, tf::Vector3(0, 0, 0.05)) + position;

      geometry_msgs::Point end_point;
      tf::pointTFToMsg(end, end_point);

      marker.points.push_back(pregrasps.poses[i].position);
      marker.points.push_back(end_point);

      end = tf::quatRotate(orientation, tf::Vector3(0, 0.01, 0)) + position;
      tf::pointTFToMsg(end, end_point);
      marker.points.push_back(pregrasps.poses[i].position);
      marker.points.push_back(end_point);

      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 1.0 - (pregrasps.quality[i] - min_quality) / (max_quality - min_quality);
      color.g = 0.0;
      color.b = 0.0;
      marker.colors.push_back(color);
      marker.colors.push_back(color);
      marker.colors.push_back(color);
      marker.colors.push_back(color);
    }

    static ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_grasps", 0 );
    vis_pub.publish(marker);
  }

  void broadcastExplorationDirection(Eigen::Vector3f& cylinder_centroid, Eigen::Vector3f& cylinder_direction, Eigen::Affine3f& cylinder_transform, float radius) {
    Eigen::Vector3f up_dir = Eigen::AngleAxisf(M_PI_2, cylinder_direction) * (-cylinder_centroid);
    Eigen::Vector3f down_dir = Eigen::AngleAxisf(-M_PI_2, cylinder_direction) * (-cylinder_centroid);
//    Eigen::Vector3f up_dir = cylinder_transform.inverse() * Eigen::Vector3f::UnitY();
//    Eigen::Vector3f down_dir = cylinder_transform.inverse() * (-Eigen::Vector3f::UnitY());

    ::pcl::PointXYZ p(up_dir[0], up_dir[1], up_dir[2]);
    double up_dist = ::pcl::pointToPlaneDistanceSigned(p, (*obstacle_plane_coefficients_)->values[0],
                                                       (*obstacle_plane_coefficients_)->values[1],
                                                       (*obstacle_plane_coefficients_)->values[2],
                                                       (*obstacle_plane_coefficients_)->values[3]);

    ::pcl::PointXYZ p2(down_dir[0], down_dir[1], down_dir[2]);
    double down_dist = ::pcl::pointToPlaneDistanceSigned(p2, (*obstacle_plane_coefficients_)->values[0],
                                                       (*obstacle_plane_coefficients_)->values[1],
                                                       (*obstacle_plane_coefficients_)->values[2],
                                                       (*obstacle_plane_coefficients_)->values[3]);

    ROS_DEBUG("distance up = %f ___ down = %f", up_dist, down_dist);

    if (up_dist > down_dist) {

    }
    else {

    }

    /*static ros::Publisher exploration_pub = nh.advertise < geometry_msgs::PoseStamped > ("exploration_direction", 10);
    geometry_msgs::PoseStamped msg_stamped;
    msg_stamped.header.stamp = ros::Time::now();
    msg_stamped.header.frame_id = "/camera_rgb_optical_frame"; // depth!
    transformEigenToMsg(t, msg_stamped.pose);
    exploration_pub.publish(msg_stamped);*/
  }

  void broadcastBestGrasp(ros::NodeHandle& nh, pregrasp_msgs::PreGrasp& pregrasps, std::vector<std::vector<int> >& cylindrical_coordinates, Eigen::Vector3f& cylinder_dir) {
    if (pregrasps.poses.empty()) {
      // at the moment this never happens but maybe in the future...
      // we would like to use other hints to choose the search direction
      return;
    }

    int best = 0;
    for (size_t i = 0; i < pregrasps.poses.size(); ++i) {
      if (pregrasps.quality[i] > pregrasps.quality[best]) {
        best = i;
      }
    }

    static tf::TransformBroadcaster br;
    tf::Transform T;
    tf::poseMsgToTF(pregrasps.poses[best], T);
    br.sendTransform(tf::StampedTransform(T, ros::Time::now(), "/camera_rgb_optical_frame", "/grasp_1"));

//    ROS_DEBUG("maximum border grasp: %i with quality = %f", max_expected_border_grasp_index, pregrasps.quality[max_expected_border_grasp_index]);

/*    static ros::Publisher exploration_pub = nh.advertise < geometry_msgs::PoseStamped > ("exploration_direction", 10);
    geometry_msgs::PoseStamped msg_stamped;
    msg_stamped.header.stamp = ros::Time::now();
    msg_stamped.header.frame_id = "/camera_rgb_optical_frame"; // depth!

    int width = cylindrical_coordinates.size();
    int height = cylindrical_coordinates[width/2].size();
    Eigen::Vector3f a(max_i - width/2, max_j - height/2, 0);

    ROS_DEBUG("ha %i %i %i %i", width, height, max_i, max_j);
    ROS_DEBUG("exploration direction: %f %f %f", a[0], a[1], a[2]);

    a.normalize();
    Eigen::Affine3f t;
    ::pcl::getTransformationFromTwoUnitVectors(a, cylinder_dir, t);
    t = t.inverse();
    transformEigenToMsg(t, msg_stamped.pose);

    exploration_pub.publish(msg_stamped);*/
  }

  Eigen::Affine3f getGraspTransform(const Eigen::Affine3f& t, float radius, float height, float y) {
    return t * Eigen::Translation3f(y, 0, -radius);
  }

  void addCylindricalGrasp(pregrasp_msgs::PreGrasp& pregrasps, const Eigen::Affine3f& t, float y, float angle, float height) {
    geometry_msgs::Pose msg;
    transformEigenToMsg(t, msg);
    pregrasps.poses.push_back(msg);

    // grasp quality depends on
    // shape (object constraints): goodness-of-fit (see *FitStatitistics), dimensions
    // environmental constraints (likelihood of collision, which depends on chosen strategy): e.g. angle between approach vector and support plane normal and distance
//    double environmental_quality =
    // control constraints: this is an open one and should not be considered now, because only the vision strategy is available (which strategy?); otherwise is another strategy always better?
    // this affects both above

    ::pcl::PointXYZ p(t.translation()[0], t.translation()[1], t.translation()[2]);
    double dist = ::pcl::pointToPlaneDistance(p, (*obstacle_plane_coefficients_)->values[0],
                                              (*obstacle_plane_coefficients_)->values[1],
                                              (*obstacle_plane_coefficients_)->values[2],
                                              (*obstacle_plane_coefficients_)->values[3]);
    double shape_match = std::max(1.0 - (fabs(y) / (0.5 * height)) - fabs(angle) * 0.5, 0.0);
//    ROS_DEBUG("shape match: %f", shape_match);

    double total_quality = shape_match * dist;
//    ROS_DEBUG("total quality: %f", total_quality);

    pregrasps.quality.push_back(total_quality);
  }

  void getTransformationFromTwoUnitVectorsXYAndOrigin(const Eigen::Vector3f& x_direction, const Eigen::Vector3f& y_direction,
                                                      const Eigen::Vector3f& origin, Eigen::Affine3f& transformation) {
    Eigen::Vector3f ha = x_direction.cross(y_direction);
    ::pcl::getTransformationFromTwoUnitVectorsAndOrigin(ha, y_direction, origin, transformation);

//    ::pcl::getTransFromUnitVectorsXY(x_direction, y_direction, transformation);
//    Eigen::Vector3f translation = transformation * origin;
//    transformation(0, 3) = -translation[0];
//    transformation(1, 3) = -translation[1];
//    transformation(2, 3) = -translation[2];
  }

  void broadcastCylindricalPositionBasedGrasps(ros::NodeHandle& nh, ::pcl::ModelCoefficientsConstPtr& model) {
    // turn cylinder coefficients into eigen vectors
    Eigen::Vector3f centroid(model->values[0], model->values[1], model->values[2]);
    Eigen::Vector3f direction(model->values[3], model->values[4], model->values[5]);
    pregrasp_msgs::PreGrasp cylindrical;

    float offset = 0;
    float radius = model->values[6];
    float height = direction.norm();

    centroid += 0.5 * direction;

//    ROS_DEBUG("Cylindrical %f %f %f", centroid[0], centroid[1], centroid[2]);

    cylindrical.header.frame_id = "/base";
    cylindrical.header.stamp = ros::Time::now();
    cylindrical.pregrasp = pregrasp_msgs::PreGrasp::PREGRASP_CYLINDER;
    cylindrical.strategy = pregrasp_msgs::PreGrasp::STRATEGY_POSITION;

    // use direction as roll vector
    Eigen::Vector3f dir_normalized = direction.normalized();
    Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Vector3f z_axis_projection = (z_axis - z_axis.dot(dir_normalized) * dir_normalized).normalized();
    Eigen::Vector3f z_axis_variation = Eigen::AngleAxisf(-0.5, dir_normalized) * z_axis_projection;
    Eigen::Affine3f t;
    getTransformationFromTwoUnitVectorsXYAndOrigin(dir_normalized, z_axis_variation, centroid, t);

    t = t.inverse();

    Eigen::Affine3f cylinder_transform = t;

    std::vector<std::vector<int > > cylindrical_coordinates;
    std::vector<int > along_axis;

    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.06), -0.06, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.04), -0.04, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.02), -0.02, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, 0.0), 0.0, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.02), +0.02, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.04), +0.04, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.06), +0.06, -0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    cylindrical_coordinates.push_back(along_axis);

    z_axis_variation = Eigen::AngleAxisf(0, dir_normalized) * z_axis_projection;
    getTransformationFromTwoUnitVectorsXYAndOrigin(dir_normalized, z_axis_variation, centroid, t);
    t = t.inverse();

    along_axis.clear();
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.06), -0.06, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.04), -0.04, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.02), -0.02, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, 0.0), 0.0, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.02), +0.02, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.04), +0.04, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.06), +0.06, 0, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    cylindrical_coordinates.push_back(along_axis);

    z_axis_variation = Eigen::AngleAxisf(0.5, dir_normalized) * z_axis_projection;
    getTransformationFromTwoUnitVectorsXYAndOrigin(dir_normalized, z_axis_variation, centroid, t);
    t = t.inverse();

    along_axis.clear();
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.06), -0.06, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.04), -0.04, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, -0.02), -0.02, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, 0.0), 0.0, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.02), +0.02, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.04), +0.04, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    addCylindricalGrasp(cylindrical, getGraspTransform(t, radius, height, +0.06), +0.06, 0.5, height);
    along_axis.push_back(cylindrical.poses.size() - 1);
    cylindrical_coordinates.push_back(along_axis);

    /*static ros::Publisher pub_viz = nh.advertise<geometry_msgs::PoseStamped>("pregrasp_pose", 10);
    geometry_msgs::PoseStamped msg_stamped;
    msg_stamped.header.stamp = ros::Time::now();
    msg_stamped.header.frame_id = "/camera_rgb_optical_frame"; // depth!
    msg_stamped.pose = msg;
    pub_viz.publish(msg_stamped);*/

    broadcastRVizMarkers(nh, cylindrical);
    broadcastBestGrasp(nh, cylindrical, cylindrical_coordinates, dir_normalized);
    broadcastExplorationDirection(centroid, dir_normalized, cylinder_transform, radius);
  }

  int process(const tendrils& /*inputs*/, const tendrils& /*outputs*/) {
    static ros::NodeHandle nh;

    broadcastCylindricalPositionBasedGrasps(nh, *bounded_cylinder_coefficients_);

    static ros::Publisher pub = nh.advertise<pregrasp_msgs::PreGrasp>("pregrasp", 100);
    pregrasp_msgs::PreGrasp pg;
    pg.header.frame_id = "/grasp_1";
    pub.publish(pg);

/*
    ROS_DEBUG("eigenvalues: %f %f %f", (*eigen_values_)[0], (*eigen_values_)[1], (*eigen_values_)[2]);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3((*xyz_centroid_)[0], (*xyz_centroid_)[1], (*xyz_centroid_)[2]) );
    transform.setBasis( tf::Matrix3x3((*eigen_vectors_)(0,0), (*eigen_vectors_)(1,0), (*eigen_vectors_)(2,0),
                                      (*eigen_vectors_)(0,1), (*eigen_vectors_)(1,1), (*eigen_vectors_)(2,1),
                                      (*eigen_vectors_)(0,2), (*eigen_vectors_)(1,2), (*eigen_vectors_)(2,2) ));
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame", "object"));

    transform.setBasis( transform.getBasis().transpose() );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame", "object"));

    tf::Matrix3x3 rot = transform.getBasis();

    static double const offset = 0.07;

    transform.setIdentity();
    transform.setOrigin( tf::Vector3(-(*eigen_values_)[0] * 0.1f - offset, 0, 0) );
    transform.getBasis().setEulerYPR(0, M_PI_2, 0);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/object", "grasp_1"));

    transform.setIdentity();
    transform.setOrigin( tf::Vector3(0, -(*eigen_values_)[1] * 0.1f - offset, 0) );
    transform.getBasis().setRPY(-M_PI_2, M_PI_2, 0);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/object", "grasp_2"));

    transform.setIdentity();
    transform.setOrigin( tf::Vector3(0, (*eigen_values_)[1] * 0.1f + offset, 0) );
    transform.getBasis().setRPY(M_PI_2, M_PI_2, 0);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/object", "grasp_3"));

    pregrasp_msgs::PreGrasp pg;
    pg.header.frame_id = "/grasp_1";
    pub.publish(pg);
    pg.header.frame_id = "/grasp_2";
    pub.publish(pg);
    pg.header.frame_id = "/grasp_3";
    pub.publish(pg);
*/

    return ecto::OK;
  }
  spore< UnalignedMatrix3f> eigen_vectors_;
  spore< UnalignedVector3f> eigen_values_;
  spore< UnalignedVector4f> xyz_centroid_;

  spore<std_msgs::Header> header_;

  spore< ::pcl::ModelCoefficientsConstPtr> bounded_cylinder_coefficients_;
  spore< ::pcl::ModelCoefficientsConstPtr> obstacle_plane_coefficients_;
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::BroadcastGrasps, "BroadcastGrasps", "Broadcast some grasps.");
