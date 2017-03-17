#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

// input are slide and edge grasp affordances
// output is instantiated transition

struct TransitionSlideEdgeGrasp
{
  ecto::spore<pregrasp_msgs::GraspStrategyArray> slide_messages_;
  ecto::spore<pregrasp_msgs::GraspStrategyArray> edge_grasp_messages_;
  ecto::spore<pregrasp_msgs::GraspStrategyArray> transition_messages_;

  spore<ecto::pcl::Clusters> planar_patches_;
  spore<std::vector<UnalignedVector4f> > plane_centroids_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > edge_coefficients_;

  ros::Time last_marker_message_;

  // parameters
//  spore<double> distance_threshold_;
//  spore<double> min_inlier_ratio_;
//  spore<double> weight_contour_;
//  spore<double> min_boxness_;
//  spore<double> max_size_;
//  spore<double> min_size_;

  static void declare_params(ecto::tendrils& params)
  {
//    params.declare<double>("min_inlier_ratio", "Minimum number of inlier points per plane.", 0.5);
//    params.declare<double>("distance_threshold", "Maximum mean error a plane fit may have.", 0.05);
//    params.declare<double>("min_boxness", "", 0.8);
//    params.declare<double>("weight_contour", "", 0.5);
//    params.declare<double>("max_size", "", 0.24);
//    params.declare<double>("min_size", "", 0.08);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare< ecto::pcl::Clusters >("planar_patches", "Planar patches.");
    inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("plane_coefficients", "Plane coefficients.");
    inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("edge_coefficients", "Edge coefficients.");
    inputs.declare<std::vector<UnalignedVector4f> >("plane_centroids", "Plane centroids.");

    inputs.declare<pregrasp_msgs::GraspStrategyArray>("edge_grasp_messages", "");
    inputs.declare<pregrasp_msgs::GraspStrategyArray>("slide_messages", "");

    outputs.declare<pregrasp_msgs::GraspStrategyArray>("transition_messages", "");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    planar_patches_ = inputs["planar_patches"];
    edge_coefficients_ = inputs["edge_coefficients"];
    plane_centroids_ = inputs["plane_centroids"];
    plane_coefficients_ = inputs["plane_coefficients"];

    edge_grasp_messages_ = inputs["edge_grasp_messages"];
    slide_messages_ = inputs["slide_messages"];

    transition_messages_ = outputs["transition_messages"];

    last_marker_message_ = ros::Time::now();
  }

  void publishRVizMarkers(const std::vector<std::vector<size_t> >& edge_to_plane, const std::string& frame_id)
  {
    static ros::NodeHandle nh;
    static ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/sliding_grasp", 20);

    visualization_msgs::MarkerArray marker_msgs;

//    static ros::Publisher obstacle_publisher = nh.advertise<visualization_msgs::Marker>("/obstacle_fits", 10);

    int counter = 0;

    for (std::vector<std::vector<size_t> >::const_iterator it = edge_to_plane.begin(); it != edge_to_plane.end(); ++it)
    {
      size_t edge_id = it - edge_to_plane.begin();

      for (std::vector<size_t>::const_iterator it2 = it->begin(); it2 != it->end(); ++it2)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Time::now() - last_marker_message_;
        marker.id = counter++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.points.resize(2);
//        marker.points[0].x = plane_centroids_->at(*it2)(0);
//        marker.points[0].y = plane_centroids_->at(*it2)(1);
//        marker.points[0].z = plane_centroids_->at(*it2)(2);
        marker.points[0].x = pregrasp_messages_->strategies.at(*it2).pregrasp_pose.pose.pose.position.x;
        marker.points[0].y = pregrasp_messages_->strategies.at(*it2).pregrasp_pose.pose.pose.position.y;
        marker.points[0].z = pregrasp_messages_->strategies.at(*it2).pregrasp_pose.pose.pose.position.z;
        marker.points[1].x = edge_coefficients_->at(edge_id)->values[0];
        marker.points[1].y = edge_coefficients_->at(edge_id)->values[1];
        marker.points[1].z = edge_coefficients_->at(edge_id)->values[2];

        marker.scale.x = 0.025;
        marker.scale.y = 0.05;

        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1;

        marker_msgs.markers.push_back(marker);
      }
    }

    last_marker_message_ = ros::Time::now();
    marker_publisher.publish(marker_msgs);
  }

  template<typename Point>
  inline float pointToLineDistance(const Point p, ::pcl::ModelCoefficientsConstPtr& line)
  {
//    ::Eigen::Vector4f point(p.x, p.y, p.z, 0.0f);
//    ::Eigen::Vector4f line_point(line->values[0], line->values[1], line->values[2], 0.0f);
//    ::Eigen::Vector4f line_direction(line->values[3], line->values[4], line->values[5], 0.0f);
//    return ::pcl::sqrPointToLineDistance(point, line_point, line_direction);
    return (p.x-line->values[0]) * (p.x-line->values[0]) + (p.y-line->values[1]) * (p.y-line->values[1]) + (p.z-line->values[2]) * (p.z-line->values[2]);
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
      *sliding_pregrasp_messages_ = *pregrasp_messages_;
      sliding_pregrasp_messages_->strategies.clear();

      std::vector<std::vector<size_t> > edge_to_plane;

      ROS_INFO("Edges: %zu x  Grasps: %zu", edge_coefficients_->size(), pregrasp_messages_->strategies.size());

      // compare all slides with all edge grasps
      for (::std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = edge_coefficients_->begin(); it != edge_coefficients_->end(); ++it)
      {
        std::vector<size_t> current_edge_to_plane;

        //ROS_INFO("PLANAR PATCHES: %zu", planar_patches_->size());

        /*
//        for (ecto::pcl::Clusters::iterator c_it = planar_patches_->begin(); c_it != planar_patches_->end(); ++c_it)
        for (::std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator c_it = plane_coefficients_->begin(); c_it != plane_coefficients_->end(); ++c_it)
        {

          size_t id = c_it - plane_coefficients_->begin();
          float min_distance = 1e10;

//          for (std::vector<int>::iterator p_it = c_it->indices.begin(); p_it != c_it->indices.end(); ++p_it)
//          {
//            min_distance = std::min(min_distance, pointToLineDistance(input->at(*p_it), *it));
//          }
//
//          if (min_distance < 0.03*0.03)
        */

            UnalignedVector3f edge_centroid((*it)->values[0], (*it)->values[1], (*it)->values[2]);
            UnalignedVector3f edge_direction((*it)->values[3], (*it)->values[4], (*it)->values[5]);
            double edge_length = edge_direction.norm();
            edge_direction.normalize();
            //UnalignedVector3f plane_centroid(plane_centroids_->at(id)(0), plane_centroids_->at(id)(1), plane_centroids_->at(id)(2));
            //UnalignedVector3f plane_normal((*c_it)->values[0], (*c_it)->values[1], (*c_it)->values[2]);
            //plane_normal.normalize();

            //UnalignedVector3f plane_to_edge = (edge_centroid - plane_centroid);
            //plane_to_edge.normalize();

            Eigen::Vector3f approach_y = edge_centroid.normalized().cross(edge_direction);
            Eigen::Vector3f approach_z = edge_direction.cross(approach_y);
            Eigen::Matrix3f rotation;
            rotation << edge_direction, approach_y, approach_z;

            // rotated the other way around
            approach_y = edge_centroid.normalized().cross(-edge_direction);
            approach_z = (-edge_direction).cross(approach_y);
            Eigen::Matrix3f rotation2;
            rotation2 << -edge_direction, approach_y, approach_z;

            //if (acos(plane_to_edge.dot(plane_normal)) > ::pcl::deg2rad(75.0f))
            {
              // iterate through all grasps and check for shittyness
              for (std::vector<pregrasp_msgs::GraspStrategy>::iterator p_it = pregrasp_messages_->strategies.begin(); p_it != pregrasp_messages_->strategies.end(); ++p_it)
              {
                // check if pregrasp is box - otherwise continue

                UnalignedVector3f grasp(p_it->pregrasp_pose.pose.pose.position.x, p_it->pregrasp_pose.pose.pose.position.y, p_it->pregrasp_pose.pose.pose.position.z);
                UnalignedVector3f grasp_to_edge = edge_centroid - grasp;
                grasp_to_edge.normalize();

                //ROS_INFO("How i met your mother %f %f", grasp_to_edge.dot(plane_normal), plane_to_edge.dot(grasp_to_edge));

//                if (acos(grasp_to_edge.dot(plane_normal)) > ::pcl::deg2rad(85.0f))//
//                if (fabs(grasp_to_edge.dot(plane_normal)) > 0.95 && fabs(plane_to_edge.dot(grasp_to_edge)) > 0.95f)
                //if (fabs(plane_to_edge.dot(grasp_to_edge)) > 0.95f && fabs(grasp_to_edge.dot(plane_normal)) < 0.2f)
                {
                  pregrasp_msgs::GraspStrategy g(*p_it);

                  g.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER;
                  g.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE;

                  g.pregrasp_pose.center = g.pregrasp_pose.pose;

                  //g.quality_grasp = g.quality_approach = g.quality_closing = 0.6f;

//                  ::Eigen::Affine3f transformation;
//                  ::pcl::getTransformationFromTwoUnitVectorsAndOrigin(edge_direction, plane_normal, edge_centroid, transformation);
                  g.pregrasp_pose.pose.pose.position.x = edge_centroid(0);
                  g.pregrasp_pose.pose.pose.position.y = edge_centroid(1);
                  g.pregrasp_pose.pose.pose.position.z = edge_centroid(2);

                  ::Eigen::Quaternionf graspq(g.pregrasp_pose.center.pose.orientation.w, g.pregrasp_pose.center.pose.orientation.x, g.pregrasp_pose.center.pose.orientation.y, g.pregrasp_pose.center.pose.orientation.z);

                  ::Eigen::Matrix3f final_rot = rotation2;
                  if (rotation.col(0).dot(graspq.matrix().col(0)) > rotation2.col(0).dot(graspq.matrix().col(0)))
                      final_rot = rotation;

                  // this is for making sure that there's a surface that can be used
                  bool exists_surface = false;
                  for (size_t i = 0; i < plane_coefficients_->size() && !exists_surface; ++i)
                  {
                      UnalignedVector3f plane_centroid(plane_centroids_->at(i)(0), plane_centroids_->at(i)(1), plane_centroids_->at(i)(2));
                      UnalignedVector3f plane_normal(plane_coefficients_->at(i)->values[0], plane_coefficients_->at(i)->values[1], plane_coefficients_->at(i)->values[2]);

                      if ((plane_centroid-grasp).norm() < 0.3 && fabs(plane_normal.dot(graspq.matrix().col(2))) > 0.8)
                          exists_surface = true;
                  }
                  if (!exists_surface)
                      continue;

                  // this is for making sure that the edge direction is within the plane of the hand motion
                  //if (fabs(edge_direction.dot(graspq.matrix().col(2))) > 0.1)
                  //    continue;

                  // this is for making sure that the edge location is within the plane of the hand motion
                  //if (fabs(grasp_to_edge.dot(graspq.matrix().col(2))) > 0.1)
                  //    continue;


                  ::Eigen::Quaternionf q(final_rot);
                  g.pregrasp_pose.pose.pose.orientation.x = q.x();
                  g.pregrasp_pose.pose.pose.orientation.y = q.y();
                  g.pregrasp_pose.pose.pose.orientation.z = q.z();
                  g.pregrasp_pose.pose.pose.orientation.w = q.w();

                  sliding_pregrasp_messages_->strategies.push_back(g);
                  current_edge_to_plane.push_back(p_it - pregrasp_messages_->strategies.begin());
                }
              }
          //  }
        }

        edge_to_plane.push_back(current_edge_to_plane);
      }


      ROS_INFO("Number of combinations %zu", edge_to_plane.size());
      publishRVizMarkers(edge_to_plane, "/camera_depth_optical_frame");

      return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::TransitionSlideEdgeGrasp>, "TransitionSlideEdgeGrasp", "Calculate whether a transition between a particular Slide and Edge grasp affordance is possible.");
