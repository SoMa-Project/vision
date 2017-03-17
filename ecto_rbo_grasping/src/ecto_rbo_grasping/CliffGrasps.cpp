#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct CliffGrasps
{
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> cliff_pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> cliff_manifolds_;

    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;

    ecto::spore<int> max_no_of_points_;
    ecto::spore<bool> invert_y_axis_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<int>("max_no_of_points", "", 10);
        params.declare<bool>("invert_y_axis", "Whether to do an edge/cliff grasp by pushing or pulling. Depends on the coordinate Frame attached to the hand (y-axis is supposed to lie within palm).", false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygons", "3D Polygons.");
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "3D Polygons.");

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("cliff_pregrasp_messages", "All the grasps that should be used.");
        outputs.declare< ::posesets::PoseSetArrayConstPtr>("cliff_manifolds", "All the grasps that should be used.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        polygons_ = inputs["polygons"];
        bounded_planes_ = inputs["bounded_planes"];

        cliff_pregrasp_messages_ = outputs["cliff_pregrasp_messages"];
        cliff_manifolds_ = outputs["cliff_manifolds"];

        max_no_of_points_ = params["max_no_of_points"];
        invert_y_axis_ = params["invert_y_axis"];
    }

    inline bool intersectRayBox(//const ::Eigen::Vector3f& ray_org, == 0
                           const ::Eigen::Vector3f& ray_dir,
                           const ::Eigen::Vector3f& box_org,
                           const ::Eigen::Matrix3f& box_rot,
                           const ::Eigen::Vector3f& box_size,
                           float &t0, float &t1)
    {
      float epsilon = 10e-4;

      int parallel = 0;
      bool found = false;

      float DA[3];
      float dA[3];
  //    ::Eigen::Vector3f d = box_org;// - ray_org;

      for (int i = 0; i < 3; ++i)
      {
        DA[i] = ray_dir.dot(box_rot.col(i));
        dA[i] = box_org.dot(box_rot.col(i));

        if (fabs(DA[i]) < epsilon)
          parallel |= 1 << i;
        else
        {
          float es = (DA[i] > 0.0) ? box_size(i) : -box_size(i);
          float invDA = 1.0 / DA[i];

          if (!found)
          {
            t0 = (dA[i] - es) * invDA;
            t1 = (dA[i] + es) * invDA;
            found = true;
          }
          else
          {
            float s = (dA[i] - es) * invDA;

            if (s > t0)
              t0 = s;

            s = (dA[i] + es) * invDA;

            if (s < t1)
              t1 = s;

            if (t0 > t1)
              return false;
          }
        }
      }

      if (parallel)
        for (int i = 0; i < 3; ++i)
          if (parallel & (1 << i))
            if (fabs(dA[i] - t0 * DA[i]) > box_size(i) || fabs(dA[i] - t1 * DA[i]) > box_size(i))
              return false;

      return found;
    }

    template <typename Point>
    int cropByRayIntersection (const ::pcl::PointCloud<Point> &cloud,
//                              const std::vector<int> &indices,
                              const ::Eigen::Vector3f& box_org,
                              const ::Eigen::Matrix3f& box_rot,
                              const ::Eigen::Vector3f& box_size,
                              std::vector<int> &cropped_indices,
                              int &occluding_point_count)
    {
      cropped_indices.clear();

      int intersection_point_count = 0;
      occluding_point_count = 0;

//      for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
      for (size_t i = 0; i < cloud.size(); ++i)
      {
//        if (!isFinite (cloud[*iIt]))
        if (!isFinite (cloud.points[i]))
          continue;

        float t0, t1;
        ::Eigen::Vector3f ray_dir(cloud[i].x, cloud[i].y, cloud[i].z);
//        ::Eigen::Vector3f ray_dir(cloud[*iIt].x, cloud[*iIt].y, cloud[*iIt].z);
        float l = ray_dir.norm();
        ray_dir.normalize();
        if (intersectRayBox(ray_dir, box_org, box_rot, box_size, t0, t1))
        {
          intersection_point_count++;
  //        if ((t0 <= l && l <= t1) || (t1 <= l && l <= t0))
          if (l <= t0)
            occluding_point_count++;
          else if (t0 <= l && l <= t1)
//            cropped_indices.push_back(*iIt);
            cropped_indices.push_back(i);
        }
      }
      return intersection_point_count;
    }

    template<typename Point>
    int countPoints(const pregrasp_msgs::GraspStrategy& g, boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                    float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
    {
        UnalignedVector4f min, max;
        max << max_x, max_y, max_z, 0;
        min << min_x, min_y, min_z, 0;

        ::tf::Transform bt;
        ::tf::poseMsgToTF(g.pregrasp_pose.pose.pose, bt);
        ::Eigen::Affine3d approach_transform_d;
        ::tf::transformTFToEigen(bt, approach_transform_d);

        UnalignedAffine3f approach_transform = approach_transform_d.cast<float>();// * Eigen::Translation3f(0, 0, -approach_length_2);

//        std::vector<int> cropped_indices;
//        ::Eigen::Vector3f box_org = approach_transform.translation();
//        UnalignedVector4f side_lengths = 0.5 * (max - min);
//        ::Eigen::Vector3f box_size = side_lengths.topRows(3);
//        ::Eigen::Matrix3f box_rot = approach_transform.rotation();
//        int occluding_point_count = 0;
//        int total_point_count = cropByRayIntersection(*input, box_org, box_rot, box_size, cropped_indices, occluding_point_count);
//        ROS_INFO("Other result: occludingpc: %i   total: %i   cropped_ind: %zu", occluding_point_count, total_point_count, cropped_indices.size());

        ::pcl::CropBox<Point> crop_filter;
        crop_filter.setTransform(approach_transform.inverse());
        crop_filter.setMax(max);
        crop_filter.setMin(min);
        crop_filter.setKeepOrganized(true);
        crop_filter.setInputCloud(input);

        ::pcl::PointIndices inliers;
        crop_filter.filter(inliers.indices);

        return inliers.indices.size();
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
        pregrasp_msgs::GraspStrategyArrayPtr cliff_pregrasp_messages(new ::pregrasp_msgs::GraspStrategyArray());
        ::posesets::PoseSetArrayPtr cliff_manifolds(new ::posesets::PoseSetArray());

        cliff_pregrasp_messages->header = pcl_conversions::fromPCL(input->header);

        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = polygons_->begin(); it != polygons_->end(); ++it)
        {
            // assume that it's at least a triangle
            UnalignedVector3f point_a((*it)->values[0], (*it)->values[1], (*it)->values[2]);
            UnalignedVector3f point_b((*it)->values[3], (*it)->values[4], (*it)->values[5]);
            UnalignedVector3f point_c((*it)->values[6], (*it)->values[7], (*it)->values[8]);
            UnalignedVector3f polygon_normal = (point_b - point_a).normalized().cross((point_c - point_b).normalized());
            polygon_normal = -polygon_normal;

            size_t index = std::distance(polygons_->begin(), it);
            polygon_normal(0) = bounded_planes_->at(index)->values[3];
            polygon_normal(1) = bounded_planes_->at(index)->values[4];
            polygon_normal(2) = bounded_planes_->at(index)->values[5];


            // iterate over every edge of every polygon
            size_t number_of_lines = (*it)->values.size() / 3;
            for (size_t line = 0; line < number_of_lines; ++line)
            {
                UnalignedVector3f start_point((*it)->values[3 * line], (*it)->values[3 * line + 1], (*it)->values[3 * line + 2]);
                UnalignedVector3f end_point;
                UnalignedVector3f before_start_point;
                if (line == 0)
                {
                    before_start_point(0) = (*it)->values[3 * number_of_lines - 3];
                    before_start_point(1) = (*it)->values[3 * number_of_lines - 2];
                    before_start_point(2) = (*it)->values[3 * number_of_lines - 1];
                }
                else
                {
                    before_start_point(0) = (*it)->values[3 * line - 3];
                    before_start_point(1) = (*it)->values[3 * line - 2];
                    before_start_point(2) = (*it)->values[3 * line - 1];
                }

                if (line == number_of_lines - 1)
                {
                    end_point(0) = (*it)->values[0];
                    end_point(1) = (*it)->values[1];
                    end_point(2) = (*it)->values[2];
                }
                else {
                    end_point(0) = (*it)->values[3 * line + 3];
                    end_point(1) = (*it)->values[3 * line + 4];
                    end_point(2) = (*it)->values[3 * line + 5];
                }

                UnalignedVector3f edge_centroid = 0.5 * start_point + 0.5 * end_point;
                UnalignedVector3f edge_direction = end_point - start_point;
                double edge_length = edge_direction.norm();
                edge_direction.normalize();

//                Eigen::Vector3f approach_y = edge_centroid.normalized().cross(edge_direction);
//                Eigen::Vector3f approach_z = edge_direction.cross(approach_y);
//                Eigen::Matrix3f rotation;
//                rotation << edge_direction, approach_y, approach_z;

                // rotated the other way around
//                UnalignedVector3f other_edge_direction = before_start_point - start_point;
//                if (*invert_y_axis_)
//                    edge_direction = -edge_direction;
                Eigen::Vector3f approach_z = polygon_normal;
//                approach_y = edge_centroid.normalized().cross(-edge_direction);
                Eigen::Vector3f approach_y = (edge_direction).cross(-approach_z);
                Eigen::Matrix3f rotation2;
                rotation2 << edge_direction, approach_y, approach_z;

                Eigen::Matrix3f orientationset(rotation2);
                if (*invert_y_axis_)
                    orientationset << -edge_direction, -approach_y, approach_z;
                ::Eigen::Matrix3d final_rot2 = orientationset.cast<double>();
                ::Eigen::Quaterniond q_eigen2(final_rot2);
                ::tf::Quaternion q_tf2;
                ::tf::quaternionEigenToTF(q_eigen2, q_tf2);

                pregrasp_msgs::GraspStrategy g;
                g.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER;
                g.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE;

                g.pregrasp_pose.pose.header = cliff_pregrasp_messages->header;
                g.pregrasp_pose.pose.pose.position.x = edge_centroid(0);
                g.pregrasp_pose.pose.pose.position.y = edge_centroid(1);
                g.pregrasp_pose.pose.pose.position.z = edge_centroid(2);

                //                  ::Eigen::Quaternionf graspq(g.pregrasp_pose.center.pose.orientation.w, g.pregrasp_pose.center.pose.orientation.x, g.pregrasp_pose.center.pose.orientation.y, g.pregrasp_pose.center.pose.orientation.z);

                ::Eigen::Matrix3d final_rot = rotation2.cast<double>();
                //                  if (rotation.col(0).dot(graspq.matrix().col(0)) > rotation2.col(0).dot(graspq.matrix().col(0)))
                //                      final_rot = rotation;

                ::Eigen::Quaterniond q_eigen(final_rot);
                ::tf::Quaternion q_tf;
                ::tf::quaternionEigenToTF(q_eigen, q_tf);
                ::tf::quaternionTFToMsg(q_tf, g.pregrasp_pose.pose.pose.orientation);

                // these numbers are based on the non-inverted axis case
                int number_of_cropped_points = countPoints(g, input, -0.07, 0.07, 0.07, 0.17, -0.1, 0.15);
                number_of_cropped_points += countPoints(g, input, -0.07, 0.07, -0.07, 0.07, -0.2, -0.05);
                ROS_DEBUG("Cliff: cropped points %i (max allowed points: %i)", number_of_cropped_points, *max_no_of_points_);

                if (number_of_cropped_points > *max_no_of_points_)
                    continue;

                // just turn it in case
                if (*invert_y_axis_)
                    ::tf::quaternionTFToMsg(q_tf2, g.pregrasp_pose.pose.pose.orientation);
                g.pregrasp_pose.center = g.pregrasp_pose.pose;
                g.pregrasp_pose.center.header = g.pregrasp_pose.pose.header = g.object.center.header = g.object.pose.header = cliff_pregrasp_messages->header;

                g.pregrasp_pose.size.push_back(edge_length);
                g.pregrasp_pose.size.push_back(0.02);
                g.pregrasp_pose.size.push_back(0.01);
                g.pregrasp_pose.image_size.push_back(edge_length);
                g.pregrasp_pose.image_size.push_back(0.02);
                g.pregrasp_pose.image_size.push_back(0.01);

                // set object pose relative to hand
                g.object.center.pose = g.object.pose.pose = g.pregrasp_pose.center.pose;
                g.object.size.push_back(0.05);
                g.object.size.push_back(0.05);
                g.object.size.push_back(0.05);
                g.object.size.push_back(4.0);
                g.object.image_size.push_back(0.005);
                g.object.image_size.push_back(0.005);
                g.object.image_size.push_back(0.005);
                g.object.image_size.push_back(0.1);

                cliff_pregrasp_messages->strategies.push_back(g);

                // add the corresponding manifold
                ::posesets::PoseSet ps(tf::Transform(q_tf, tf::Vector3(edge_centroid(0), edge_centroid(1), edge_centroid(2))));
                ps.setPositions(tf::Vector3(edge_length, 0.01, 0.02));
                ps.getOrientations().addFuzzy(q_tf2);
                cliff_manifolds->push_back(ps);
            }
        }

        (*cliff_pregrasp_messages_) = cliff_pregrasp_messages;
        (*cliff_manifolds_) = cliff_manifolds;

        ROS_INFO("Number of cliff grasps: %zu", (*cliff_pregrasp_messages_)->strategies.size());

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::CliffGrasps>, "CliffGrasps", "A cliff grasp.");
