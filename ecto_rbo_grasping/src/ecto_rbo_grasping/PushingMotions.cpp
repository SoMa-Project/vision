/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

#include <Eigen/Geometry>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct PushingMotions
{
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pushing_pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> pushing_manifolds_;

    spore<std::vector< ::pcl::PointIndices > > clusters_;
    spore<std::vector< ::pcl::PointIndices > > cluster_seeds_;
    spore<std::vector< ::pcl::PointIndices > > cluster_borders_;

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
        inputs.declare<std::vector< ::pcl::PointIndices> >("clusters", "Clusters.");
        inputs.declare<std::vector< ::pcl::PointIndices> >("cluster_seeds", "Clusters.");
        inputs.declare<std::vector< ::pcl::PointIndices> >("cluster_borders", "Clusters.");

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pushing_pregrasp_messages", "All the grasps that should be used.");
        outputs.declare< ::posesets::PoseSetArrayConstPtr>("pushing_manifolds", "All the grasps that should be used.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        clusters_ = inputs["clusters"];
        cluster_seeds_ = inputs["cluster_seeds"];
        cluster_borders_ = inputs["cluster_borders"];

        pushing_pregrasp_messages_ = outputs["pushing_pregrasp_messages"];
        pushing_manifolds_ = outputs["pushing_manifolds"];

        last_marker_message_ = ros::Time::now();
    }

    inline void computeNormalMean(const ::pcl::PointCloud< ::pcl::Normal>& normals, const ::pcl::PointIndices& indices, Eigen::Vector3f& mean_normal)
    {
        mean_normal.setZero();
        for (size_t i = 0; i < indices.indices.size(); ++i)
            mean_normal += normals.at(indices.indices[i]).getNormalVector3fMap();

        mean_normal.normalize();
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
        pregrasp_msgs::GraspStrategyArrayPtr push_messages(new ::pregrasp_msgs::GraspStrategyArray());
        ::posesets::PoseSetArrayPtr pushing_manifolds(new ::posesets::PoseSetArray());

        push_messages->header = pcl_conversions::fromPCL(input->header);

        for (std::vector< ::pcl::PointIndices >::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
        {
            size_t index = std::distance(clusters_->begin(), it);

            if (clusters_->at(index).indices.empty() || cluster_seeds_->at(index).indices.empty() || cluster_borders_->at(index).indices.empty())
                continue;

//            ROS_INFO("Generating Push No. %zu.: %zu (cluster size), %zu (seed size), %zu (border size)", index, clusters_->at(index).indices.size(), cluster_seeds_->at(index).indices.size(), cluster_borders_->at(index).indices.size());

            ::pregrasp_msgs::GraspStrategy msg;
            msg.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_HOOK;
            msg.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_PUSH;

            // start of slide
            Eigen::Vector4f seed_centroid, border_centroid;
            Eigen::Vector3f mean_normal(0, 0, 0);
            ::pcl::compute3DCentroid(*input, cluster_seeds_->at(index), seed_centroid);
            ::pcl::compute3DCentroid(*input, cluster_borders_->at(index), border_centroid);
            computeNormalMean(*normals, cluster_seeds_->at(index), mean_normal);
            msg.pregrasp_pose.center.pose.position.x = seed_centroid(0);
            msg.pregrasp_pose.center.pose.position.y = seed_centroid(1);
            msg.pregrasp_pose.center.pose.position.z = seed_centroid(2);
            /*
            tf::Vector3 z_axis(0, 0, -1);
            tf::Vector3 avg_normal(mean_normal(0), mean_normal(1), mean_normal(2));
            tf::quaternionTFToMsg(tf::shortestArcQuat(z_axis, avg_normal), msg.pregrasp_pose.center.pose.orientation);
            */
            Eigen::Vector3f push_direction = (border_centroid.topRows(3) - seed_centroid.topRows(3)).normalized();
            Eigen::Vector3f approach_z = -mean_normal;
            Eigen::Vector3f approach_x = push_direction.cross(approach_z);
            Eigen::Matrix3f rotation;
            rotation << approach_x, approach_z.cross(approach_x), approach_z;
            Eigen::Quaterniond q_eigen(rotation.cast<double>());
            tf::Quaternion q_tf;
            tf::quaternionEigenToTF(q_eigen, q_tf);
            tf::quaternionTFToMsg(q_tf, msg.pregrasp_pose.center.pose.orientation);

            ::pcl::PointCloud<Point> tmp_cloud;
            ::Eigen::Affine3f seed_transform(::Eigen::Translation3f(seed_centroid.topRows(3)));
            seed_transform.rotate(::Eigen::AngleAxisf(q_eigen.cast<float>()));
            ::pcl::transformPointCloud(*input, cluster_seeds_->at(index), tmp_cloud, seed_transform.inverse());
            ::Eigen::Vector4f max_point, min_point;
            ::pcl::getMinMax3D(tmp_cloud, min_point, max_point);
            ::Eigen::Vector4f difference = max_point - min_point;

            msg.pregrasp_pose.size.resize(4);
            msg.pregrasp_pose.size[0] = difference[0];
            msg.pregrasp_pose.size[1] = difference[1];
            msg.pregrasp_pose.size[2] = difference[2];
            msg.pregrasp_pose.size[3] = 0.1;

            // end of slide
            // HINT: could also be the mean of the seeds region
            computeNormalMean(*normals, cluster_borders_->at(index), mean_normal);
            msg.pregrasp_pose.pose.pose.position.x = border_centroid(0);
            msg.pregrasp_pose.pose.pose.position.y = border_centroid(1);
            msg.pregrasp_pose.pose.pose.position.z = border_centroid(2);

            /*
            avg_normal.setValue(mean_normal(0), mean_normal(1), mean_normal(2));
            tf::quaternionTFToMsg(tf::shortestArcQuat(z_axis, avg_normal), msg.pregrasp_pose.pose.pose.orientation);
            */
            approach_z = -mean_normal;
            approach_x = push_direction.cross(approach_z);
            rotation << approach_x, approach_z.cross(approach_x), approach_z;
            q_eigen = rotation.cast<double>();
            tf::quaternionEigenToTF(q_eigen, q_tf);
            tf::quaternionTFToMsg(q_tf, msg.pregrasp_pose.pose.pose.orientation);

            tmp_cloud.clear();
            ::Eigen::Affine3f border_transform(::Eigen::Translation3f(border_centroid.topRows(3)));
            border_transform.rotate(::Eigen::AngleAxisf(q_eigen.cast<float>()));
            ::pcl::transformPointCloud(*input, cluster_borders_->at(index), tmp_cloud, border_transform.inverse());
            ::pcl::getMinMax3D(tmp_cloud, min_point, max_point);
            difference = max_point - min_point;

//            sensor_msgs::PointCloud2 color_msg;
//            ::pcl::toROSMsg(tmp_cloud, color_msg);
//            color_msg.header.frame_id = input->header.frame_id;
//            color_msg.header.stamp = ::ros::Time::now();
//            static ::ros::NodeHandle nh;
//            static ::ros::Publisher debug_publisher = nh.advertise< sensor_msgs::PointCloud2 > ("/debug_shit", 1);
//            debug_publisher.publish(color_msg);

            msg.pregrasp_pose.image_size.resize(4);
            msg.pregrasp_pose.image_size[0] = difference[0];
            msg.pregrasp_pose.image_size[1] = difference[1];
            msg.pregrasp_pose.image_size[2] = difference[2];
            msg.pregrasp_pose.image_size[3] = 0.1;

            // set object pose relative to hand
            msg.object.center.pose = msg.object.pose.pose = msg.pregrasp_pose.center.pose;
            msg.object.size.push_back(0.1);
            msg.object.size.push_back(0.1);
            msg.object.size.push_back(0.07);
            msg.object.size.push_back(4.0);
            msg.object.image_size.push_back(0.01);
            msg.object.image_size.push_back(0.1);
            msg.object.image_size.push_back(0.1);
            msg.object.image_size.push_back(4.0);

            push_messages->strategies.push_back(msg);

            // add corresponding manifold
            ::posesets::PoseSet ps(tf::Transform(q_tf, tf::Vector3(seed_centroid(0), seed_centroid(1), seed_centroid(2))));
            ps.setPositions(tf::Vector3(difference[0], difference[1], difference[2]));
            ps.getOrientations().add(q_tf, tf::Vector3(approach_z(0), approach_z(1), approach_z(2)));
            pushing_manifolds->push_back(ps);

            ROS_INFO("Push pointing from: %f, %f, %f", msg.pregrasp_pose.center.pose.position.x, msg.pregrasp_pose.center.pose.position.y, msg.pregrasp_pose.center.pose.position.z);
            ROS_INFO("                to: %f, %f, %f", msg.pregrasp_pose.pose.pose.position.x, msg.pregrasp_pose.pose.pose.position.y, msg.pregrasp_pose.pose.pose.position.z);
        }

        (*pushing_pregrasp_messages_) = push_messages;
        (*pushing_manifolds_) = pushing_manifolds;

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::PushingMotions>, "PushingMotions", "Finding pushing motions.");
