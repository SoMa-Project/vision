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

struct LandingMotions
{
    spore<pregrasp_msgs::GraspStrategyArrayConstPtr> landing_pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> landing_manifolds_;

    spore< UnalignedAffine3f > transformation_;
//    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;
//    spore<std::vector< ::pcl::PointIndices > > clusters_;

    ros::Time last_marker_message_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare< UnalignedAffine3f >("transformation", "Bounding Box frame.");
//        inputs.declare<std::vector< ::pcl::PointIndices> >("clusters", "Landing clusters.");
//        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "3D Polygons.");

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("landing_pregrasp_messages", "All the grasps that should be used.");
        outputs.declare< ::posesets::PoseSetArrayConstPtr>("landing_manifolds", "All the grasps that should be used.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
//        clusters_ = inputs["clusters"];
//        bounded_planes_ = inputs["bounded_planes"];

        transformation_ = inputs["transformation"];

        landing_pregrasp_messages_ = outputs["landing_pregrasp_messages"];
        landing_manifolds_ = outputs["landing_manifolds"];

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
        pregrasp_msgs::GraspStrategyArrayPtr landing_messages(new ::pregrasp_msgs::GraspStrategyArray());
        ::posesets::PoseSetArrayPtr landing_manifolds(new ::posesets::PoseSetArray());

        landing_messages->header = pcl_conversions::fromPCL(input->header);

//        for (std::vector< ::pcl::PointIndices >::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
//        for (std::vector< ::pcl::ModelCoefficientsConstPtr >::iterator it = bounded_planes_->begin(); it != bounded_planes_->end(); ++it)
//        {
//            size_t index = std::distance(clusters_->begin(), it);
//            ROS_INFO("Generating Landing No. %zu.", index);

            ::pregrasp_msgs::GraspStrategy msg;
            msg.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_BOX;
            msg.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_LAND;

            // goal of landing is the center of the cluster and it's centroid
//            Eigen::Vector4f centroid;
//            Eigen::Vector3f mean_normal(0, 0, 0);

            Eigen::Affine3d transform = transformation_->cast<double>();
            tf::Transform tf_transform;
            tf::transformEigenToTF(transform, tf_transform);

            tf_transform *= tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, +0.0));

            tf::poseTFToMsg(tf_transform, msg.pregrasp_pose.center.pose);
            msg.pregrasp_pose.pose.pose = msg.pregrasp_pose.center.pose;

//            Eigen::Vector4f centroid((*it)->values[0], (*it)->values[1], (*it)->values[2], 0);
//            Eigen::Vector3f mean_normal((*it)->values[3], (*it)->values[4], (*it)->values[5]);
////            ::pcl::compute3DCentroid(*input, *it, centroid);
////            computeNormalMean(*normals, *it, mean_normal);
//            Eigen::Vector3f starting_point = centroid.topRows(3) - 0.5 * mean_normal;
//            msg.pregrasp_pose.center.pose.position.x = starting_point(0);
//            msg.pregrasp_pose.center.pose.position.y = starting_point(1);
//            msg.pregrasp_pose.center.pose.position.z = starting_point(2);
//            tf::Vector3 avg_normal(mean_normal(0), mean_normal(1), mean_normal(2));
//            tf::Vector3 z_axis(0, 0, 1);
//            tf::quaternionTFToMsg(tf::shortestArcQuat(z_axis, avg_normal), msg.pregrasp_pose.center.pose.orientation);

            msg.pregrasp_pose.size.push_back(0.05);
            msg.pregrasp_pose.size.push_back(0.05);
            msg.pregrasp_pose.size.push_back(0.05);
            msg.pregrasp_pose.size.push_back(3.2);

            msg.pregrasp_pose.image_size.push_back(0.5);
            msg.pregrasp_pose.image_size.push_back(3.2);

            // end of slide
            // HINT: could also be the mean of the seeds region
//            msg.pregrasp_pose.pose.pose.position.x = centroid(0);
//            msg.pregrasp_pose.pose.pose.position.y = centroid(1);
//            msg.pregrasp_pose.pose.pose.position.z = centroid(2);
//            msg.pregrasp_pose.pose.pose.orientation = msg.pregrasp_pose.center.pose.orientation;

//            // set object pose relative to hand
            msg.object.center.pose = msg.object.pose.pose = msg.pregrasp_pose.center.pose;
            msg.object.size.push_back(10.0);
            msg.object.size.push_back(10.0);
            msg.object.size.push_back(10.0);
            msg.object.size.push_back(4.0);
            msg.object.image_size.push_back(0.09);
            msg.object.image_size.push_back(0.09);
            msg.object.image_size.push_back(0.05);
            msg.object.image_size.push_back(4.0);

            landing_messages->strategies.push_back(msg);

            // add the corresponding manifold
            ::posesets::PoseSet ps(tf_transform);
            ps.setPositions(tf::Vector3(0.07, 0.06, 0.02));
//            ps.getOrientations().addFuzzy(tf_transform.getRotation(), tf_transform.getBasis().getColumn(2));
            ps.getOrientations().add(tf_transform.getRotation());
            ps.getOrientations().add(tf_transform.getRotation() * tf::createQuaternionFromRPY(0, 0, M_PI_2));
            ps.getOrientations().add(tf_transform.getRotation() * tf::createQuaternionFromRPY(0, 0, M_PI));
            ps.getOrientations().add(tf_transform.getRotation() * tf::createQuaternionFromRPY(0, 0, -M_PI_2));
            landing_manifolds->push_back(ps);

//            for (int i = 0; i < 3; ++i)
//            {
//                tf_transform *= tf::Transform(tf::createQuaternionFromYaw(M_PI_2));
//                tf::poseTFToMsg(tf_transform, msg.pregrasp_pose.center.pose);
//                msg.pregrasp_pose.pose.pose = msg.pregrasp_pose.center.pose;
//                landing_messages->strategies.push_back(msg);
//            }


            // add other box direction

//            ROS_INFO("Landing pointing from: %f, %f, %f", msg.pregrasp_pose.center.pose.position.x, msg.pregrasp_pose.center.pose.position.y, msg.pregrasp_pose.center.pose.position.z);
//            ROS_INFO("                   to: %f, %f, %f", msg.pregrasp_pose.pose.pose.position.x, msg.pregrasp_pose.pose.pose.position.y, msg.pregrasp_pose.pose.pose.position.z);
//        }

        (*landing_pregrasp_messages_) = landing_messages;
        (*landing_manifolds_) = landing_manifolds;

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::LandingMotions>, "LandingMotions", "Finding landing motions.");
