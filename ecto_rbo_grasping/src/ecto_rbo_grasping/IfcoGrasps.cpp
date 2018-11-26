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
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

using namespace ecto;
using namespace ecto::pcl;

#define pc(x) std::cout << #x << ": " << (x) << std::endl;
#define ps(x) std::cout << #x << std::endl;
#define pv(x) std::cout << #x << ": " << (x).transpose() << std::endl;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

// ======================================================================================================================
// ======================================================================================================================
struct IfcoGrasps
{

    // inputs
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> >  ifco_planes_;
    ecto::spore<double> ifco_length_;
    ecto::spore<double> ifco_width_;
    ecto::spore<double> ifco_height_;


    // outputs
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> wall_pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> wall_manifolds_;



    static void declare_params(ecto::tendrils& params)
    {
    }


    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("ifco_planes", "3D Polygons.");
        inputs.declare<double>("ifco_length", "Size of the long IFCO edge", 0.0);
        inputs.declare<double>("ifco_width", "Size of the short IFCO edge", 0.0);
        inputs.declare<double>("ifco_height", "Depth of the ifco", 0.0);

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("wall_pregrasp_messages", "All the grasps that should be used.");
        outputs.declare< ::posesets::PoseSetArrayConstPtr>("wall_manifolds", "All the grasps that should be used.");
    }


    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        // inputs
        ifco_planes_ = inputs["ifco_planes"];
        ifco_length_ = inputs["ifco_length"];
        ifco_width_ = inputs["ifco_width"];
        ifco_height_ = inputs["ifco_height"];


        // outputs
        wall_pregrasp_messages_ = outputs["wall_pregrasp_messages"];
        wall_manifolds_ = outputs["wall_manifolds"];
    }



    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {

        // Initialize the pregrasp messages
        pregrasp_msgs::GraspStrategyArrayPtr wall_messages(new ::pregrasp_msgs::GraspStrategyArray());
        wall_messages->header = pcl_conversions::fromPCL(input->header);
        ::posesets::PoseSetArrayPtr wall_manifolds(new ::posesets::PoseSetArray());

        // If the IFCO is not detected, don't create any messages
        if(ifco_planes_->empty())
        {
          ROS_ERROR("Ifco could not be detected!!!");
          return QUIT;
        }

        // Create a grasp per wall
        for(int i = 0; i < 4; i++) {

            // Get the plane model
            ::pcl::ModelCoefficientsConstPtr wall = (*ifco_planes_)[i];

            pregrasp_msgs::GraspStrategy g;
            g.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER;
            g.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP;

            g.pregrasp_pose.pose.header = wall_messages->header;
            g.pregrasp_pose.pose.pose.position.x = wall->values[0];
            g.pregrasp_pose.pose.pose.position.y = wall->values[1];
            g.pregrasp_pose.pose.pose.position.z = wall->values[2] - 0.5 * (*ifco_height_);

            // Create the rotation for
            tf::Vector3 normal(wall->values[3], wall->values[4], wall->values[5]);
            normal /= normal.length();
            tf::Vector3 principal_axis(wall->values[6], wall->values[7], wall->values[8]);
            principal_axis /= principal_axis.length();
            tf::Vector3 third_axis = normal.cross(principal_axis);
            third_axis /= third_axis.length();
            Eigen::Matrix3f rotation;
            rotation <<            principal_axis.x(),third_axis.x(), normal.x(),
                    principal_axis.y(),third_axis.y(), normal.y(),
                    principal_axis.z(),third_axis.z(), normal.z();
            ::Eigen::Matrix3d final_rot = rotation.cast<double>();
            ::Eigen::Quaterniond q_eigen(final_rot);
            ::tf::Quaternion q_tf;
            ::tf::quaternionEigenToTF(q_eigen, q_tf);
            ::tf::quaternionTFToMsg(q_tf, g.pregrasp_pose.pose.pose.orientation);

            tf::Quaternion rotated_around_x(tf::Vector3(1, 0, 0), 0);
            // tf::Quaternion rotated_around_x(tf::Vector3(1, 0, 0), -M_PI);
            // tf::Transform whole(q_tf*rotated_around_x, tf::Vector3(wall->values[0], wall->values[1], wall->values[2]));
            tf::Transform whole(q_tf, tf::Vector3(wall->values[0], wall->values[1], wall->values[2]));
            whole *= tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, -0.5 * (*ifco_height_), 0.0));
            ::tf::poseTFToMsg(whole, g.pregrasp_pose.pose.pose);
            g.pregrasp_pose.center = g.pregrasp_pose.pose;

            double edge_length = (i == 0) ? (*ifco_length_) : (*ifco_width_);
            g.pregrasp_pose.size.push_back(edge_length);
            g.pregrasp_pose.size.push_back(0.15);
            g.pregrasp_pose.size.push_back(0.05);
            g.pregrasp_pose.image_size.push_back(0.1);
            g.pregrasp_pose.image_size.push_back(0.4);
            g.pregrasp_pose.image_size.push_back(0.02);

            // Set object pose relative to hand
            g.object.center.pose = g.object.pose.pose = g.pregrasp_pose.center.pose;
            g.object.size.push_back(0.05);
            g.object.size.push_back(0.05);
            g.object.size.push_back(0.05);
            g.object.size.push_back(4.0);
            g.object.image_size.push_back(0.005);
            g.object.image_size.push_back(0.005);
            g.object.image_size.push_back(0.005);
            g.object.image_size.push_back(0.1);

            wall_messages->strategies.push_back(g);

            // Add the corresponding manifold
            ::posesets::PoseSet poseSet(tf::Transform(q_tf, tf::Vector3(wall->values[0], wall->values[1], wall->values[2])));
            poseSet.setPositions(tf::Vector3(edge_length, 0.01, 0.02));
            poseSet.getOrientations().addFuzzy(q_tf);
            wall_manifolds->push_back(poseSet);
        }

        (*wall_pregrasp_messages_) = wall_messages;   // delete all messages stuff here (and test!)
        (*wall_manifolds_) = wall_manifolds;


        return OK;

    }
    // ======================================================================================================================

};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::IfcoGrasps>, "IfcoGrasps", "Finds grasps for IFCO.");
