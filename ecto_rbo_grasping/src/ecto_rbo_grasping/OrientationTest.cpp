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

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

#include <ecto_rbo_grasping/OrientationSet.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct OrientationTest
{
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> wall_pregrasp_messages_;

    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygons", "3D Polygons.");
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "3D Polygons.");

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("wall_pregrasp_messages", "All the grasps that should be used.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        polygons_ = inputs["polygons"];
        bounded_planes_ = inputs["bounded_planes"];

        wall_pregrasp_messages_ = outputs["wall_pregrasp_messages"];
    }

    void publishRVizMarkers(const ::posesets::OrientationSet& orientations, const UnalignedVector3f& offset)
    {
        static ros::NodeHandle nh;
        static ros::Publisher axis_publisher = nh.advertise< ::visualization_msgs::Marker>("/orientation_test", 10);
        static int id = 0;

        ::visualization_msgs::Marker msg;
        msg.id = id++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/map";
        msg.type = ::visualization_msgs::Marker::LINE_LIST;
        msg.action = ::visualization_msgs::Marker::ADD;
        msg.lifetime = ros::Duration(1.0);

        msg.scale.x = msg.scale.y = msg.scale.z = 0.03;

        size_t samples = 30;
        msg.points.resize(2 * 3 * samples);
        msg.colors.resize(2 * 3 * samples);

        for (size_t i = 0; i < samples; ++i)
        {
            UnalignedMatrix3f mat = orientations.sample();

            for (int axis = 0; axis < 3; ++axis)
            {
                std_msgs::ColorRGBA color;
                color.r = (axis == 0) ? 1.0 : 0.0;
                color.g = (axis == 1) ? 1.0 : 0.0;
                color.b = (axis == 2) ? 1.0 : 0.0;
                color.a = 1.0;

                msg.points[6 * i + axis * 2].x = 0 + offset(0);
                msg.points[6 * i + axis * 2].y = 0 + offset(1);
                msg.points[6 * i + axis * 2].z = 0 + offset(2);
                msg.colors[6 * i + axis * 2] = color;

                msg.points[6 * i + axis * 2 + 1].x = mat.col(axis)(0) + offset(0);
                msg.points[6 * i + axis * 2 + 1].y = mat.col(axis)(1) + offset(1);
                msg.points[6 * i + axis * 2 + 1].z = mat.col(axis)(2) + offset(2);
                msg.colors[6 * i + axis * 2 + 1] = color;
            }
        }

        axis_publisher.publish(msg);
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
        ROS_INFO("STARTING!");

        ::posesets::OrientationSet set_of_orientations;
        std::ofstream file("orient.dat");

        int max_iter = 50000;
        for (int i = 0; i < max_iter; ++i)
        {
            // generate random quaternion and add them to the set
            double roll = ((double) rand() / ((double) RAND_MAX + 1.0)) * M_PI * 2.0;
            double pitch = ((double) rand() / ((double) RAND_MAX + 1.0)) * M_PI * 2.0;
            double yaw = ((double) rand() / ((double) RAND_MAX + 1.0)) * M_PI * 2.0;

            tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
            set_of_orientations.add(q);

            file << i << " " << set_of_orientations.getSet().size() << std::endl;
        }

        file.close();

        ROS_INFO("DOOOONNNNNNEEEEE!!");

        return OK;
    }

//    template<typename Point>
//    int process(const tendrils& inputs, const tendrils& outputs,
//                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
//                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
//    {
//        ::posesets::OrientationSet set_of_orientations_1;
//        ::posesets::OrientationSet set_of_orientations_2;

//        tf::Quaternion q = tf::createQuaternionFromRPY(1.0, 0.6, 1.6);
//        tf::Quaternion q1 = tf::createQuaternionFromRPY(0.4, 1.3, 0.6);
//        tf::Quaternion q2 = tf::Quaternion(q1.getAxis(), 0.1);

//        for (size_t a = 0; a < 60; ++a)
//        {
//            tf::Matrix3x3 rot(q);
//            UnalignedMatrix3f mat;

//            for(int i=0; i<3; i++)
//                for(int j=0; j<3; j++)
//                    mat(i,j) = rot[i][j];

//            set_of_orientations_1.add(mat);

//            q = q * q2;
//        }

//        q = tf::createQuaternionFromRPY(1.0, 0.6, 1.6);
//        q1 = tf::createQuaternionFromRPY(1.7, 0.3, 0.1);
//        q2 = tf::Quaternion(q1.getAxis(), 0.1);

//        for (size_t a = 0; a < 60; ++a)
//        {
//            tf::Matrix3x3 rot(q);
//            UnalignedMatrix3f mat;

//            for(int i=0; i<3; i++)
//                for(int j=0; j<3; j++)
//                    mat(i,j) = rot[i][j];

//            set_of_orientations_2.add(mat);

//            q = q * q2;
//        }

//        publishRVizMarkers(set_of_orientations_1, UnalignedVector3f(0, 0, 0));
//        publishRVizMarkers(set_of_orientations_2, UnalignedVector3f(2.0, 0, 0));

//        set_of_orientations_1.intersect(set_of_orientations_2);

//        ROS_INFO("Size of intersection: %zu", set_of_orientations_1.getSet().size());
//        publishRVizMarkers(set_of_orientations_1, UnalignedVector3f(0, 2.0, 0));

//        return OK;
//    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::OrientationTest>, "OrientationTest", "Plane Segmentation using Sample Consensus.");
