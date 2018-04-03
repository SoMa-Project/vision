/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct SegmentObjects
{

    ecto::spore<geometry_msgs::PoseArray::ConstPtr> pose_array_;
    ecto::spore<std_msgs::Float32MultiArray::ConstPtr> bounding_box_array_;

    ecto::spore<std::vector<UnalignedAffine3f> > object_poses_;
    ecto::spore<std::vector<UnalignedVector3f> > object_sizes_;
    ecto::spore<std::vector<UnalignedVector4f> > centroids_;

    static void declare_params(ecto::tendrils& params)
    {

    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<geometry_msgs::PoseArray::ConstPtr>("pose_array", "A PoseArray msg with the poses of the objects with respect to the camera");
        inputs.declare<std_msgs::Float32MultiArray::ConstPtr>("bounding_box_array", "An array with the dimensions of the bounding boxes expressed in each object's frame");
        outputs.declare<std::vector<UnalignedAffine3f> >(&SegmentObjects::object_poses_, "transforms", "A vector of 4x4 affine transformations for the objects.");
        outputs.declare<std::vector<UnalignedVector3f> >(&SegmentObjects::object_sizes_, "sizes", "A vector of 3d sizes for the bounding boxes.");
        outputs.declare<std::vector<UnalignedVector4f> >(&SegmentObjects::centroids_, "centroids", "A vector of the centroids of the objects.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        pose_array_ = inputs["pose_array"];
        bounding_box_array_ = inputs["bounding_box_array"];
        object_poses_ = outputs["transforms"];
        object_sizes_ = outputs["sizes"];
        centroids_ = outputs["centroids"];
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {

        object_poses_->clear();
        centroids_->clear();
        object_sizes_->clear();

        for(int i=0; i<(*pose_array_)->poses.size(); i++)
        {   
            Eigen::Affine3d object_pose;
            tf::poseMsgToEigen((*pose_array_)->poses[i], object_pose);
            object_poses_->push_back((UnalignedAffine3f)object_pose.cast<float>());

            UnalignedVector4f centroid;
            centroid << object_pose.translation()[0], object_pose.translation()[1], object_pose.translation()[2], 0;
            centroids_->push_back(centroid);

            UnalignedVector3f bounding_box;
            bounding_box << (*bounding_box_array_)->data[3*i], (*bounding_box_array_)->data[3*i + 1], (*bounding_box_array_)->data[3*i + 2];
            object_sizes_->push_back(bounding_box);
        }
        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::SegmentObjects, "SegmentObjects", "Output a list of objects found using a third-party segmentation algorithm.")
