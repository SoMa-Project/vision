/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_rbo_pcl/common.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>

#include <std_msgs/Header.h>

using namespace ecto;

namespace ecto_rbo_pcl
{

struct BroadcastTF
{
    ros::NodeHandle nh_;
    tf::Transform tf_listener_;

    spore< UnalignedAffine3f> transform_;
    spore< std_msgs::Header> timestamp_;

    spore< std::string> source_frame_, target_frame_;

    static void declare_params(tendrils& params)
    {
        params.declare<std::string>(&BroadcastTF::source_frame_, "source_frame", "The source frame to listen to.").required(true);
        params.declare<std::string>(&BroadcastTF::target_frame_, "target_frame", "The target frame to listen to.").required(true);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<UnalignedAffine3f>(&BroadcastTF::transform_, "transform", "The transform.");
        inputs.declare<std_msgs::Header>(&BroadcastTF::timestamp_, "timestamp", "The timestamp for looking up the transform (not used).").required(false);
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {

        static tf2_ros::TransformBroadcaster tf_broadcaster;

        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = *source_frame_;
        msg.child_frame_id = *target_frame_;
        tf::transformEigenToMsg((Eigen::Affine3d) transform_->cast<double>(), msg.transform);
        tf_broadcaster.sendTransform(msg);

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::BroadcastTF, "BroadcastTF", "Broadcast a specified transform to ROS.");
