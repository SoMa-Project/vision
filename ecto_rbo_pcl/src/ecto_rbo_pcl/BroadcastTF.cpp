#include <ecto_rbo_pcl/common.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
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
        static tf::TransformBroadcaster tf_broadcaster;

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
