#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ecto_rbo_grasping/PoseSet.h>
#include <pregrasp_msgs/GraspStrategyArray.h>

namespace ecto_rbo_grasping
{

struct MergeManifolds
{
    ros::NodeHandle nh_;

    static const int max_input_messages = 10;
    std::vector< ecto::spore< posesets::PoseSetArrayConstPtr> > messages_;

    ecto::spore< posesets::PoseSetArrayConstPtr> merged_messages_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<posesets::PoseSetArrayConstPtr>("message_0", "Grasps that should be merged.").required(true);
        for (int i = 1; i < max_input_messages; ++i)
        {
            std::string name = "message_" + boost::lexical_cast<std::string>(i);
            inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>(name, "Grasps that should be merged.").required(false);
        }

        outputs.declare<posesets::PoseSetArrayConstPtr>(&MergeManifolds::merged_messages_, "merged_messages", "Merged messages.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        messages_.resize(max_input_messages);
        for (int i = 0; i < max_input_messages; ++i)
        {
            std::string name = "message_" + boost::lexical_cast<std::string>(i);
            messages_[i] = inputs[name];
        }
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        posesets::PoseSetArrayPtr all_messages(new posesets::PoseSetArray(**(messages_.front())));

        for (size_t i = 1; i < messages_.size(); ++i)
        {
            if (messages_[i].user_supplied())
                all_messages->insert(all_messages->end(), (*messages_[i])->begin(), (*messages_[i])->end());
            else
                break;
        }

        *merged_messages_ = all_messages;

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::MergeManifolds, "MergeManifolds", "Merge PoseSetArrays into one.")
