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
