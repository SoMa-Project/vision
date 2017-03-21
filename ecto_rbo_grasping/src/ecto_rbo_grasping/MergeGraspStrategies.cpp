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

#include <Eigen/Geometry>

#include <pregrasp_msgs/GraspStrategyArray.h>

namespace ecto_rbo_grasping
{

struct MergeGraspStrategies
{
    static const int max_input_messages = 10;
    std::vector< ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> > messages_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> merged_messages_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("message_0", "Grasps that should be merged.").required(true);
        for (int i = 1; i < max_input_messages; ++i)
        {
            std::string name = "message_" + boost::lexical_cast<std::string>(i);
            inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>(name, "Grasps that should be merged.").required(false);
        }

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>(&MergeGraspStrategies::merged_messages_, "merged_messages", "Merged messages.");
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
        pregrasp_msgs::GraspStrategyArrayPtr all_messages(new ::pregrasp_msgs::GraspStrategyArray(**(messages_.front())));

        for (size_t i = 1; i < messages_.size(); ++i)
        {
            if (messages_[i].user_supplied())
                all_messages->strategies.insert(all_messages->strategies.end(), (*messages_[i])->strategies.begin(), (*messages_[i])->strategies.end());
            else
                break;
        }

        *merged_messages_ = all_messages;

        return ecto::OK;
    }
};

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct GraspStrategies2Transform
{
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> input_;
    
    ecto::spore<int> strategy_;
    ecto::spore<int> pregrasp_configuration_;
    
    ecto::spore<UnalignedAffine3f> transform_;
    ecto::spore<UnalignedAffine3f> transform2_;
    ecto::spore<bool> grasp_available_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<int>(&GraspStrategies2Transform::pregrasp_configuration_, "pregrasp_configuration", "", pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE);
        params.declare<int>(&GraspStrategies2Transform::strategy_, "strategy", "", pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>(&GraspStrategies2Transform::input_, "input", "Merged messages.");
        
        outputs.declare<UnalignedAffine3f>(&GraspStrategies2Transform::transform_, "transform", "Extracted transform.");
        outputs.declare<UnalignedAffine3f>(&GraspStrategies2Transform::transform2_, "transform2", "Extracted transform.");
        outputs.declare<bool>(&GraspStrategies2Transform::grasp_available_, "grasp_available", "Is true if a grasp of the desired type was found.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
    }

    UnalignedAffine3f pose2eigen(geometry_msgs::Pose& p)
    {
        return Eigen::Translation3f(p.position.x, p.position.y, p.position.z) * 
               Eigen::Quaternionf(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        *grasp_available_ = false;
        pregrasp_msgs::GraspStrategy g;
        
        // get desired type of grasp
        for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = (*input_)->strategies.begin(); it != (*input_)->strategies.end(); ++it)
        {
            if (it->strategy == *strategy_ && it->pregrasp_configuration == *pregrasp_configuration_)
            {
                g = *it;
                *grasp_available_ = true;
                break;
            }
        }
        
        // extract information, similar to ec_grasps.py
        if (*grasp_available_)
        {
            *transform_  = pose2eigen(g.pregrasp_pose.pose.pose);
            *transform2_ = pose2eigen(g.object.pose.pose);
        }
        
        return ecto::OK;
    }
};


}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::MergeGraspStrategies, "MergeGraspStrategies", "Merge four GraspStrategyArrays into one.")
ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::GraspStrategies2Transform, "GraspStrategies2Transform", "Extract a transform from a vector of grasp strategies.")
