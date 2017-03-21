/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <algorithm>
#include <functional>

#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

namespace ecto_rbo_grasping
{

struct FilterPreGrasps
{
  ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;
  ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> filtered_messages_;

  ecto::spore<double> min_quality_grasp_;
  ecto::spore<double> min_quality_closing_;
  ecto::spore<double> min_quality_approach_;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<double>("min_quality_grasp", "", 0.0);
    params.declare<double>("min_quality_closing", "", 0.0);
    params.declare<double>("min_quality_approach", "", 0.0);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "All the grasps that should be filtered.");
    outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("filtered_messages", "");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    pregrasp_messages_ = inputs["pregrasp_messages"];
    filtered_messages_ = outputs["filtered_messages"];

    min_quality_approach_ = params["min_quality_approach"];
    min_quality_closing_ = params["min_quality_closing"];
    min_quality_grasp_ = params["min_quality_grasp"];
  }

  int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
  {
    pregrasp_msgs::GraspStrategyArrayPtr filtered_messages(new ::pregrasp_msgs::GraspStrategyArray());
    filtered_messages->header = (*pregrasp_messages_)->header;

    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = (*pregrasp_messages_)->strategies.begin(); it != (*pregrasp_messages_)->strategies.end(); ++it)
    {
       if (it->quality_grasp >= *min_quality_grasp_ &&
           it->quality_closing >= *min_quality_closing_ &&
           it->quality_approach >= *min_quality_approach_)
       {
         filtered_messages->strategies.push_back(*it);
       }
    }

    (*filtered_messages_) = filtered_messages;

    return ecto::OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::FilterPreGrasps, "FilterPreGrasps", "Filter according to various criterias.")
