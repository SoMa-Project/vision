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
