#include <ecto/ecto.hpp>
#include <ecto_ros/wrap_sub.hpp>
#include <ros/ros.h>
#include <pregrasp_msgs/GraspStrategyArray.h>

using namespace pregrasp_msgs;

namespace ecto_rbo_grasping
{

  struct GraspStrategyArraySubscriber :
    public ecto_ros::Subscriber<GraspStrategyArray>
    {
      // inherit everything from Subscribe template
    };

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::GraspStrategyArraySubscriber,
          "GraspStrategyArraySubscriber", "Subscribe to grasp strategy array");
