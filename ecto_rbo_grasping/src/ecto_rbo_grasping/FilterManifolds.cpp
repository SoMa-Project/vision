#include <algorithm>
#include <functional>

#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ecto_rbo_grasping/PoseSet.h>

namespace ecto_rbo_grasping
{

struct FilterManifolds
{
  ecto::spore< ::posesets::PoseSetArrayConstPtr> manifolds_;
  ecto::spore< ::posesets::PoseSetArrayConstPtr> filtered_manifolds_;

  ecto::spore<int> index_;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<int>("index", "", 0);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    inputs.declare< ::posesets::PoseSetArrayConstPtr>("manifolds", "All the manifolds that should be published as markers.");
    outputs.declare< ::posesets::PoseSetArrayConstPtr>("filtered_manifolds", "All the manifolds that should be published as markers.");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    manifolds_ = inputs["manifolds"];
    filtered_manifolds_ = outputs["filtered_manifolds"];

    index_ = params["index"];
  }

  int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
  {
    ::posesets::PoseSetArrayPtr filtered_manifolds(new ::posesets::PoseSetArray());

    if ((*manifolds_)->size() > *index_)
    {
        filtered_manifolds->push_back((*manifolds_)->at(*index_));
    }

    (*filtered_manifolds_) = filtered_manifolds;

    return ecto::OK;
  }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::FilterManifolds, "FilterManifolds", "Filter according to various criterias.")
