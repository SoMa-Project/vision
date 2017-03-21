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
