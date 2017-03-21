/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <vector>

namespace ecto_rbo_pcl
{

struct MergeClusters
{
    static const int max_input_messages = 10;

    std::vector< ecto::spore< ecto::pcl::Clusters > > clusters_;
    ecto::spore< ecto::pcl::Clusters > merged_clusters_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<ecto::pcl::Clusters>("cluster_0", "Cluster that should be part of the merger.").required(true);
        for (int i = 1; i < max_input_messages; ++i)
        {
            std::string name = "cluster_" + boost::lexical_cast<std::string>(i);
            inputs.declare<ecto::pcl::Clusters>(name, "Cluster that should be part of the merger.").required(false);
        }

        outputs.declare<ecto::pcl::Clusters>(&MergeClusters::merged_clusters_, "merged_clusters", "Merged clusters.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        clusters_.resize(max_input_messages);
        for (int i = 0; i < max_input_messages; ++i)
        {
            std::string name = "cluster_" + boost::lexical_cast<std::string>(i);
            clusters_[i] = inputs[name];
        }
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        merged_clusters_->clear();

        for (size_t i = 0; i < clusters_.size(); ++i)
        {
            if (clusters_[i].user_supplied())
                merged_clusters_->insert(merged_clusters_->end(), clusters_[i]->begin(), clusters_[i]->end());
            else
                break;
        }

        ROS_INFO("After merging: %zu clusters", merged_clusters_->size());

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::MergeClusters, "MergeClusters", "Merge multiple point cloud clusters into one.")
