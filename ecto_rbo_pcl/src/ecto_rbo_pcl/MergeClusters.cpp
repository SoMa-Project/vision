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
