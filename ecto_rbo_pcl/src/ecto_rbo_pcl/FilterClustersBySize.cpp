/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ros/console.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

#include <ros/ros.h>

using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct FilterClustersBySize
{
    spore<Clusters> clusters_;
    spore<Clusters> filtered_clusters_;

    spore<int> filter_criterion_;
    spore<double> min_volume_, max_volume_;
    spore<int> max_size_, min_size_;
    spore<double> max_distance_;
    spore<UnalignedAffine3f> transform_;

    static void declare_params(tendrils& params) {
        params.declare<int>(&FilterClustersBySize::filter_criterion_, "filter_criterion", "Whether to cluster based on volume or number of points.", 0);
        params.declare<double>(&FilterClustersBySize::min_volume_, "min_volume", "Minimum size of a cluster in m^3.", 0.02 * 0.02 * 0.02).required(false);
        params.declare<double>(&FilterClustersBySize::max_volume_, "max_volume", "Maximum size of a cluster in m^3.", 0.2 * 0.2 * 0.01).required(false);
        params.declare<int>(&FilterClustersBySize::min_size_, "min_size", "Minimum size of a cluster in number of points.", 100).required(false);
        params.declare<int>(&FilterClustersBySize::max_size_, "max_size", "Minimum size of a cluster in number of points.", 100000).required(false);
        params.declare<double>(&FilterClustersBySize::max_distance_, "max_distance", "Maximum distance of a cluster to a specified point.", 0.2).required(false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
        inputs.declare<Clusters>(&FilterClustersBySize::clusters_, "clusters", "Cluster indices.");
        inputs.declare<UnalignedAffine3f>(&FilterClustersBySize::transform_, "transform", "Pose.");
        outputs.declare<Clusters>(&FilterClustersBySize::filtered_clusters_, "filtered_clusters", "Filtered clusters.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input) {

        ROS_DEBUG("FilterClustersBySize.cpp: No. of clusters to choose from: %zu", clusters_->size());
        if (clusters_->empty()) {
            ROS_WARN("FilterClustersBySize.cpp: No clusters to choose from!");
            return ecto::OK;
        }

        filtered_clusters_->clear();

        switch (*filter_criterion_)
        {
        case 0:
        {
            for (ecto::pcl::Clusters::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
            {
                Eigen::Vector4f min_pt;
                Eigen::Vector4f max_pt;
                ::pcl::getMinMax3D(*input, it->indices, min_pt, max_pt);

                double volume = (max_pt(0) - min_pt(0)) * (max_pt(1) - min_pt(1)) * (max_pt(2) - min_pt(2));
                //ROS_INFO_STREAM("Cluster volume: " << volume);

                if (volume > *min_volume_ && volume < *max_volume_)
                {
                    filtered_clusters_->push_back(*it);
                }
            }
        }
            break;
        case 1:
        {
            for (ecto::pcl::Clusters::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
            {
                //ROS_INFO_STREAM("Cluster points: " << it->indices.size());

                if (it->indices.size() > *min_size_ && it->indices.size() < *max_size_)
                {
                    filtered_clusters_->push_back(*it);
                }
            }
        }
            break;
        case 2:
        {
            for (ecto::pcl::Clusters::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
            {
                Eigen::Vector4f mean;
                ::pcl::compute3DCentroid(*input, *it, mean);

                double distance = (mean.head<3>() - transform_->translation()).norm();
                //ROS_INFO_STREAM("Distance: " << distance);

                if (distance <= *max_distance_)
                {
                    filtered_clusters_->push_back(*it);
                }
            }
        }
            break;
        }

        ROS_INFO("FilterClustersBySize.cpp: No. of clusters filtered: %zu of %zu", filtered_clusters_->size(), clusters_->size());

        return OK;
    }
};

struct FilterClustersByNormals
{
    spore<Clusters> clusters_;
    spore<Clusters> filtered_clusters_;

    spore<double> max_distance_;
    spore<bool> negate_;
    spore<UnalignedAffine3f> transform_;

    static void declare_params(tendrils& params) {
        params.declare<double>(&FilterClustersByNormals::max_distance_, "max_distance", "Maximum distance of a cluster normal to the input normal in degrees.", 20.0);
        params.declare<bool>(&FilterClustersByNormals::negate_, "negate", "Maximum distance of a cluster normal to the input normal in degrees.", true);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
        inputs.declare<Clusters>(&FilterClustersByNormals::clusters_, "clusters", "Cluster indices.");
        inputs.declare<UnalignedAffine3f>(&FilterClustersByNormals::transform_, "transform", "Pose.");
        outputs.declare<Clusters>(&FilterClustersByNormals::filtered_clusters_, "filtered_clusters", "Filtered clusters.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
        clusters_ = inputs["clusters"];
        transform_ = inputs["transform"];
        filtered_clusters_ = outputs["filtered_clusters"];

        max_distance_ = params["max_distance"];
        negate_ = params["negate"];
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& input_normals)
    {
        ROS_DEBUG("FilterClustersByNormals.cpp: No. of clusters to choose from: %zu", clusters_->size());
        if (clusters_->empty()) {
            ROS_WARN("FilterClustersByNormals.cpp: No clusters to choose from!");
            return ecto::OK;
        }

        filtered_clusters_->clear();

        const ::pcl::PointCloud< ::pcl::Normal> normals = *input_normals;
        // we only consider 0 - 90deg
        double max_distance = 1.0 - fabs(::std::cos(::pcl::deg2rad(*max_distance_)));

        for (ecto::pcl::Clusters::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
        {
            Eigen::Vector3f mean_normal;

            // calculate mean normal
            for (std::vector<int>::iterator jt = it->indices.begin(); jt != it->indices.end(); ++jt) {
              mean_normal[0] += normals[*jt].normal_x;
              mean_normal[1] += normals[*jt].normal_y;
              mean_normal[2] += normals[*jt].normal_z;
            }
            mean_normal.normalize();

            //std::cerr << "First normal: " << mean_normal(0) << " " << mean_normal(1) << " " << mean_normal(2) << std::endl;

            double distance = 1.0 - fabs(mean_normal.dot(transform_->linear().col(2)));

            //ROS_INFO_STREAM("Angular distance: " << distance << "   max = " << max_distance);

            if (*negate_)
            {
                if (distance > max_distance)
                {
                    filtered_clusters_->push_back(*it);
                }
            }
            else {
                if (distance <= max_distance)
                {
                    filtered_clusters_->push_back(*it);
                }
            }
        }

        ROS_INFO("FilterClustersByNormals.cpp: No. of clusters filtered: %zu of %zu", filtered_clusters_->size(), clusters_->size());

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::FilterClustersBySize>, "FilterClustersBySize", "Filter clusters according to their size.");
ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCellWithNormals<ecto_rbo_pcl::FilterClustersByNormals>, "FilterClustersByNormals", "Filter clusters according to their mean surface normal.");
