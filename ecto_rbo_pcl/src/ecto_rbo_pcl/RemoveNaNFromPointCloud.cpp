/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/filters/filter.h>

namespace ecto_rbo_pcl {

struct RemoveNaNFromPointCloud
{
    ecto::spore<ecto::pcl::PointCloud> output_;
    ecto::spore< ::pcl::PointIndices> indices_;
    ecto::spore<bool> construct_pointcloud_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<bool>("construct_pointcloud", "Construct the pointcloud and provid it as output or just the indices.", true);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs) {
        outputs.declare<ecto::pcl::PointCloud>("output", "Filtered Cloud.");
        outputs.declare< ::pcl::PointIndices>("indices", "Filtered indices.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        output_ = outputs["output"];
        indices_ = outputs["indices"];
        construct_pointcloud_ = params["construct_pointcloud"];
    }

    template <typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
        typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        cloud->header = input->header;
        indices_->header = input->header;
        if (*construct_pointcloud_)
            ::pcl::removeNaNFromPointCloud(*input, *cloud, indices_->indices);
//        Seems like this is not in the current PCL version? (although docs say so)
//        else
//            ::pcl::removeNaNFromPointCloud(*input, indices_->indices);


        *output_ = ecto::pcl::xyz_cloud_variant_t(cloud);

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::RemoveNaNFromPointCloud>,
          "RemoveNaNFromPointCloud", "Remove all NaNs from the point cloud.");



