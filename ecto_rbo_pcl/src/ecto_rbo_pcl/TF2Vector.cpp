/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct TF2Vector
{
    spore<UnalignedAffine3f> transform_;
    spore<UnalignedVector3f> vector_;
    spore<int> column_index_;
    spore<bool> negate_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<int>(&TF2Vector::column_index_, "column_index", "Which column vector to extract.", 0);
        params.declare<bool>(&TF2Vector::negate_, "negate", "Should the resulting vector be negated.", false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<UnalignedAffine3f>(&TF2Vector::transform_, "transform", "A 4x4 affine transformation.");
        outputs.declare<UnalignedVector3f>(&TF2Vector::vector_, "vector", "The selected column vector.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        *(vector_) = (*transform_).linear().col(*column_index_) * ((*negate_) ? -1.0 : 1.0);
        return OK;
    }
};

struct TFs2Vectors
{
    spore< std::vector<UnalignedAffine3f> > transforms_;
    spore< std::vector<UnalignedVector3f> > vectors_;
    spore<int> column_index_;
    spore<bool> negate_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<int>(&TFs2Vectors::column_index_, "column_index", "Which column vector to extract.", 0);
        params.declare<bool>(&TFs2Vectors::negate_, "negate", "Should the resulting vector be negated.", false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare< std::vector<UnalignedAffine3f> >(&TFs2Vectors::transforms_, "transforms", "A 4x4 affine transformation.");
        outputs.declare< std::vector<UnalignedVector3f> >(&TFs2Vectors::vectors_, "vectors", "The selected column vector.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        vectors_->clear();
        
        for (size_t i = 0; i < transforms_->size(); ++i)
            vectors_->push_back((*transforms_)[i].linear().col(*column_index_) * ((*negate_) ? -1.0 : 1.0));
        
        return OK;
    }
};


}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::TF2Vector, "TF2Vector", "Extracts a specified column vector from an affine 4x4 matrix.");
ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::TFs2Vectors, "TFs2Vectors", "Extracts a specified column vector from an affine 4x4 matrix.");
