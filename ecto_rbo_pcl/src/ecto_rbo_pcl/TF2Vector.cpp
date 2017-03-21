/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_rbo_pcl/common.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

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

struct TFs2FourVectors
{
    spore< std::vector<UnalignedAffine3f> > transforms_;
    spore< std::vector<UnalignedVector3f> > columns_1_;
    spore< std::vector<UnalignedVector3f> > columns_2_;
    spore< std::vector<UnalignedVector3f> > columns_3_;
    spore< std::vector<UnalignedVector4f> > columns_4_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare< std::vector<UnalignedAffine3f> >(&TFs2FourVectors::transforms_, "transforms", "A 4x4 affine transformation.");
        outputs.declare< std::vector<UnalignedVector3f> >(&TFs2FourVectors::columns_1_, "columns_1", "The selected column vector.");
        outputs.declare< std::vector<UnalignedVector3f> >(&TFs2FourVectors::columns_2_, "columns_2", "The selected column vector.");
        outputs.declare< std::vector<UnalignedVector3f> >(&TFs2FourVectors::columns_3_, "columns_3", "The selected column vector.");
        outputs.declare< std::vector<UnalignedVector4f> >(&TFs2FourVectors::columns_4_, "columns_4", "The selected column vector.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        columns_1_->clear();
        columns_2_->clear();
        columns_3_->clear();
        columns_4_->clear();
        
        for (size_t i = 0; i < transforms_->size(); ++i)
        {
            columns_1_->push_back((*transforms_)[i].linear().col(0));
            columns_2_->push_back((*transforms_)[i].linear().col(1));
            columns_3_->push_back((*transforms_)[i].linear().col(2));
            // TODO: columns_4_->push_back((*transforms_)[i].col(3));
        }
        
        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::TF2Vector, "TF2Vector", "Extracts a specified column vector from an affine 4x4 matrix.");
ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::TFs2Vectors, "TFs2Vectors", "Extracts a specified column vector from an affine 4x4 matrix.");
