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

struct BBox2GSig
{
    spore<std::vector<UnalignedAffine3f> > transforms_;
    spore<std::vector<UnalignedAffine3f> > transforms_out_;
    spore<std::vector<UnalignedVector4f> > centroids_;
    spore<std::vector<UnalignedVector4f> > gs_positions_;


    static void declare_params(ecto::tendrils& params)
    {
        //params.declare<int>(&TF2Vector::column_index_, "column_index", "Which column vector to extract.", 0);
        //params.declare<bool>(&TF2Vector::negate_, "negate", "Should the resulting vector be negated.", false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector<UnalignedAffine3f> >(&BBox2GSig::transforms_, "transforms", "A 4x4 affine transformation.");
        inputs.declare<std::vector<UnalignedVector4f> >(&BBox2GSig::centroids_ , "centroids", "Clusters centroids");

        outputs.declare<std::vector<UnalignedAffine3f> >(&BBox2GSig::transforms_out_, "transforms_out", "Grasp Signature Orientation.");
        outputs.declare<std::vector<UnalignedVector4f> >(&BBox2GSig::gs_positions_, "gs_positions", "Grasp Signature Position.");

    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        Eigen::Affine3f GSr(Eigen::AngleAxisf(-0.2,Eigen::Vector3f(0,0,1)));
        Eigen::Affine3f GSt(Eigen::Translation3f(Eigen::Vector3f(0.05,0,0.2)));
        transforms_out_->clear();
        for (int i=0;i<(*transforms_).size();i++){
            (*transforms_out_).push_back((*transforms_)[i]*GSr);

            //(*transforms_out_)*Eigen::Affine3f(Eigen::Translation3f((*gs_positions_)[i]);


        }
        //Eigen::Affine3f(Eigen::Translation3f(Eigen::Vector3f((*centroids_)[0].x(),(*centroids_)[0].y(),(*centroids_)[0].z())));

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::BBox2GSig, "BBox2GSig", "Creates Grasp Signature from object 4x4 matrix.");
