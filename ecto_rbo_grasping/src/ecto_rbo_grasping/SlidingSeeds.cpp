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
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

#include <Eigen/Geometry>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct SlidingSeeds
{
    spore< UnalignedAffine3f > transformation_;

    spore<std::vector< UnalignedVector3f > > seed_lines_1_;
    spore<std::vector< UnalignedVector3f > > seed_lines_2_;
    spore<std::vector< UnalignedVector3f > > seed_directions_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare< UnalignedAffine3f >("transformation", "Bounding Box frame.");

        outputs.declare<std::vector< UnalignedVector3f > >("seed_lines_1", "Starting points for pushing.");
        outputs.declare<std::vector< UnalignedVector3f > >("seed_lines_2", "Starting points for pushing.");
        outputs.declare<std::vector< UnalignedVector3f > >("seed_directions", "Directions for pushing.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        transformation_ = inputs["transformation"];

        seed_lines_1_ = outputs["seed_lines_1"];
        seed_lines_2_ = outputs["seed_lines_2"];
        seed_directions_ = outputs["seed_directions"];

    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        seed_lines_1_->clear();
        seed_lines_2_->clear();
        seed_directions_->clear();

        // assume bounding box and generate slides towards four faces
        UnalignedVector3f corner_1 = transformation_->translation() + 0.05 * transformation_->rotation().col(2) + 0.05 * transformation_->rotation().col(1);
        UnalignedVector3f corner_2 = transformation_->translation() + 0.05 * transformation_->rotation().col(2) - 0.05 * transformation_->rotation().col(1);
        UnalignedVector3f corner_3 = transformation_->translation() - 0.05 * transformation_->rotation().col(2) + 0.05 * transformation_->rotation().col(1);
        UnalignedVector3f corner_4 = transformation_->translation() - 0.05 * transformation_->rotation().col(2) - 0.05 * transformation_->rotation().col(1);

        seed_lines_1_->push_back(corner_1);
        seed_lines_2_->push_back(corner_2);
        seed_lines_1_->push_back(corner_3);
        seed_lines_2_->push_back(corner_4);
        seed_lines_1_->push_back(corner_1);
        seed_lines_2_->push_back(corner_3);
        seed_lines_1_->push_back(corner_2);
        seed_lines_2_->push_back(corner_4);

        seed_directions_->push_back(transformation_->rotation().col(2));
        seed_directions_->push_back(-(transformation_->rotation().col(2)));
        seed_directions_->push_back(transformation_->rotation().col(1));
        seed_directions_->push_back(-(transformation_->rotation().col(1)));

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::SlidingSeeds, "SlidingSeeds", "Finding pushing seeds.");
