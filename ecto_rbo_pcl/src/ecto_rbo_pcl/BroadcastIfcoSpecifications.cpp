/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_rbo_pcl/common.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>




using namespace ecto;

namespace ecto_rbo_pcl
{

typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;

struct BroadcastIfcoSpecifications
{

    ros::NodeHandle nh_;

    // params
    spore<double> ifco_length_;
    spore<double> ifco_width_;
    spore<double> ifco_height_;
    spore< std::vector<double> > ec_wall_offset_;


    // outputs
    spore<double> ifco_length__;
    spore<double> ifco_width__;
    spore<double> ifco_height__;
    spore< std::vector<double> > ec_wall_offset__;




    static void declare_params(tendrils& params)
    {

        std::vector<double> ec_wall_offset_default;
        ec_wall_offset_default.push_back(0.0);

        params.declare<double>("ifco_length", "Size of the long IFCO edge", 0.0);
        params.declare<double>("ifco_width", "Size of the short IFCO edge", 0.0);
        params.declare<double>("ifco_height", "Depth of the ifco", 0.0);
        params.declare< std::vector<double> >("ec_wall_offset", "The space that is occupied by the ec.", ec_wall_offset_default);

    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        outputs.declare<double>("ifco_length", "Size of the long IFCO edge", 0.0);
        outputs.declare<double>("ifco_width", "Size of the short IFCO edge", 0.0);
        outputs.declare<double>("ifco_height", "Depth of the ifco", 0.0);
        outputs.declare<std::vector<double> >("ec_wall_offset", "The space that is occupied by the ec.");

    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        // params
        ifco_length_            = params["ifco_length"];
        ifco_width_             = params["ifco_width"];
        ifco_height_            = params["ifco_height"];
        ec_wall_offset_         = params["ec_wall_offset"];

        // outputs
        ifco_length__           = outputs["ifco_length"];
        ifco_width__            = outputs["ifco_width"];
        ifco_height__           = outputs["ifco_height"];
        ec_wall_offset__        = outputs["ec_wall_offset"];

    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        (*ifco_length__) = *ifco_length_;
        (*ifco_width__)  = *ifco_width_;
        (*ifco_height__) = *ifco_height_;

        (*ec_wall_offset__) = *ec_wall_offset_;

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::BroadcastIfcoSpecifications, "BroadcastIfcoSpecifications", "Broadcast ifco specifics.");
