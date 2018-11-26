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
#undef BOOST_PARAMETER_MAX_ARITY
#define BOOST_PARAMETER_MAX_ARITY 7
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#if PCL_VERSION_COMPARE(>=,1,7,0)
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif

namespace ecto_rbo_pcl
{

typedef sensor_msgs::PointCloud2::ConstPtr MsgT;


template<typename PointT>
typename ::pcl::PointCloud<PointT>::ConstPtr
convert(MsgT msg)
{
    typedef typename ::pcl::PointCloud<PointT> CloudT;
    typedef typename CloudT::Ptr PtrT;
    PtrT p(new CloudT);
#if PCL_VERSION_COMPARE(<,1,7,0)
    ::pcl::fromROSMsg(*msg, *p);
#else
    ::pcl::PCLPointCloud2 pcd_tmp;
    pcl_conversions::toPCL(*msg, pcd_tmp);
    ::pcl::fromPCLPointCloud2(pcd_tmp, *p);
#endif
    return p;
}
/* dispatch to handle process */
struct to_message: boost::static_visitor<MsgT>
{
    template<typename CloudType>
    MsgT
    operator()(CloudType& i) const
    {
        sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
#if PCL_VERSION_COMPARE(<,1,7,0)
        ::pcl::toROSMsg(*i, *msg);
#else
        ::pcl::PCLPointCloud2 pcd_tmp;
        ::pcl::toPCLPointCloud2(*i, pcd_tmp);
        pcl_conversions::fromPCL(pcd_tmp, *msg);
#endif
        return msg;
    }
};

struct Message2PointCloud
{
    static void declare_params(tendrils& params) {
        params.declare<int>("format", "Format of cloud to grab. Choices are: XYZ, XYZRGB (default)",
                            ecto::pcl::FORMAT_XYZRGB);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs) {
        inputs.declare<MsgT>("input", "An ROS point cloud message.");
        outputs.declare<ecto::pcl::PointCloud>("output", "An XYZ/XYZRGB point cloud from the kinect");
        outputs.declare<std_msgs::Header>("header", "The header of the ros message.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs) {
        format_ = params["format"];
        input_ = inputs["input"];
        output_ = outputs["output"];
        header_ = outputs["header"];

        rgbd_snapshot_pub = n.advertise<sensor_msgs::PointCloud2>("rgbd_snapshot", 2, true); // latch=true
    }

    void publishPointCloudSnapshot() {
        rgbd_snapshot_pub.publish(*input_);
    }

    int process(const tendrils& /*inputs*/, const tendrils& outputs) {
        (*header_) = (*input_)->header;
        switch (*format_) {
        case ecto::pcl::FORMAT_XYZ:
            *output_ = convert< ::pcl::PointXYZ>(*input_);
            break;
        case ecto::pcl::FORMAT_XYZRGB:
            *output_ = convert< ::pcl::PointXYZRGB>(*input_);
            break;
        case ecto::pcl::FORMAT_XYZI:
            *output_ = convert< ::pcl::PointXYZI>(*input_);
            break;
        case ecto::pcl::FORMAT_XYZRGBA:
            *output_ = convert< ::pcl::PointXYZRGBA>(*input_);
            break;
        default:
            throw std::runtime_error("Unsupported point cloud type.");
        }

        // publish uses point cloud
        publishPointCloudSnapshot();

        return ecto::OK;
    }
    ecto::spore<int> format_;
    ecto::spore<MsgT> input_;
    ecto::spore<ecto::pcl::PointCloud> output_;
    ecto::spore<std_msgs::Header> header_;

    // required for snapshot publisher
    ros::NodeHandle n;
    ros::Publisher rgbd_snapshot_pub;
};

struct PointCloud2Message
{
    static void declare_params(tendrils& params)
    {
        params.declare<int>("format", "Format of cloud to grab. Choices are: XYZ, XYZRGB (default)",
                            ecto::pcl::FORMAT_XYZRGB);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<ecto::pcl::PointCloud>("input", "An ROS point cloud message.");
        outputs.declare<MsgT>("output", "An XYZ/XYZRGB point cloud from the kinect");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        format_ = params["format"];
        input_ = inputs["input"];
        output_ = outputs["output"];
    }

    int process(const tendrils& /*inputs*/, const tendrils& outputs)
    {
        ecto::pcl::xyz_cloud_variant_t v = input_->make_variant();
        *output_ = boost::apply_visitor(to_message(), v);
        return ecto::OK;
    }
    ecto::spore<int> format_;
    ecto::spore<ecto::pcl::PointCloud> input_;
    ecto::spore<MsgT> output_;
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::Message2PointCloud, "Message2PointCloud", "Take a PointCloud Message and converts to pcl type.");
ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::PointCloud2Message, "PointCloud2Message", "Take a pcl type and converts to PointCloud Message.");
