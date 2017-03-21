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
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Header.h>

using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

    struct PCDReader
    {
      static void declare_params(tendrils& params)
      {
        params.declare<int>("format", "Format of cloud found in PCD file.", 3);
        params.declare<std::string> ("filename", "Name of the pcd file", "");
        params.declare<std::string> ("header_frame_id", "Name of the frame id", "/camera_rgb_optical_frame");
      }

      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<PointCloud>("output", "A point cloud from the pcd file.");
        outputs.declare<std_msgs::Header>("header", "The header of the ros message.");
      }

      PCDReader() { first = true; }
      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        output_ = outputs["output"];
        format_ = params["format"];
        filename_ = params["filename"];
        header_frame_id_ = params["header_frame_id"];
        header_ = outputs["header"];
      }

      int process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        if (!first)
        {
//          header_->stamp = ros::Time::now().toNSec() / 1e3;
          return OK;
        }

        first = false;
        ecto::pcl::Format format = static_cast<ecto::pcl::Format>(*format_);
        switch(format)
        {
          case FORMAT_XYZ:
            {
              std::cout << "opening " << *filename_ << std::endl;
              ::pcl::PointCloud< ::pcl::PointXYZ >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZ >);
              if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZ >(*filename_, *cloud) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZ cloud.");
                return 1;
              }
              if (cloud->header.frame_id.empty())
              {
                ros::Time::init();
                cloud->header.frame_id = *header_frame_id_;
                cloud->header.stamp = ros::Time::now().toNSec() / 1e3; // see pcl_conversions::toPCL
              }
              PointCloud p( cloud );
              *output_ = p;
              (*header_) = pcl_conversions::fromPCL(cloud->header);
            } break;
          case FORMAT_XYZRGB:
            {
              ::pcl::PointCloud< ::pcl::PointXYZRGB >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGB >);
              if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZRGB > (*filename_, *cloud) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZRGB cloud.");
                return 1;
              }
              if (cloud->header.frame_id.empty())
              {
                ros::Time::init();
                cloud->header.frame_id = *header_frame_id_;
                cloud->header.stamp = ros::Time::now().toNSec() / 1e3; // see pcl_conversions::toPCL
              }
              PointCloud p( cloud );
              *output_ = p;
              (*header_) = pcl_conversions::fromPCL(cloud->header);
            } break;
          case FORMAT_XYZRGBA:
            {
              ::pcl::PointCloud< ::pcl::PointXYZRGBA >::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGBA >);
              if ( ::pcl::io::loadPCDFile< ::pcl::PointXYZRGBA > (*filename_, *cloud) == -1)
              {
                throw std::runtime_error("PCDReader: failed to read PointXYZRGBA cloud.");
                return 1;
              }
              if (cloud->header.frame_id.empty())
              {
                ros::Time::init();
                cloud->header.frame_id = *header_frame_id_;
                cloud->header.stamp = ros::Time::now().toNSec() / 1e3; // see pcl_conversions::toPCL
              }
              PointCloud p( cloud );
              *output_ = p;
              (*header_) = pcl_conversions::fromPCL(cloud->header);
            } break;
          default:
            throw std::runtime_error("PCDReader: Unknown cloud type.");
        }
        return OK;
      }

      bool first;

      spore<PointCloud> output_;
      spore<int> format_;
      spore<std::string> filename_;
      spore<std::string> header_frame_id_;
      spore<std_msgs::Header> header_;
    };

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::PCDReader, "PCDReader", "Read a cloud from a PCD file");
