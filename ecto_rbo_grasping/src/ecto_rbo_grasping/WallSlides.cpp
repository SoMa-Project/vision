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
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct WallSlides
{
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;
    spore< std::vector< ::pcl::PointIndices > > clusters_;

//    ecto::spore<int> max_no_of_points_;

    static void declare_params(ecto::tendrils& params)
    {
        //        params.declare<int>("max_no_of_points", "", 10);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygons", "3D Polygons.");

        outputs.declare< std::vector< ::pcl::PointIndices > >("clusters", "Clusters indices.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        polygons_ = inputs["polygons"];
        clusters_ = outputs["clusters"];
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
        clusters_->clear();

        // iterate over all edges and classify as {concave, convex, neither}
        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = polygons_->begin(); it != polygons_->end(); ++it)
        {
            size_t number_of_lines = (*it)->values.size() / 3;

            for (size_t line = 0; line < number_of_lines; ++line)
            {
                ::pcl::PointIndices cluster;
                cluster.header = input->header;

                UnalignedVector3f start_point((*it)->values[3 * line], (*it)->values[3 * line + 1], (*it)->values[3 * line + 2]);
                size_t end_point_index = (line == number_of_lines - 1) ? 0 : 3 * line + 3;
                UnalignedVector3f end_point((*it)->values[end_point_index], (*it)->values[end_point_index + 1], (*it)->values[end_point_index + 2]);
                UnalignedVector3f edge_direction = (end_point - start_point);
                float edge_length = edge_direction.norm();
                edge_direction.normalize();

                float step_length = 0.005;

                UnalignedVector3f point;
                float focal_length = 262.5;
                float focal_center_x = 159.5;
                float focal_center_y = 119.5;

                // scan over line and convert point to image coordinate using the focal length
                for (float t = 0; t < edge_length; t += step_length)
                {
                    point = start_point + t * edge_direction;

                    int image_x = (focal_length * point(0)) / point(2) + focal_center_x;
                    int image_y = (focal_length * point(1)) / point(2) + focal_center_y;

                    if (!::pcl::isFinite(input->at(image_x, image_y)))
                        continue;

//                    if ()
                    cluster.indices.push_back(image_y * input->width + image_x);
                }

                clusters_->push_back(cluster);
            }
        }

        ROS_INFO("FIINISH");

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::WallSlides>, "WallSlides", "Selecting clusters.");
