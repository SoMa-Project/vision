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

struct ManifoldsAtEdges
{
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > convex_edge_manifolds_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > concave_edge_manifolds_;

    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;

    ecto::spore<int> max_no_of_points_;

    static void declare_params(ecto::tendrils& params)
    {
        //        params.declare<int>("max_no_of_points", "", 10);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygons", "3D Polygons.");
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "3D Polygons.");

        outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("convex_edge_manifolds", "Edges.");
        outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("concave_edge_manifolds", "Edges.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        polygons_ = inputs["polygons"];
        bounded_planes_ = inputs["bounded_planes"];

        convex_edge_manifolds_ = outputs["convex_edge_manifolds"];
        concave_edge_manifolds_ = outputs["concave_edge_manifolds"];
    }
    void publishRVizMarkers(const std::string& frame_id)
    {
        static ros::NodeHandle nh;
        static ros::Publisher edge_marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/edges", 1);
        visualization_msgs::MarkerArray msgs;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time();
        marker.lifetime = ros::Duration(10.0);
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = 0.03;
        marker.color.r = marker.color.a = 1.0;
        marker.color.g = marker.color.b = 0.0;
        marker.points.resize(2 * concave_edge_manifolds_->size());
        marker.ns = "concave";
        marker.id = 0;

        size_t cnt = 0;

        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = concave_edge_manifolds_->begin(); it != concave_edge_manifolds_->end(); ++it, ++cnt)
        {
            marker.points[cnt * 2].x = (*it)->values[0];
            marker.points[cnt * 2].y = (*it)->values[1];
            marker.points[cnt * 2].z = (*it)->values[2];
            marker.points[cnt * 2 + 1].x = (*it)->values[3];
            marker.points[cnt * 2 + 1].y = (*it)->values[4];
            marker.points[cnt * 2 + 1].z = (*it)->values[5];
        }

        msgs.markers.push_back(marker);

        marker.ns = "convex";
        marker.id = 1;
        marker.points.resize(2 * convex_edge_manifolds_->size());

        cnt = 0;

        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = convex_edge_manifolds_->begin(); it != convex_edge_manifolds_->end(); ++it, ++cnt)
        {
            marker.points[cnt * 2].x = (*it)->values[0];
            marker.points[cnt * 2].y = (*it)->values[1];
            marker.points[cnt * 2].z = (*it)->values[2];
            marker.points[cnt * 2 + 1].x = (*it)->values[3];
            marker.points[cnt * 2 + 1].y = (*it)->values[4];
            marker.points[cnt * 2 + 1].z = (*it)->values[5];
        }

        edge_marker_publisher.publish(msgs);
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
        convex_edge_manifolds_->clear();
        concave_edge_manifolds_->clear();

        // iterate over all edges and classify as {concave, convex, neither}
        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = polygons_->begin(); it != polygons_->end(); ++it)
        {
            size_t index = std::distance(polygons_->begin(), it);
            UnalignedVector3f polygon_normal(bounded_planes_->at(index)->values[3], bounded_planes_->at(index)->values[4], bounded_planes_->at(index)->values[5]);

            size_t number_of_lines = (*it)->values.size() / 3;
            for (size_t line = 0; line < number_of_lines; ++line)
            {
                UnalignedVector3f start_point((*it)->values[3 * line], (*it)->values[3 * line + 1], (*it)->values[3 * line + 2]);
                size_t end_point_index = (line == number_of_lines - 1) ? 0 : 3 * line + 3;
                UnalignedVector3f end_point((*it)->values[end_point_index], (*it)->values[end_point_index + 1], (*it)->values[end_point_index + 2]);
                UnalignedVector3f edge_direction = (end_point - start_point);
                float edge_length_squared = edge_direction.squaredNorm();
                edge_direction.normalize();

                // create slide-image (by projecting all points onto a circle normal to the edge)
                //                Eigen::Affine3f transformToPlaneCoordinates;
                //                ::pcl::getTransformationFromTwoUnitVectorsAndOrigin(polygon_normal, edge_direction, start_point, transformToPlaneCoordinates);

                // compute max and min polar-rho's of slide edge
                float circle_radius_squared = 0.1 * 0.1;
                size_t max_index = -1;
                float max_value = 0.0;
                size_t min_index = -1;
                float min_value = 0.0;
                for (size_t i = 0; i < input->size(); ++i)
                {
                    const Point& p = (*input)[i];

                    if (!::pcl::isFinite(p))
                        continue;

                    const Eigen::Vector3f point = p.getVector3fMap();

                    if ((point - start_point).squaredNorm() > edge_length_squared)
                    {
                        if ((point - end_point).squaredNorm() > edge_length_squared)
                            continue;
                    }

                    UnalignedVector3f point_start = point - start_point;
                    UnalignedVector3f projected_point = point_start - (edge_direction.dot(point_start) * edge_direction);

                    if (projected_point.squaredNorm() < circle_radius_squared)
                    {
                        UnalignedVector3f projected_normal = (*normals)[i].getNormalVector3fMap() - (edge_direction.dot((*normals)[i].getNormalVector3fMap()) * edge_direction);

                        float val = projected_point.dot(projected_normal);

//                        ROS_INFO("projected squared norm: %f  %f  %f", projected_point.squaredNorm(), circle_radius_squared, val);

                        if (val < min_value)
                        {
                            min_value = val;
                            min_index = i;
                        }
                        else if (val > max_value)
                        {
                            max_value = val;
                            max_index = i;
                        }
                    }
                }

                if (max_index != -1 && min_index != -1)
                {
                    UnalignedVector3f mean_point = 0.5f * ((*input)[max_index].getVector3fMap() - (edge_direction.dot((*input)[max_index].getVector3fMap() - start_point) * edge_direction) +
                                                         (*input)[min_index].getVector3fMap() - (edge_direction.dot((*input)[min_index].getVector3fMap() - start_point) * edge_direction));
                    UnalignedVector3f mean_normal = 0.5f * ((*normals)[max_index].getNormalVector3fMap() + (*normals)[min_index].getNormalVector3fMap());

                    float dot_product = mean_normal.dot(mean_point);

//                    ROS_INFO("Mean point : %f, %f, %f", mean_point(0), mean_point(1), mean_point(2));
//                    ROS_INFO("Mean normal: %f, %f, %f", mean_normal(0), mean_normal(1), mean_normal(2));
//                    ROS_INFO("Dot product: %f", dot_product);

                    if (dot_product < -0.5 || dot_product > 0.5)
                    {
                        ::pcl::ModelCoefficientsPtr edge(new ::pcl::ModelCoefficients());
                        edge->values.resize(6);
                        edge->values[0] = start_point(0);
                        edge->values[1] = start_point(1);
                        edge->values[2] = start_point(2);
                        edge->values[3] = end_point(0);
                        edge->values[4] = end_point(1);
                        edge->values[5] = end_point(2);

                        if (dot_product < -0.5)
                            concave_edge_manifolds_->push_back(edge);
                        else if(dot_product > 0.5)
                            convex_edge_manifolds_->push_back(edge);
                    }
                }
            }
        }

        ROS_INFO("Edges (concave / convex): %zu / %zu", concave_edge_manifolds_->size(), convex_edge_manifolds_->size());

        publishRVizMarkers(input->header.frame_id);

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::ManifoldsAtEdges>, "ManifoldsAtEdges", "A cliff grasp.");
