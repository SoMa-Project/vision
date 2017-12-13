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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct ManifoldsPlanar
{
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pushing_pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> manifolds_;

    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;

    ::ros::Time last_marker_message_;
    ecto::spore<int> max_no_of_points_;
    ecto::spore<bool> all_in_plane_orientations_;
    ecto::spore<double> rotation_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<int>("max_no_of_points", "", 10);
        params.declare<bool>("all_in_plane_orientations", "Whether to add all in-plane rotations to the set of possible orientations.", true);
        params.declare<double>("rotation", "A constant rotation offset.", 0.0);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygons", "3D Polygons.").required(false);
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "Rectangular 3D Planes.");

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pushing_pregrasp_messages", "All the grasps that should be used.");
        outputs.declare< ::posesets::PoseSetArrayConstPtr>("manifolds", "All the planar manifolds found.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        polygons_ = inputs["polygons"];
        bounded_planes_ = inputs["bounded_planes"];

        pushing_pregrasp_messages_ = outputs["pushing_pregrasp_messages"];
        manifolds_ = outputs["manifolds"];

        max_no_of_points_ = params["max_no_of_points"];
        all_in_plane_orientations_ = params["all_in_plane_orientations"];
        rotation_ = params["rotation"];
        
        ros::Time::init();
        last_marker_message_ = ::ros::Time::now();
    }

    inline bool intersectRayBox(//const ::Eigen::Vector3f& ray_org, == 0
                                const ::Eigen::Vector3f& ray_dir,
                                const ::Eigen::Vector3f& box_org,
                                const ::Eigen::Matrix3f& box_rot,
                                const ::Eigen::Vector3f& box_size,
                                float &t0, float &t1)
    {
        float epsilon = 10e-4;

        int parallel = 0;
        bool found = false;

        float DA[3];
        float dA[3];
        //    ::Eigen::Vector3f d = box_org;// - ray_org;

        for (int i = 0; i < 3; ++i)
        {
            DA[i] = ray_dir.dot(box_rot.col(i));
            dA[i] = box_org.dot(box_rot.col(i));

            if (fabs(DA[i]) < epsilon)
                parallel |= 1 << i;
            else
            {
                float es = (DA[i] > 0.0) ? box_size(i) : -box_size(i);
                float invDA = 1.0 / DA[i];

                if (!found)
                {
                    t0 = (dA[i] - es) * invDA;
                    t1 = (dA[i] + es) * invDA;
                    found = true;
                }
                else
                {
                    float s = (dA[i] - es) * invDA;

                    if (s > t0)
                        t0 = s;

                    s = (dA[i] + es) * invDA;

                    if (s < t1)
                        t1 = s;

                    if (t0 > t1)
                        return false;
                }
            }
        }

        if (parallel)
            for (int i = 0; i < 3; ++i)
                if (parallel & (1 << i))
                    if (fabs(dA[i] - t0 * DA[i]) > box_size(i) || fabs(dA[i] - t1 * DA[i]) > box_size(i))
                        return false;

        return found;
    }

    template <typename Point>
    int cropByRayIntersection (const ::pcl::PointCloud<Point> &cloud,
                               //                              const std::vector<int> &indices,
                               const ::Eigen::Vector3f& box_org,
                               const ::Eigen::Matrix3f& box_rot,
                               const ::Eigen::Vector3f& box_size,
                               std::vector<int> &cropped_indices,
                               int &occluding_point_count)
    {
        cropped_indices.clear();

        int intersection_point_count = 0;
        occluding_point_count = 0;

        //      for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            //        if (!isFinite (cloud[*iIt]))
            if (!isFinite (cloud.points[i]))
                continue;

            float t0, t1;
            ::Eigen::Vector3f ray_dir(cloud[i].x, cloud[i].y, cloud[i].z);
            //        ::Eigen::Vector3f ray_dir(cloud[*iIt].x, cloud[*iIt].y, cloud[*iIt].z);
            float l = ray_dir.norm();
            ray_dir.normalize();
            if (intersectRayBox(ray_dir, box_org, box_rot, box_size, t0, t1))
            {
                intersection_point_count++;
                //        if ((t0 <= l && l <= t1) || (t1 <= l && l <= t0))
                if (l <= t0)
                    occluding_point_count++;
                else if (t0 <= l && l <= t1)
                    //            cropped_indices.push_back(*iIt);
                    cropped_indices.push_back(i);
            }
        }
        return intersection_point_count;
    }

    template<typename Point>
    int countPoints(const pregrasp_msgs::GraspStrategy& g, boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
        UnalignedVector4f min, max;
        max <<  0.07, 0.17,  0.15, 0;
        min << -0.07, 0.07, -0.1, 0;

        ::tf::Transform bt;
        ::tf::poseMsgToTF(g.pregrasp_pose.pose.pose, bt);
        ::Eigen::Affine3d approach_transform_d;
        ::tf::transformTFToEigen(bt, approach_transform_d);

        UnalignedAffine3f approach_transform = approach_transform_d.cast<float>();// * Eigen::Translation3f(0, 0, -approach_length_2);

        //        std::vector<int> cropped_indices;
        //        ::Eigen::Vector3f box_org = approach_transform.translation();
        //        UnalignedVector4f side_lengths = 0.5 * (max - min);
        //        ::Eigen::Vector3f box_size = side_lengths.topRows(3);
        //        ::Eigen::Matrix3f box_rot = approach_transform.rotation();
        //        int occluding_point_count = 0;
        //        int total_point_count = cropByRayIntersection(*input, box_org, box_rot, box_size, cropped_indices, occluding_point_count);

        //        ROS_INFO("Other result: occludingpc: %i   total: %i   cropped_ind: %zu", occluding_point_count, total_point_count, cropped_indices.size());

        ::pcl::CropBox<Point> crop_filter;
        crop_filter.setTransform(approach_transform.inverse());
        crop_filter.setMax(max);
        crop_filter.setMin(min);
        crop_filter.setKeepOrganized(true);
        crop_filter.setInputCloud(input);

        ::pcl::PointIndices inliers;
        crop_filter.filter(inliers.indices);

        return inliers.indices.size();
    }

    void publishRVizMarkers(const std::string& frame_id, int non_intersections)
    {
        static ros::NodeHandle nh;
        static ros::Publisher planar_manifolds_publisher = nh.advertise< ::visualization_msgs::MarkerArray>("/planar_manifolds", 10);
        static int id = 0;

        ::visualization_msgs::MarkerArray msgs;

        for (size_t i = 0; i < (*manifolds_)->size(); ++i)
        {
            const ::posesets::PoseSet& p = (*manifolds_)->at(i);

            tf::Vector3 org = p.getOrigin().getOrigin();

            ::visualization_msgs::Marker msg;
            msg.ns = (i < non_intersections) ? "originals" : "intersections";
            msg.id = id++;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame_id;
            msg.type = ::visualization_msgs::Marker::LINE_LIST;
            msg.action = ::visualization_msgs::Marker::ADD;
            msg.lifetime = ::ros::Time::now() - last_marker_message_;

            msg.scale.x = msg.scale.y = msg.scale.z = 0.01;

            size_t samples = 30;
            msg.points.resize(2 * 3 * samples);
            msg.colors.resize(2 * 3 * samples);

            for (size_t i = 0; i < samples; ++i)
            {
                UnalignedMatrix3f mat = p.getOrientations().sample();

                for (int axis = 0; axis < 3; ++axis)
                {
                    std_msgs::ColorRGBA color;
                    color.r = (axis == 0) ? 1.0 : 0.0;
                    color.g = (axis == 1) ? 1.0 : 0.0;
                    color.b = (axis == 2) ? 1.0 : 0.0;
                    color.a = 1.0;

                    msg.points[6 * i + axis * 2].x = 0 + org.x();
                    msg.points[6 * i + axis * 2].y = 0 + org.y();
                    msg.points[6 * i + axis * 2].z = 0 + org.z();
                    msg.colors[6 * i + axis * 2] = color;

                    msg.points[6 * i + axis * 2 + 1].x = mat.col(axis)(0)*0.07 + org.x();
                    msg.points[6 * i + axis * 2 + 1].y = mat.col(axis)(1)*0.07 + org.y();
                    msg.points[6 * i + axis * 2 + 1].z = mat.col(axis)(2)*0.07 + org.z();
                    msg.colors[6 * i + axis * 2 + 1] = color;
                }
            }
            msgs.markers.push_back(msg);

            msg.id = id++;
            msg.type = ::visualization_msgs::Marker::CUBE;
            tf::poseTFToMsg(p.getOrigin(), msg.pose);
            tf::vector3TFToMsg(p.getPositions(), msg.scale);
            msg.color.g = 1.0;
            msg.color.r = msg.color.b = 0.0;
            msg.color.a = 0.3;
            msgs.markers.push_back(msg);


            // publish intersection points
//            msg.id = id++;
////            msg.type = ::visualization_msgs::Marker::LINE_LIST;
//            msg.type = ::visualization_msgs::Marker::SPHERE_LIST;
//            msg.color.g = 0.0;
//            msg.color.r = msg.color.b = 1.0;
//            msg.color.a = 1.0;
//            tf::poseTFToMsg(tf::Pose::getIdentity(), msg.pose);
//            tf::vector3TFToMsg(tf::Vector3(0.02, 0.02, 0.02), msg.scale);
//            msg.points.clear();
//            for (size_t j = 0; j < p.intersection_points.size(); ++j)
//            {
//                geometry_msgs::Point intersect;
//                intersect.x = p.intersection_points[j].X();
//                intersect.y = p.intersection_points[j].Y();
//                intersect.z = p.intersection_points[j].Z();
//                msg.points.push_back(intersect);
//            }

//            Wm5::Vector3d vertices[8];
//            Wm5::Segment3d edges[12];
//            p.updateBoxVertices(vertices, edges);
//            for (int k = 0; k < 8; ++k)
//            {
//                geometry_msgs::Point intersect;
//                intersect.x = vertices[k].X();
//                intersect.y = vertices[k].Y();
//                intersect.z = vertices[k].Z();
//                msg.points.push_back(intersect);
//            }
//            for (int k = 0; k < 12; ++k)
//            {
//                geometry_msgs::Point intersect;
//                intersect.x = edges[k].P0.X();
//                intersect.y = edges[k].P0.Y();
//                intersect.z = edges[k].P0.Z();
//                msg.points.push_back(intersect);
//                intersect.x = edges[k].P1.X();
//                intersect.y = edges[k].P1.Y();
//                intersect.z = edges[k].P1.Z();
//                msg.points.push_back(intersect);
//            }

            msgs.markers.push_back(msg);
        }

        last_marker_message_ = ::ros::Time::now();
        planar_manifolds_publisher.publish(msgs);
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
        pregrasp_msgs::GraspStrategyArrayPtr push_messages(new ::pregrasp_msgs::GraspStrategyArray());
        ::posesets::PoseSetArrayPtr manifolds(new ::posesets::PoseSetArray());

        push_messages->header = pcl_conversions::fromPCL(input->header);

//        int dbg_cnt = -1;
//        int dbg_one = 0;
//        int dbg_two = 2;
        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = bounded_planes_->begin(); it != bounded_planes_->end(); ++it)
        {
//            dbg_cnt++;
//            if (dbg_cnt != dbg_one && dbg_cnt != dbg_two)
//                continue;

            tf::Vector3 origin_position((*it)->values[0], (*it)->values[1], (*it)->values[2]);
            tf::Vector3 normal((*it)->values[3], (*it)->values[4], (*it)->values[5]);
            tf::Vector3 principal_axis((*it)->values[6], (*it)->values[7], (*it)->values[8]);
            tf::Vector3 plane_size(principal_axis.length(), (*it)->values[9], 0.02);
            normal.normalize();
            principal_axis.normalize();
            tf::Vector3 third_axis = normal.cross(principal_axis);
//            tf::Matrix3x3 rotation(principal_axis.x(), principal_axis.y(), principal_axis.z(),
//                                   third_axis.x(), third_axis.y(), third_axis.z(),
//                                   normal.x(), normal.y(), normal.z());
            tf::Matrix3x3 rotation(principal_axis.x(), third_axis.x(), normal.x(),
                                   principal_axis.y(), third_axis.y(), normal.y(),
                                   principal_axis.z(), third_axis.z(), normal.z());
            tf::Matrix3x3 offset(tf::Quaternion(tf::Vector3(0, 0, 1), *rotation_));
            rotation *= offset;
            
            tf::Transform origin(rotation, origin_position);

            tf::Quaternion q;
            rotation.getRotation(q);

            ::posesets::PoseSet s(origin);
            s.setPositions(plane_size);

            // C.E.: Added this to connect wall grasps and slides. In general, I think we would like to allow rotation around each edge that is bounding this plane. This can be done using the polygon info already available.
            s.getOrientations().addAll();
            //if (*all_in_plane_orientations_)
            //    s.getOrientations().add(q, normal);
            //else
            //    s.getOrientations().addFuzzy(q);
            manifolds->push_back(s);

            // old stuff to make things compatible
            ::pregrasp_msgs::GraspStrategy msg;
            msg.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_HOOK;
            msg.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_PUSH;

            tf::poseTFToMsg(origin, msg.pregrasp_pose.center.pose);
            tf::poseTFToMsg(origin, msg.pregrasp_pose.pose.pose);
            msg.pregrasp_pose.pose.header = msg.object.pose.header = msg.object.center.header = push_messages->header;
            msg.object.center.pose = msg.object.pose.pose = msg.pregrasp_pose.center.pose;

            msg.pregrasp_pose.size.resize(4);
            msg.pregrasp_pose.size[0] = plane_size[0];
            msg.pregrasp_pose.size[1] = plane_size[1];
            msg.pregrasp_pose.size[2] = plane_size[2];
            msg.pregrasp_pose.size[3] = 0.1;

            msg.pregrasp_pose.image_size.resize(4);
            msg.pregrasp_pose.image_size[0] = plane_size[0];
            msg.pregrasp_pose.image_size[1] = plane_size[1];
            msg.pregrasp_pose.image_size[2] = plane_size[2];
            msg.pregrasp_pose.image_size[3] = 0.1;

            msg.object.size.push_back(0.1);
            msg.object.size.push_back(0.1);
            msg.object.size.push_back(0.07);
            msg.object.size.push_back(4.0);
            msg.object.image_size.push_back(0.01);
            msg.object.image_size.push_back(0.1);
            msg.object.image_size.push_back(0.1);
            msg.object.image_size.push_back(4.0);

            push_messages->strategies.push_back(msg);
        }

        size_t manifold_size = manifolds->size();
//        for (size_t i = 0; i < manifold_size; ++i)
//        {
//            for (size_t j = 0; j < manifold_size; ++j)
//            {
//                if (i >= j)
//                    continue;

//                ::posesets::PoseSet p = manifolds->at(i);
//                if (p.intersect(manifolds->at(j)))
//                    manifolds->push_back(p);
//            }
//        }

        (*pushing_pregrasp_messages_) = push_messages;
        (*manifolds_) = manifolds;

        
        //TODO: publishRVizMarkers(input->header.frame_id, manifold_size);

        ROS_INFO("Number of planar manifolds: %zu (without intersections: %zu)", (*manifolds_)->size(), manifold_size);

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::ManifoldsPlanar>, "ManifoldsPlanar", "Generates Planar Manifolds.");
