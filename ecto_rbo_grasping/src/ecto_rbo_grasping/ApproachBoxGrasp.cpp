#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>


using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct ApproachBoxGrasp
{
    spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;
    spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_with_approach_quality_;

    spore<std::vector<double> > approach_occupancy_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "");

        outputs.declare<std::vector<double> >("occupancy", "Occupancy of the approach volume.");
        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        pregrasp_messages_ = inputs["pregrasp_messages"];

        approach_occupancy_ = outputs["occupancy"];
        pregrasp_messages_with_approach_quality_ = outputs["pregrasp_messages"];
    }

    void publishRVizMarker(const std::string& frame_id, const std::vector<UnalignedAffine3f>& transforms, const std::vector<UnalignedVector3f>& sizes, const std::vector<double>& transparency) {
        visualization_msgs::MarkerArray markers;

        for (size_t i = 0; i < transforms.size(); ++i)
        {
            visualization_msgs::Marker marker;

            tf::Transform bt;
            tf::transformEigenToTF((Eigen::Affine3d) transforms[i].cast<double>(), bt);
            tf::poseTFToMsg(bt, marker.pose);

            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.ns = "approach_volume";
            marker.lifetime = ros::Duration(0.2);
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = (sizes[i])(0);
            marker.scale.y = (sizes[i])(1);
            marker.scale.z = (sizes[i])(2);

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0 - transparency[i];
            markers.markers.push_back(marker);
        }

        static ros::NodeHandle nh;
        static ros::Publisher approach_marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/approach_volume", 1, true);
        approach_marker_publisher.publish(markers);
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
                               const std::vector<int> &indices,
                               const ::Eigen::Vector3f& box_org,
                               const ::Eigen::Matrix3f& box_rot,
                               const ::Eigen::Vector3f& box_size,
                               std::vector<int> &cropped_indices,
                               int &occluding_point_count)
    {
        cropped_indices.clear();

        int intersection_point_count = 0;
        occluding_point_count = 0;

        for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
            //    for (size_t i = 0; i < cloud.size(); ++i)
        {
            if (!isFinite (cloud[*iIt]))
                //      if (!isFinite (cloud.points[i]))
                continue;

            float t0, t1;
            //      ::Eigen::Vector3f ray_dir(cloud[i].x, cloud[i].y, cloud[i].z);
            ::Eigen::Vector3f ray_dir(cloud[*iIt].x, cloud[*iIt].y, cloud[*iIt].z);
            float l = ray_dir.norm();
            ray_dir.normalize();
            if (intersectRayBox(ray_dir, box_org, box_rot, box_size, t0, t1))
            {
                intersection_point_count++;
                //        if ((t0 <= l && l <= t1) || (t1 <= l && l <= t0))
                if (l <= t0)
                    occluding_point_count++;
                else if (t0 <= l && l <= t1)
                    cropped_indices.push_back(*iIt);
                //          cropped_indices.push_back(i);
            }
        }
        return intersection_point_count;
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
        std::vector<int> indices_without_NANs (input->size());
        size_t id_without_NANs = 0;
        for (size_t i = 0; i < input->size(); ++i)
        {
            if (input->points[i].x == 0 && input->points[i].y == 0 && input->points[i].z == 0)
                continue;
            indices_without_NANs[id_without_NANs++] = static_cast<int>(i);
        }
        indices_without_NANs.resize(id_without_NANs);

        std::vector<UnalignedAffine3f> approach_transforms;
        std::vector<UnalignedVector3f> approach_sizes;
        std::vector<double> approach_occupancy;

        UnalignedVector4f max;
        UnalignedVector4f min;

        double approach_length_2 = 0.2; // half of the length

        pregrasp_msgs::GraspStrategyArrayPtr pregrasp_messages_with_approach_quality(new ::pregrasp_msgs::GraspStrategyArray(**pregrasp_messages_));
//        pregrasp_messages_with_approach_quality->header = (*pregrasp_messages_)->header;

        // iterate through each potential box grasp and check how many points are inside the approach volume
        for (size_t i = 0; i < (*pregrasp_messages_)->strategies.size(); ++i)
        {
            ::tf::Transform bt;
            ::tf::poseMsgToTF((*pregrasp_messages_)->strategies[i].pregrasp_pose.pose.pose, bt);
            ::Eigen::Affine3d approach_transform_d;
            ::tf::transformTFToEigen(bt, approach_transform_d);

            UnalignedAffine3f approach_transform = approach_transform_d.cast<float>() * Eigen::Translation3f(0, 0, -approach_length_2);

            UnalignedVector3f approach_size_2(0, 0, approach_length_2);
            switch ((*pregrasp_messages_)->strategies[i].pregrasp_configuration)
            {
            case pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER:
            case pregrasp_msgs::GraspStrategy::PREGRASP_BOX:
                approach_size_2(0) = 0.07;
                approach_size_2(1) = 0.15;
                break;
            case pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE:
            case pregrasp_msgs::GraspStrategy::PREGRASP_DISK:
                approach_size_2(0) = 0.15;
                approach_size_2(1) = 0.15;
                break;
            }

            max(0) = +approach_size_2(0);
            max(1) = +approach_size_2(1);
            max(2) = +approach_size_2(2);
            min(0) = -approach_size_2(0);
            min(1) = -approach_size_2(1);
            min(2) = -approach_size_2(2);

            ::pcl::CropBox<Point> crop_filter;
            crop_filter.setTransform(approach_transform.inverse());
            //crop_filter.setRotation(rotation_);
            //crop_filter.setTranslation(translation_);
            crop_filter.setMax(max);
            crop_filter.setMin(min);
            crop_filter.setKeepOrganized(true);
            crop_filter.setInputCloud(input);

            Indices::Ptr inliers ( new Indices() );
            crop_filter.filter(inliers->indices);

//            std::vector<int> cropped_indices;
//            ::Eigen::Vector3f box_org = approach_transform.translation();
//            ::Eigen::Vector3f box_size = approach_size_2 * 1.0;
//            ::Eigen::Matrix3f box_rot = approach_transform.rotation();
//            int occluding_point_count = 0;
//            int total_point_count = cropByRayIntersection(*input, indices_without_NANs, box_org, box_rot, box_size, cropped_indices, occluding_point_count);

            double approach_quality = ::std::max(0.0, 1.0 - static_cast<double>(inliers->indices.size()) / 10000.0);
//            std::cout << cropped_indices.size() << " " << total_point_count << " " << occluding_point_count << " old approach quality " << approach_quality;
//            approach_quality = static_cast<double>(total_point_count - cropped_indices.size()) / (2.0 * occluding_point_count + total_point_count);
//            std::cout << " vs new one: " << approach_quality << std::endl;

            pregrasp_messages_with_approach_quality->strategies[i].quality_approach = approach_quality;

            approach_occupancy.push_back(approach_quality);
            approach_transforms.push_back(approach_transform);
            approach_sizes.push_back(approach_size_2 * 2.0);

//            ROS_INFO("Approach Box, no. of inliers: %zu -- %f", inliers->indices.size(), approach_occupancy.back());
        }

        (*pregrasp_messages_with_approach_quality_) = pregrasp_messages_with_approach_quality;
        (*approach_occupancy_) = approach_occupancy;

        publishRVizMarker(input->header.frame_id, approach_transforms, approach_sizes, approach_occupancy);

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCell<ecto_rbo_grasping::ApproachBoxGrasp>, "ApproachBoxGrasp", "Predict the success of approach a particular box grasp.");
