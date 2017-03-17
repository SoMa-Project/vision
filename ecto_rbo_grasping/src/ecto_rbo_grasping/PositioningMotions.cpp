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

#include <pcl_conversions/pcl_conversions.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

#include <Eigen/Geometry>

using namespace ecto;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct PositioningMotions
{
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> positioning_pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> positioning_manifolds_;

    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;

    spore<std::vector< ::pcl::PointIndices > > clusters_;
    
    ros::Time last_marker_message_;

    // parameters
    //  spore<double> distance_threshold_;
    //  spore<double> min_inlier_ratio_;
    //  spore<double> weight_contour_;
    //  spore<double> min_boxness_;
    //  spore<double> max_size_;
    //  spore<double> min_size_;

    static void declare_params(ecto::tendrils& params)
    {
        //    params.declare<double>("min_inlier_ratio", "Minimum number of inlier points per plane.", 0.5);
        //    params.declare<double>("distance_threshold", "Maximum mean error a plane fit may have.", 0.05);
        //    params.declare<double>("min_boxness", "", 0.8);
        //    params.declare<double>("weight_contour", "", 0.5);
        //    params.declare<double>("max_size", "", 0.24);
        //    params.declare<double>("min_size", "", 0.08);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::PointIndices> >("clusters", "Clusters.");

        outputs.declare< pregrasp_msgs::GraspStrategyArrayConstPtr>("positioning_pregrasp_messages", "All the grasps that should be used.");
        outputs.declare< ::posesets::PoseSetArrayConstPtr>("positioning_manifolds", "All the grasps that should be used.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        clusters_ = inputs["clusters"];

        positioning_pregrasp_messages_ = outputs["positioning_pregrasp_messages"];
        positioning_manifolds_ = outputs["positioning_manifolds"];

        last_marker_message_ = ros::Time::now();
    }

    inline void computeNormalMean(const ::pcl::PointCloud< ::pcl::Normal>& normals, const ::pcl::PointIndices& indices, Eigen::Vector3f& mean_normal)
    {
        mean_normal.setZero();
        for (size_t i = 0; i < indices.indices.size(); ++i)
            mean_normal += normals.at(indices.indices[i]).getNormalVector3fMap();

        mean_normal.normalize();
    }

    template<typename Point>
    inline float pointToLineDistance(const Point p, ::pcl::ModelCoefficientsConstPtr& line)
    {
        //    ::Eigen::Vector4f point(p.x, p.y, p.z, 0.0f);
        //    ::Eigen::Vector4f line_point(line->values[0], line->values[1], line->values[2], 0.0f);
        //    ::Eigen::Vector4f line_direction(line->values[3], line->values[4], line->values[5], 0.0f);
        //    return ::pcl::sqrPointToLineDistance(point, line_point, line_direction);
        return (p.x-line->values[0]) * (p.x-line->values[0]) + (p.y-line->values[1]) * (p.y-line->values[1]) + (p.z-line->values[2]) * (p.z-line->values[2]);
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
        pregrasp_msgs::GraspStrategyArrayPtr positioning_messages(new ::pregrasp_msgs::GraspStrategyArray());
        positioning_messages->header = pcl_conversions::fromPCL(input->header);
        
        ::posesets::PoseSetArrayPtr positioning_manifolds(new ::posesets::PoseSetArray());

        ::pregrasp_msgs::GraspStrategy msg;
        msg.pregrasp_configuration = pregrasp_msgs::GraspStrategy::PREGRASP_HOOK;
        msg.strategy = pregrasp_msgs::GraspStrategy::STRATEGY_POSITION;

        tf::Point position(0, 0, 0.5);
        tf::Quaternion orientation(tf::Quaternion::getIdentity());

        tf::quaternionTFToMsg(orientation, msg.pregrasp_pose.center.pose.orientation);
        tf::pointTFToMsg(position, msg.pregrasp_pose.center.pose.position);
    
        msg.pregrasp_pose.pose.header = positioning_messages->header;
        msg.pregrasp_pose.pose.pose.position = msg.pregrasp_pose.center.pose.position;
        msg.pregrasp_pose.pose.pose.orientation = msg.pregrasp_pose.center.pose.orientation;

        msg.pregrasp_pose.size.push_back(10.0);
        msg.pregrasp_pose.size.push_back(4.0);

        msg.pregrasp_pose.image_size.push_back(10.0);
        msg.pregrasp_pose.image_size.push_back(4.0);

        // set object pose relative to hand
        msg.object.center.pose = msg.object.pose.pose = msg.pregrasp_pose.center.pose;
        msg.object.size.push_back(10.0);
        msg.object.size.push_back(10.0);
        msg.object.size.push_back(10.0);
        msg.object.size.push_back(4.0);
        msg.object.image_size.push_back(10.0);
        msg.object.image_size.push_back(10.0);
        msg.object.image_size.push_back(10.0);
        msg.object.image_size.push_back(4.0);

        positioning_messages->strategies.push_back(msg);

        // add the corresponding manifold
        ::posesets::PoseSet ps(tf::Transform(orientation, position));
        ps.setPositions(tf::Vector3(4.5, 4.5, 4.5));
        ps.getOrientations().addAll();
        positioning_manifolds->push_back(ps);

        (*positioning_pregrasp_messages_) = positioning_messages;
        (*positioning_manifolds_) = positioning_manifolds;

        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::PositioningMotions>, "PositioningMotions", "Finding positioning motions.");
