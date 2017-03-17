#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;

struct PublishManifolds
{
    ros::NodeHandle nh_;
    ros::Publisher marker_publisher_;

    ecto::spore< ::posesets::PoseSetArrayConstPtr> manifolds_;
    ecto::spore< ::std::string> topic_name_;

    ros::Time last_broadcast_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<std::string>("topic_name", "Name of the topic the markers are published to.", "manifold_markers");
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("manifolds", "All the manifolds that should be published as markers.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        manifolds_ = inputs["manifolds"];

        topic_name_ = params["topic_name"];

        marker_publisher_ = nh_.advertise< ::visualization_msgs::MarkerArray>(*topic_name_, 10);
        last_broadcast_ = ros::Time::now();
    }


    ::std_msgs::ColorRGBA createColor(int strategy_type, int grasp_type)
    {
        ::std_msgs::ColorRGBA color;
        color.a = 1.0;

        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
        {
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
        }
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
        {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 1.0;
        }
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
        {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
        }
        else
        {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
        }

        return color;
    }

    std::string createString(int strategy_type, int grasp_type)
    {
        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
            return "Positionings";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return "Landings";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
            return "Pushes";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
            return "WallGrasps";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE)
            return "EdgeGrasps";

        return "Unknown";
    }

    void createGraspMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;
        msg.ns = createString(grasp.strategy, grasp.pregrasp_configuration);

        // hydro changed the scaling; 0.01 makes the grasps invisible
        msg.scale.x = 1.;
        msg.scale.y = 1.;
        msg.scale.z = 1.;

        msg.pose = grasp.pregrasp_pose.pose.pose;
        msg.points.clear();

        switch (grasp.pregrasp_configuration)
        {
        case pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.dae";
            msg.color.r = 1.0;
            msg.color.g = msg.color.b = 0;
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_BOX:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.dae";
            msg.color.g = 1.0;
            msg.color.r = msg.color.b = 0;
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.dae";
            msg.color.b = msg.color.r = 1.0;
            msg.color.g = 0;
        case pregrasp_msgs::GraspStrategy::PREGRASP_DISK:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.dae";
            msg.color.b = 1.0;
            msg.color.r = msg.color.g = 0;
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_HOOK:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_hook.dae";
            msg.color.b = 1.0;
            msg.color.r = msg.color.g = 0;
            break;
        }

        if (grasp.strategy == pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
        {
            msg.color.r = 1.0;
            msg.color.b = 1.0;
            msg.color.g = 0.0;
        }
    }

    void createArrowMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::ARROW;
        msg.ns = createString(grasp.strategy, grasp.pregrasp_configuration) + "_arrows";

        msg.scale.x = 0.01; // shaft diameter
        msg.scale.y = 0.02; // head diameter
        msg.scale.z = 0.05; // head length

        ::tf::Pose identity(::tf::Pose::getIdentity());
        ::tf::poseTFToMsg(identity, msg.pose);

        msg.points.push_back(grasp.pregrasp_pose.center.pose.position);
        msg.points.push_back(grasp.pregrasp_pose.pose.pose.position);

        msg.color = createColor(grasp.strategy, grasp.pregrasp_configuration);
        msg.color.a = 1.0;
    }

    void createPositionsMarker(::visualization_msgs::Marker& msg, const posesets::PoseSet& p)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::CUBE;
        msg.ns = "positions";

        msg.color.r = 0.0;
        msg.color.g = 1.0;
        msg.color.b = 0.0;
        msg.color.a = 0.5;

        tf::poseTFToMsg(p.getOrigin(), msg.pose);
        tf::vector3TFToMsg(p.getPositions(), msg.scale);
    }

    void createOrientationsMarker(::visualization_msgs::Marker& msg, const posesets::PoseSet& p)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::LINE_LIST;
        msg.ns = "orientations";

        ::tf::poseTFToMsg(::tf::Pose::getIdentity(), msg.pose);

        tf::Vector3 org = p.getOrigin().getOrigin();

        msg.scale.x = msg.scale.y = msg.scale.z = 0.003;

        size_t samples = 100;
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
                color.a = 1.0;//(axis == 2) ? 0.0 : 1.0;

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
    }

    void publishRVizMarkers(const posesets::PoseSetArray& manifolds)
    {
        ::visualization_msgs::MarkerArray msgs;

        int id = 0;
        ros::Time right_now = ros::Time::now();
        for (posesets::PoseSetArray::const_iterator it = manifolds.begin(); it != manifolds.end(); ++it)
        {
            ::visualization_msgs::Marker msg;
            msg.id = id;
            msg.header.frame_id = "camera_rgb_optical_frame";
            msg.header.stamp = right_now;
            msg.action = ::visualization_msgs::Marker::ADD;
            msg.lifetime = right_now - last_broadcast_;

            // create arrow from start to end pose
//            createArrowMarker(msg, *it);
//            msgs.markers.push_back(msg);

            // create marker at end pose
//            createGraspMarker(msg, *it);
//            msg.color.a = 0.7;
//            msgs.markers.push_back(msg);

            // create cube representing positions
            createPositionsMarker(msg, *it);
            msgs.markers.push_back(msg);

            // create rgb-axes representing orientations
            createOrientationsMarker(msg, *it);
            msgs.markers.push_back(msg);

            id = msg.id++;
        }

        last_broadcast_ = ros::Time::now();
        marker_publisher_.publish(msgs);
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
//        static ros::Publisher grasp_publisher = nh_.advertise<pregrasp_msgs::GraspStrategyArray>("/all_grasps", 10);

//        grasp_publisher.publish(**pregrasp_messages_ptr_);
        publishRVizMarkers(**manifolds_);

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PublishManifolds, "PublishManifolds", "Publish manifolds as markers.")
