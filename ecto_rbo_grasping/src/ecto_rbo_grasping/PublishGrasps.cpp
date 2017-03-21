/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto/ecto.hpp>

#include <time.h>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct PublishGrasps
{
    ros::NodeHandle nh_;
    ros::Publisher grasp_publisher_, marker_publisher_;
    
    ecto::spore<pregrasp_msgs::GraspStrategyArray> pregrasp_messages_;
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_ptr_;
    
    ecto::spore<std::string> marker_mesh_resource_;
    ecto::spore<UnalignedVector4f> marker_color_;
    ecto::spore<UnalignedVector3f> marker_offset_position_;
    ecto::spore<UnalignedVector4f> marker_offset_rotation_xyzw_;
    
    ros::Time last_broadcast_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<std::string>("topic_name", "An string to be used as topic name for marker and grasp messages.", "/all_grasps").required(false);
        params.declare<std::string>("marker_mesh_resource", "An url of a mesh to be used in RViz.").required(false);
        params.declare<UnalignedVector4f>("marker_color", "Color vector.").required(false);
        params.declare<UnalignedVector3f>("marker_offset_position", "Position vector.").required(false);
        params.declare<UnalignedVector4f>("marker_offset_rotation_xyzw", "Quaternion.").required(false);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<pregrasp_msgs::GraspStrategyArray>("pregrasp_messages", "All the grasps that should be published.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages_ptr", "All the grasps that should be published.").required(false);
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        pregrasp_messages_ = inputs["pregrasp_messages"];
        pregrasp_messages_ptr_ = inputs["pregrasp_messages_ptr"];
        
        marker_mesh_resource_ = params["marker_mesh_resource"];
        marker_color_ = params["marker_color"];
        marker_offset_position_ = params["marker_offset_position"];
        marker_offset_rotation_xyzw_ = params["marker_offset_rotation_xyzw"];

        last_broadcast_ = ros::Time::now();
        grasp_publisher_ = nh_.advertise<pregrasp_msgs::GraspStrategyArray>(params["topic_name"]->get<std::string>(), 10);
        marker_publisher_ = nh_.advertise< ::visualization_msgs::MarkerArray>(params["topic_name"]->get<std::string>() + "_marker", 10);
        
    }

    ::std_msgs::ColorRGBA createColor(int strategy_type, int grasp_type)
    {
        ::std_msgs::ColorRGBA color;
        color.a = 1.0;

        switch (strategy_type)
        {
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION:
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND:
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH:
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE:
            // cyan
            color.r = 0.0;
            color.g = 1.0;
            color.b = 1.0;
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE:
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_APPROACH_THEN_SQUEEZE:
            // orange
            color.r = 1.0;
            color.g = 0.64705882352;
            color.b = 0.0;
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP:
            // purple
            color.r = 0.5;
            color.g = 0.0;
            color.b = 0.5;
            break;
        default:
            // white
            color.r = 1.0;
            color.g = 1.0;
            color.b = 1.0;
        }

        return color;
    }

    std::string createString(int strategy_type, int grasp_type)
    {
        std::string first_word("Unknown");
        std::string second_word("Unknown");

        switch (strategy_type)
        {
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION:
            first_word = "Position";
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND:
            first_word = "Land";
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH:
            first_word = "Push";
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE:
            first_word = "SlideToEdge";
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE:
            first_word = "Squeeze";
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_APPROACH_THEN_SQUEEZE:
            first_word = "ApproachThenSqueeze";
            break;
        case ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP:
            first_word = "WallGrasp";
            break;
        }

        switch (grasp_type)
        {
        case ::pregrasp_msgs::GraspStrategy::PREGRASP_DISK:
            second_word = "Disk";
            break;
        case ::pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER:
            second_word = "Cylinder";
            break;
        case ::pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE:
            second_word = "Sphere";
            break;
        case ::pregrasp_msgs::GraspStrategy::PREGRASP_HOOK:
            second_word = "Hook";
            break;
        case ::pregrasp_msgs::GraspStrategy::PREGRASP_BOX:
            second_word = "Box";
            break;
        }

        return first_word + "_" + second_word;
    }
    
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& p)
    {
        tf::Transform t;
        tf::poseMsgToTF(p, t);
        
        tf::Transform offset = tf::Transform::getIdentity();
        if (marker_offset_position_.user_supplied())
            offset.setOrigin(tf::Vector3((*marker_offset_position_)[0],
                                         (*marker_offset_position_)[1],
                                         (*marker_offset_position_)[2]));
        if (marker_offset_rotation_xyzw_.user_supplied())
            offset.setRotation(tf::Quaternion((*marker_offset_rotation_xyzw_)[0],
                                              (*marker_offset_rotation_xyzw_)[1],
                                              (*marker_offset_rotation_xyzw_)[2],
                                              (*marker_offset_rotation_xyzw_)[3]));
        
        t *= offset;
        
        geometry_msgs::Pose result;
        tf::poseTFToMsg(t, result);
        return result;
    }
    
    geometry_msgs::Pose rotatePose(const geometry_msgs::Pose& p)
    {
        // we need this because the obj-file that the marker is showing
        // in rviz has a different frame attached than the dae-file
        // (before z was pointing out of the palm, now it's pointing
        // along the thumb --> we rotate 90deg around x to fix it)
        
        tf::Transform t;
        tf::poseMsgToTF(p, t);
        
        t.setRotation(t.getRotation() * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2));
        
        geometry_msgs::Pose result;
        tf::poseTFToMsg(t, result);
        return result;
    }

    void createGraspMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;

        switch (grasp.pregrasp_configuration)
        {
        case pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.obj";
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_BOX:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.obj";
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.obj";
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_DISK:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.obj";
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_HOOK:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_hook.obj";
            break;
        }

        msg.ns = createString(grasp.strategy, grasp.pregrasp_configuration);

        // hydro-rviz changed the scaling; 0.01 makes the grasps too small/invisible
        msg.scale.x = msg.scale.y = msg.scale.z = 1.0;

        //msg.pose = rotatePose(grasp.pregrasp_pose.pose.pose);
        msg.pose = transformPose(grasp.pregrasp_pose.pose.pose);
        msg.points.clear();

        msg.color = createColor(grasp.strategy, grasp.pregrasp_configuration);
        msg.color.a = 0.7;
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

    void createPreImageMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::CUBE;
        msg.ns = createString(grasp.strategy, grasp.pregrasp_configuration) + "_preimages";

        msg.color.r = 0.0;
        msg.color.g = 1.0;
        msg.color.b = 0.0;
        msg.color.a = 0.5;

        if (grasp.pregrasp_pose.size.empty())
        {
            msg.scale.x = msg.scale.y = msg.scale.z = 0.1;
        }
        else if (grasp.pregrasp_pose.size.size() < 3)
        {
            msg.scale.x = msg.scale.y = msg.scale.z = grasp.pregrasp_pose.size[0];
        }
        else
        {
            msg.scale.x = grasp.pregrasp_pose.size[0];
            msg.scale.y = grasp.pregrasp_pose.size[1];
            msg.scale.z = grasp.pregrasp_pose.size[2];
        }

        msg.pose = grasp.pregrasp_pose.center.pose;
    }

    void createImageMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.id++;
        msg.type = ::visualization_msgs::Marker::CUBE;
        msg.ns = createString(grasp.strategy, grasp.pregrasp_configuration) + "_images";

        msg.color.r = 0.0;
        msg.color.g = 0.0;
        msg.color.b = 1.0;
        msg.color.a = 0.5;

        if (grasp.pregrasp_pose.image_size.empty())
        {
            msg.scale.x = msg.scale.y = msg.scale.z = 0.1;
        }
        else if (grasp.pregrasp_pose.image_size.size() < 3)
        {
            msg.scale.x = msg.scale.y = msg.scale.z = grasp.pregrasp_pose.image_size[0];
        }
        else
        {
            msg.scale.x = grasp.pregrasp_pose.image_size[0];
            msg.scale.y = grasp.pregrasp_pose.image_size[1];
            msg.scale.z = grasp.pregrasp_pose.image_size[2];
        }

        msg.pose = grasp.pregrasp_pose.pose.pose;
    }

    void publishRVizMarkers(const pregrasp_msgs::GraspStrategyArray& grasps)
    {
        ::visualization_msgs::MarkerArray msgs;
        
        int id = 0;
        ros::Time right_now = ros::Time::now();
        for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = grasps.strategies.begin(); it != grasps.strategies.end(); ++it)
        {
            ::visualization_msgs::Marker msg;
            msg.id = id;
            msg.header = grasps.header;
            msg.header.frame_id = "/camera_rgb_optical_frame";
            msg.header.stamp = right_now;
            msg.action = ::visualization_msgs::Marker::ADD;
            msg.lifetime = right_now - last_broadcast_;

            // create arrow from start to end pose
//            createArrowMarker(msg, *it);
//            msgs.markers.push_back(msg);

            // create marker at end pose
            createGraspMarker(msg, *it);
            msg.color.a = 0.7;
            
            if (marker_mesh_resource_.user_supplied())
            {
                msg.mesh_resource = *marker_mesh_resource_;
                msg.scale.x = msg.scale.y = msg.scale.z = 0.1;
                msg.color.r = msg.color.g = msg.color.b = msg.color.a = 0;
                msg.mesh_use_embedded_materials = true;
            }
            
            if (marker_color_.user_supplied())
            {
                msg.color.r = (*marker_color_)[0];
                msg.color.g = (*marker_color_)[1];
                msg.color.b = (*marker_color_)[2];
                msg.color.a = (*marker_color_)[3];
            }
            
            msgs.markers.push_back(msg);

            // create pre-image cube at start pose
//            createPreImageMarker(msg, *it);
//            msgs.markers.push_back(msg);

            // create image cube at end pose
//            createImageMarker(msg, *it);
//            msgs.markers.push_back(msg);

            id = msg.id++;
        }

        last_broadcast_ = ros::Time::now();
        marker_publisher_.publish(msgs);

        ROS_INFO("Number of published grasps: %zu", grasps.strategies.size());
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        if (pregrasp_messages_.user_supplied())
        {
            grasp_publisher_.publish(*pregrasp_messages_);
            publishRVizMarkers(*pregrasp_messages_);
        }
        else if (pregrasp_messages_ptr_.user_supplied())
        {
            grasp_publisher_.publish(**pregrasp_messages_ptr_);
            publishRVizMarkers(**pregrasp_messages_ptr_);
        }

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PublishGrasps, "PublishGrasps", "Publish some grasps.")
