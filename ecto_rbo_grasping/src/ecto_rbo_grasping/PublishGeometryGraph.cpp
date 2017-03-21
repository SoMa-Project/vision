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

#include <geometry_graph_msgs/Graph.h>

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct PublishGeometryGraph
{
    ros::NodeHandle nh_;
    ros::Publisher graph_publisher_, marker_publisher_;
    
    ecto::spore<geometry_graph_msgs::GraphConstPtr> graph_message_;
    
    ecto::spore<std::string> marker_mesh_resource_;
    ecto::spore<UnalignedVector4f> marker_color_;
    ecto::spore<UnalignedVector3f> marker_offset_position_;
    ecto::spore<UnalignedVector4f> marker_offset_rotation_xyzw_;
    
    ros::Time last_broadcast_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<std::string>("topic_name", "An string to be used as topic name for marker and grasp messages.", "/geometry_graph").required(false);
        params.declare<std::string>("marker_mesh_resource", "An url of a mesh to be used in RViz.").required(false);
        params.declare<UnalignedVector4f>("marker_color", "Color vector.").required(false);
        params.declare<UnalignedVector3f>("marker_offset_position", "Position vector.").required(false);
        params.declare<UnalignedVector4f>("marker_offset_rotation_xyzw", "Quaternion.").required(false);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<geometry_graph_msgs::GraphConstPtr>("graph_message", "The geometry graph that should be published.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        graph_message_ = inputs["graph_message"];
        
        marker_mesh_resource_ = params["marker_mesh_resource"];
        marker_color_ = params["marker_color"];
        marker_offset_position_ = params["marker_offset_position"];
        marker_offset_rotation_xyzw_ = params["marker_offset_rotation_xyzw"];

        last_broadcast_ = ros::Time::now();
        graph_publisher_ = nh_.advertise<geometry_graph_msgs::Graph>(params["topic_name"]->get<std::string>(), 10);
        marker_publisher_ = nh_.advertise< ::visualization_msgs::MarkerArray>(params["topic_name"]->get<std::string>() + "_marker", 10);
        
    }

    geometry_msgs::Pose createPoseMsg(const geometry_msgs::Transform& t)
    {
        geometry_msgs::Pose p;
        p.position = createPointMsg(t.translation);
        p.orientation = t.rotation;
        return p;
    }

    geometry_msgs::Point createPointMsg(const geometry_msgs::Vector3& v)
    {
        geometry_msgs::Point p;
        p.x = v.x;
        p.y = v.y;
        p.z = v.z;
        return p;
    }
    
    void addCylinderMarker(::visualization_msgs::MarkerArray& msgs, double color_r, double color_g, double color_b, double length, const geometry_msgs::Transform& t, const ::std_msgs::Header& default_header, const ros::Time& timestamp)
    {
        ::visualization_msgs::Marker msg = createDefaultMarker(default_header, timestamp);
        msg.id = msgs.markers.size();
        msg.type = ::visualization_msgs::Marker::CYLINDER;
        msg.ns = "cylinders";
        
        double radius = 0.01;
        msg.scale.x = msg.scale.y = radius;
        msg.scale.z = length;
        
        msg.pose = createPoseMsg(t);
        
        msg.color.r = color_r;
        msg.color.g = color_g;
        msg.color.b = color_b;
        msg.color.a = 1.0;
        
        msgs.markers.push_back(msg);
    }
    
    void addAxisMarker(::visualization_msgs::MarkerArray& msgs, const geometry_msgs::Transform& t, const ::std_msgs::Header& default_header, const ros::Time& timestamp)
    {
        double length = 0.1;
        tf::Transform pose;
        tf::transformMsgToTF(t, pose);
        tf::Transform x_pose = pose * tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(length / 2.0, 0, 0)) * tf::Transform(tf::Quaternion(tf::Vector3(0, 1, 0), M_PI / 2.0));
        tf::Transform y_pose = pose * tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0, length / 2.0, 0)) * tf::Transform(tf::Quaternion(tf::Vector3(1, 0, 0), M_PI / 2.0));
        tf::Transform z_pose = pose * tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, length / 2.0));
        
        geometry_msgs::Transform x_t, y_t, z_t;
        tf::transformTFToMsg(x_pose, x_t);
        tf::transformTFToMsg(y_pose, y_t);
        tf::transformTFToMsg(z_pose, z_t);
        
        addCylinderMarker(msgs, 1, 0, 0, length, x_t, default_header, timestamp);
        msgs.markers.back().ns = "axes";
        addCylinderMarker(msgs, 0, 1, 0, length, y_t, default_header, timestamp);
        msgs.markers.back().ns = "axes";
        addCylinderMarker(msgs, 0, 0, 1, length, z_t, default_header, timestamp);
        msgs.markers.back().ns = "axes";
    }

    void addArrowMarker(::visualization_msgs::MarkerArray& msgs, const geometry_msgs::Vector3& from, const geometry_msgs::Vector3& to, const ::std_msgs::Header& default_header, const ros::Time& timestamp)
    {
        ::visualization_msgs::Marker msg = createDefaultMarker(default_header, timestamp);
        msg.id = msgs.markers.size();
        msg.type = ::visualization_msgs::Marker::ARROW;
        msg.ns = "arrows";

        msg.scale.x = 0.01; // shaft diameter
        msg.scale.y = 0.02; // head diameter
        msg.scale.z = 0.05; // head length

        ::tf::Pose identity(::tf::Pose::getIdentity());
        ::tf::poseTFToMsg(identity, msg.pose);

        msg.points.push_back(createPointMsg(to));
        msg.points.push_back(createPointMsg(from));

        msg.color.r = 1.0;
        msg.color.g = msg.color.b = 0.0;
        msg.color.a = 1.0;
        
        msgs.markers.push_back(msg);
    }
    
    ::visualization_msgs::Marker createDefaultMarker(const ::std_msgs::Header& default_header, const ros::Time& timestamp)
    {
        ::visualization_msgs::Marker msg;
        msg.id = 0;
        msg.header = default_header;
        msg.header.frame_id = "/camera_rgb_optical_frame";
        msg.header.stamp = timestamp;
        msg.action = ::visualization_msgs::Marker::ADD;
        msg.lifetime = timestamp - last_broadcast_;
        return msg;
    }
    
    void addTextMarker(::visualization_msgs::MarkerArray& msgs, const std::string& text, const geometry_msgs::Vector3& p, const ::std_msgs::Header& default_header, const ros::Time& timestamp)
    {
        ::visualization_msgs::Marker msg = createDefaultMarker(default_header, timestamp);
        msg.id = msgs.markers.size();
        msg.type = ::visualization_msgs::Marker::TEXT_VIEW_FACING;
        msg.ns = "text";
        
        msg.text = text;
        msg.scale.z = 0.05; // Height of an "A"
        
        msg.pose.position = createPointMsg(p);

        msg.color.r = 1.0;
        msg.color.g = msg.color.b = 1.0;
        msg.color.a = 1.0;
        
        msgs.markers.push_back(msg);
    }

    void publishRVizMarkers(const geometry_graph_msgs::GraphConstPtr& graph)
    {
        ::visualization_msgs::MarkerArray msgs;
        
        ros::Time right_now = ros::Time::now();
        // for each node draw an axis and a text
        for (std::vector<geometry_graph_msgs::Node>::const_iterator it = graph->nodes.begin(); it != graph->nodes.end(); ++it)
        {
            std::string text_label = it->label + std::string("_") + boost::lexical_cast<std::string>(static_cast<int>(std::distance(graph->nodes.begin(), it)));
            addTextMarker(msgs, text_label, it->transform.translation, graph->header, right_now);
            addAxisMarker(msgs, it->transform, graph->header, right_now);
        }
        // for each edge draw an arrow
        for (std::vector<geometry_graph_msgs::Edge>::const_iterator it = graph->edges.begin(); it != graph->edges.end(); ++it)
        {
            addArrowMarker(msgs, graph->nodes[it->node_id_start].transform.translation, graph->nodes[it->node_id_end].transform.translation, graph->header, right_now);
        }

        last_broadcast_ = ros::Time::now();
        marker_publisher_.publish(msgs);
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        graph_publisher_.publish(**graph_message_);
        publishRVizMarkers(*graph_message_);

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PublishGeometryGraph, "PublishGeometryGraph", "Publish a graph.")
