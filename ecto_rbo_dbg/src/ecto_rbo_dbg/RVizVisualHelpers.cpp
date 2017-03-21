/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ros/ros.h>

#include <ecto/ecto.hpp>
#include <ecto/registry.hpp>
#include <ecto_pcl/ecto_pcl.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategy.h>
#include <pregrasp_msgs/GraspStrategyArray.h>

#include <eigen_conversions/eigen_msg.h>

namespace ecto_rbo_dbg
{

typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

typedef boost::shared_ptr< ecto::pcl::ModelCoefficients const> ModelCoefficientsConstPtr;

int addSphereMarker(visualization_msgs::MarkerArrayPtr& msgs,
                     const std::string& frame_id,
                     const std::string& ns,
                     int id,
                     const std::vector<double>& color,
                     double scale,
                     const Eigen::Vector3f& position)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    //marker.header.stamp = ros::Time();
    //marker.lifetime = ros::Duration(0.2);
    marker.id = id++;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);

    marker.scale.x = marker.scale.y = marker.scale.z = scale;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    msgs->markers.push_back(marker);

    return id;
}

int addCylinderMarker(visualization_msgs::MarkerArrayPtr& msgs,
                     const std::string& frame_id,
                     const std::string& ns,
                     int id,
                     const std::vector<double>& color,
                     double length,
                     double radius,
                     const Eigen::Affine3f& pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    //marker.header.stamp = ros::Time();
    //marker.lifetime = ros::Duration(0.2);
    marker.id = id++;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    tf::poseEigenToMsg(pose.cast<double>(), marker.pose);

    marker.scale.x = marker.scale.y = radius;
    marker.scale.z = length;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    msgs->markers.push_back(marker);

    return id;
}

int addArrowMarker(visualization_msgs::MarkerArrayPtr& msgs,
                      const std::string& frame_id,
                      const std::string& ns,
                      int id,
                      const std::vector<double>& color,
                      double shaft_diameter,
                      double head_diameter,
                      const Eigen::Vector3f& point_start,
                      const Eigen::Vector3f& point_end)
 {
     visualization_msgs::Marker marker;
     marker.header.frame_id = frame_id;
     //marker.header.stamp = ros::Time();
     //marker.lifetime = ros::Duration(0.2);
     marker.id = id++;
     marker.ns = ns;
     marker.type = visualization_msgs::Marker::ARROW;
     marker.action = visualization_msgs::Marker::ADD;

     marker.points.resize(2);
     tf::pointEigenToMsg(point_start.cast<double>(), marker.points[0]);
     tf::pointEigenToMsg(point_end.cast<double>(), marker.points[1]);

     marker.scale.x = shaft_diameter;
     marker.scale.y = head_diameter;
     marker.scale.z = 0;

     marker.color.r = color[0];
     marker.color.g = color[1];
     marker.color.b = color[2];
     marker.color.a = color[3];

     msgs->markers.push_back(marker);

     return id;
 }

int addArrowMarker(visualization_msgs::MarkerArrayPtr& msgs,
                      const std::string& frame_id,
                      const std::string& ns,
                      int id,
                      const std::vector<double>& color,
                      double shaft_diameter,
                      double head_diameter,
                      const Eigen::Vector3f& point)
 {
     return addArrowMarker(msgs, frame_id, ns, id, color, shaft_diameter, head_diameter, Eigen::Vector3f::Zero(), point);
 }

int addLineStripMarker(visualization_msgs::MarkerArrayPtr& msgs,
                      const std::string& frame_id,
                      const std::string& ns,
                      int id,
                      const std::vector<double>& color,
                      double line_width,
                      const std::vector<Eigen::Vector3f>& points,
                      bool closed_line = true)
 {
     visualization_msgs::Marker marker;
     marker.header.frame_id = frame_id;
     //marker.header.stamp = ros::Time();
     //marker.lifetime = ros::Duration(0.2);
     marker.id = id++;
     marker.ns = ns;
     marker.type = visualization_msgs::Marker::LINE_STRIP;
     marker.action = visualization_msgs::Marker::ADD;

     marker.points.resize(points.size() + closed_line);
     for (std::vector<Eigen::Vector3f>::const_iterator it = points.begin(); it != points.end(); ++it)
         tf::pointEigenToMsg(it->cast<double>(), marker.points[std::distance(points.begin(), it)]);
     if (closed_line)
         tf::pointEigenToMsg(points.front().cast<double>(), marker.points[marker.points.size() - 1]);

     marker.scale.x = line_width;

     marker.color.r = color[0];
     marker.color.g = color[1];
     marker.color.b = color[2];
     marker.color.a = color[3];

     msgs->markers.push_back(marker);

     return id;
 }

int addCubeMarker(visualization_msgs::MarkerArrayPtr& msgs,
                     const std::string& frame_id,
                     const std::string& ns,
                     int id,
                     const std::vector<double>& color,
                     const std::vector<double>& scale,
                     const Eigen::Affine3f& pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    //marker.header.stamp = ros::Time();
    //marker.lifetime = ros::Duration(0.2);
    marker.id = id++;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    tf::poseEigenToMsg(pose.cast<double>(), marker.pose);

    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    msgs->markers.push_back(marker);

    return id;
}

int addAxisMarker(visualization_msgs::MarkerArrayPtr& msgs,
                     const std::string& frame_id,
                     const std::string& ns,
                     int id,
                     double length,
                     double radius,
                     const Eigen::Affine3f& pose)
{
    Eigen::Affine3f x_pose = pose * (Eigen::Translation3f(length / 2.0, 0, 0) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY()));
    Eigen::Affine3f y_pose = pose * (Eigen::Translation3f(0, length / 2.0, 0) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f z_pose = pose * (Eigen::Translation3f(0, 0, length / 2.0) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));

    addCylinderMarker(msgs, frame_id, ns, id++, {1, 0, 0, 1}, length, radius, x_pose);
    addCylinderMarker(msgs, frame_id, ns, id++, {0, 1, 0, 1}, length, radius, y_pose);
    addCylinderMarker(msgs, frame_id, ns, id++, {0, 0, 1, 1}, length, radius, z_pose);

    return id;
}


template <typename T>
int publish(const T& data, visualization_msgs::MarkerArrayConstPtr& msg, const std::string& ns)
{
    std::cerr << "RVizPublisher: No publisher defined for this data type!" << std::endl;
    return ecto::QUIT;
}

int publish(const std::vector<UnalignedVector4f>& data, visualization_msgs::MarkerArrayPtr& msg, const std::string& ns)
{
    for (size_t i = 0; i < data.size(); ++i)
    {
        addSphereMarker(msg, "base_frame", ns, i, {1.0, 0, 0, 1.0}, 0.05, data[i].head<3>());
    }
    return ecto::OK;
}

int publish(const std::vector<UnalignedVector3f>& data, visualization_msgs::MarkerArrayPtr& msg, const std::string& ns)
{
    for (size_t i = 0; i < data.size(); ++i)
    {
        addArrowMarker(msg, "base_frame", ns, i, {1.0, 0, 0, 1.0}, 0.02, 0.05, data[i]);
    }
    return ecto::OK;
}

int publish(const UnalignedAffine3f& data, visualization_msgs::MarkerArrayPtr& msg, const std::string& ns)
{
    addAxisMarker(msg, "base_frame", ns, 0, 0.1, 0.01, data);
    return ecto::OK;
}

int publish(const std::vector<UnalignedAffine3f>& data, visualization_msgs::MarkerArrayPtr& msg, const std::string& ns)
{
    int id = 0;
    for (size_t i = 0; i < data.size(); ++i)
    {
        id = addAxisMarker(msg, "base_frame", ns, id, 0.1, 0.01, data[i]);
    }
    return ecto::OK;
}

int publish(const std::vector< ModelCoefficientsConstPtr>& data, visualization_msgs::MarkerArrayPtr& msg, const std::string& ns)
{
    int id = 0;
    for (size_t i = 0; i < data.size(); ++i)
    {
        if (data[i]->values.size() == 4)
        {
            // generate transform from plane equation
            Eigen::Vector3f normal(data[i]->values[0], data[i]->values[1], data[i]->values[2]);
            Eigen::Quaternionf rot(Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), normal));
            Eigen::Affine3f pose;
            pose.linear() = rot.toRotationMatrix();
            pose.translation() = normal * (-data[i]->values[3]);

            addCylinderMarker(msg, "base_frame", ns, i, {0, 0, 1, 1}, 0.01, 1.15, pose);
        }
        else if (data[i]->values.size() == 10)
        {
            // generate transform from plane equation
            Eigen::Vector3f center(data[i]->values[0], data[i]->values[1], data[i]->values[2]);
            Eigen::Vector3f normal(data[i]->values[3], data[i]->values[4], data[i]->values[5]);
            Eigen::Vector3f longest_axis(data[i]->values[6], data[i]->values[7], data[i]->values[8]);
            double height = longest_axis.norm();
            double width = data[i]->values[9];
            longest_axis.normalize();
            Eigen::Matrix3f rot;
            rot << longest_axis, normal.cross(longest_axis), normal;
            Eigen::Affine3f pose;
            pose.linear() = rot;
            pose.translation() = center;

            addCubeMarker(msg, "base_frame", ns, i, {0, 1, 0, 1}, {height, width, 0.01}, pose);
        }
        else if ((data[i]->values.size() % 3) == 0)
        {
            // it's a polygon
            std::vector<Eigen::Vector3f> tmp;
            for (size_t j = 0; j < data[i]->values.size() / 3; ++j)
                tmp.push_back(Eigen::Vector3f(data[i]->values[j*3], data[i]->values[j*3 + 1], data[i]->values[j*3 + 2]));
            addLineStripMarker(msg, "base_frame", ns, i, {0, 0, 1, 1}, 0.01, tmp);
        }
    }
    return ecto::OK;
}

int publish(const pregrasp_msgs::GraspStrategyArrayConstPtr& data, visualization_msgs::MarkerArrayPtr& msg, const std::string& ns)
{
    int id = 0;
    for (std::vector<pregrasp_msgs::GraspStrategy>::const_iterator it = data->strategies.begin(); it != data->strategies.end(); ++it)
    {
        Eigen::Affine3d tmp;
        tf::poseMsgToEigen(it->pregrasp_pose.pose.pose, tmp);
        id = addAxisMarker(msg, "base_frame", ns, id, 0.1, 0.01, tmp.cast<float>());
        Eigen::Vector3d start, end;
        tf::pointMsgToEigen(it->pregrasp_pose.pose.pose.position, start);
        tf::pointMsgToEigen(it->object.pose.pose.position, end);
        id = addArrowMarker(msg, "base_frame", ns, id, {1.0, 0, 1.0, 1.0}, 0.005, 0.01, start.cast<float>(), end.cast<float>());
    }
    return ecto::OK;
}

template <typename T>
struct RVizMessageConverter
{
    ecto::spore< T > input_;
    ecto::spore<visualization_msgs::MarkerArrayConstPtr> output_;
    ecto::spore<std::string> namespace_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare< std::string >("namespace", "namespace of markers");
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare< T >("input", "Input to be send as RViz markers.");
        outputs.declare<visualization_msgs::MarkerArrayConstPtr>("output", "Messages to be published to RViz.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        namespace_ = params["namespace"];
        input_ = inputs["input"];
        output_ = outputs["output"];
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        int return_value = ecto::OK;

        visualization_msgs::MarkerArrayPtr tmp(new visualization_msgs::MarkerArray);

        if (input_)
            return_value = publish(*input_, tmp, *namespace_);

        (*output_) = tmp;

        return return_value;
    }
};


struct RVizMarkerPublisher
{
    static const int max_input_messages = 50;

    std::vector< ecto::spore<visualization_msgs::MarkerArrayConstPtr> > rviz_markers_;
    ecto::spore<std_msgs::Header> header_;
    
    ros::Publisher publisher_;

    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<std_msgs::Header>("header", "The header whose frame_id is used.");
        inputs.declare<visualization_msgs::MarkerArrayConstPtr>("input_0", "").required(true);
        for (int i = 1; i < max_input_messages; ++i)
        {
            std::string name = "input_" + boost::lexical_cast<std::string>(i);
            inputs.declare<visualization_msgs::MarkerArrayConstPtr>(name, "").required(false);
        }
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
		header_ = inputs["header"];
        rviz_markers_.resize(max_input_messages);
        for (int i = 0; i < max_input_messages; ++i)
        {
            std::string name = "input_" + boost::lexical_cast<std::string>(i);
            rviz_markers_[i] = inputs[name];
        }

        ros::NodeHandle nh("~");
        bool latched = true;
        publisher_ = nh.advertise<visualization_msgs::MarkerArray>("ecto_markers", 10, latched);
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        visualization_msgs::MarkerArray markers;

        for (size_t i = 0; i < rviz_markers_.size(); ++i)
        {
            if (rviz_markers_[i].user_supplied())
                markers.markers.insert(markers.markers.end(), (*rviz_markers_[i])->markers.begin(), (*rviz_markers_[i])->markers.end());
            else
                break;
        }
        
        if (header_.user_supplied())
        {
			const std::string& frame = header_->frame_id;
			
			for (size_t i = 0; i < markers.markers.size(); ++i)
			{
				markers.markers[i].header.frame_id = frame;
			}
		}

        publisher_.publish(markers);

        return ecto::OK;
    }
};
}

ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMarkerPublisher, "RVizMarkerPublisher", "Publishes marker array messages.");

ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMessageConverter< std::vector<ecto_rbo_dbg::UnalignedVector4f> >, "ecto_rbo_dbg::RVizMessageConverter<std::vector<ecto_rbo_dbg::UnalignedVector4f>>", "Publish markers.");
ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMessageConverter< std::vector<ecto_rbo_dbg::UnalignedVector3f> >, "ecto_rbo_dbg::RVizMessageConverter<std::vector<ecto_rbo_dbg::UnalignedVector3f>>", "Publish markers.");
ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMessageConverter< ecto_rbo_dbg::UnalignedAffine3f >, "ecto_rbo_dbg::RVizMessageConverter<ecto_rbo_dbg::UnalignedAffine3f>", "Publish markers.");
ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMessageConverter< std::vector<ecto_rbo_dbg::UnalignedAffine3f> >, "ecto_rbo_dbg::RVizMessageConverter<std::vector<ecto_rbo_dbg::UnalignedAffine3f>>", "Publish markers.");
ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMessageConverter< std::vector<ecto_rbo_dbg::ModelCoefficientsConstPtr> >, "ecto_rbo_dbg::RVizMessageConverter<std::vector<ecto::pcl::ModelCoefficientsConstPtr>>", "Publish markers.");
ECTO_CELL(ecto_rbo_dbg, ecto_rbo_dbg::RVizMessageConverter< pregrasp_msgs::GraspStrategyArrayConstPtr >, "ecto_rbo_dbg::RVizMessageConverter< pregrasp_msgs::GraspStrategyArrayConstPtr >", "Publish markers.");
