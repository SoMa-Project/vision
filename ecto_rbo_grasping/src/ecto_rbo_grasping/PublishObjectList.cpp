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
#include <eigen_conversions/eigen_msg.h>

#include <geometry_graph_msgs/Object.h>
#include <geometry_graph_msgs/ObjectList.h>

#include "object_segmentation/Segment.h"

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

struct PublishObjectList
{
    ros::NodeHandle nh_;
    ros::Publisher object_publisher_;

    ecto::spore<std::vector<UnalignedAffine3f> > object_poses_;
    ecto::spore<std::vector<UnalignedVector3f> > object_sizes_;
    ecto::spore<std::vector<object_segmentation::Segment> > segments_;

    std::string frame_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<std::string>("topic_name", "A string to be used as topic name for found objects.", "/objects").required(false);
        params.declare<std::string>("frame", "The source frame to listen to.", "camera_rgb_optical_frame").required(false);

    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<std::vector<UnalignedAffine3f> >(&PublishObjectList::object_poses_, "transforms", "A vector of 4x4 affine transformations for the objects.");
        inputs.declare<std::vector<UnalignedVector3f> >(&PublishObjectList::object_sizes_, "sizes", "A vector of 3d sizes for the bounding boxes.");
        inputs.declare<std::vector<object_segmentation::Segment> >(&PublishObjectList::segments_, "segments", "A vector of segments.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        object_poses_ = inputs["transforms"];
        object_sizes_ = inputs["sizes"];
        segments_ = inputs["segments"];
        object_publisher_ = nh_.advertise< ::geometry_graph_msgs::ObjectList>(params["topic_name"]->get<std::string>(), 10);
        frame_ = params["frame"]->get<std::string>();
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        std::vector<UnalignedAffine3f> poses = *object_poses_;
        std::vector<UnalignedVector3f> sizes = *object_sizes_;
        std::vector<object_segmentation::Segment> segments = *segments_;


        geometry_graph_msgs::ObjectList obj_list;
        for(int i=0; i< poses.size(); i++)
        {
            geometry_graph_msgs::Object obj;

            geometry_msgs::PoseStamped ts;
            geometry_msgs::Pose tr;
            tf::poseEigenToMsg((Eigen::Affine3d) (poses[i]).cast<double>(), tr);
            ts.pose = tr;
            std::stringstream ss;
            //ss<<"object_"<<i;
            //ts.child_frame_id = ss.str(); //Todo
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id = frame_;

            obj.transform = ts;

            geometry_msgs::Vector3 bbvec;
            bbvec.x = sizes[i][0];
            bbvec.y = sizes[i][1];
            bbvec.z = sizes[i][2];
            obj.boundingbox = bbvec;
            for(int j=0; j<segments[i].segment_points.size();j++){
                obj.segment_points.push_back(segments[i].segment_points[j]);
            }
            
            obj_list.objects.push_back(obj);
        }

        object_publisher_.publish(obj_list);
        ROS_INFO("PublishObjectList exited with OK");
        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PublishObjectList, "PublishObjectList", "Publish a list of found objects.")
