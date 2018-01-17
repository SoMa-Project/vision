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

#include <geometry_graph_msgs/Objects.h>

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

    ros::Time last_broadcast_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<std::string>("topic_name", "A string to be used as topic name for found objects.", "/objects").required(false);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<std::vector<UnalignedAffine3f> >(&PublishObjectList::object_poses_, "transforms", "A vector of 4x4 affine transformations for the objects.");
        inputs.declare<std::vector<UnalignedVector3f> >(&PublishObjectList::object_sizes_, "sizes", "A vector of 3d sizes for the bounding boxes.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        object_poses_ = inputs["transforms"];
        object_sizes_ = inputs["sizes"];
        object_publisher_ = nh_.advertise< ::geometry_graph_msgs::Objects>(params["topic_name"]->get<std::string>(), 10);
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        std::vector<UnalignedAffine3f> poses = *object_poses_;
        std::vector<UnalignedVector3f> sizes = *object_sizes_;

        geometry_graph_msgs::Objects obj;


        for(int i=0; i< poses.size(); i++)
        {
            geometry_msgs::Transform ts;
            //tf::transformEigenToMsg((Eigen::Affine3d) transform_->cast<double>(), ts);
            Eigen::Affine3d* ets = (Eigen::Affine3d*)(&(poses[i]));
            tf::transformEigenToMsg(ets->cast<double>(), ts);
            obj.transforms.push_back(ts);

            obj.names.push_back("object");

            geometry_msgs::Vector3 bbvec;
            bbvec.x = sizes[i][0];
            bbvec.y = sizes[i][1];
            bbvec.z = sizes[i][2];
            obj.boundingboxes.push_back(bbvec);
        }

        object_publisher_.publish(obj);
        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PublishObjectList, "PublishObjectList", "Publish a list of found objects.")
