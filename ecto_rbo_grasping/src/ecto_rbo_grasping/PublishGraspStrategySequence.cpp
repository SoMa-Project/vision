#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <pregrasp_msgs/GraspStrategySequence.h>
#include <ecto_rbo_grasping/PoseSet.h>

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;

struct PublishGraspStrategySequence
{
    ros::NodeHandle nh_;
    ros::Publisher sequence_publisher_;

    ecto::spore< ::posesets::PoseSetArrayConstPtr> manifolds_;
    ecto::spore< ::std::string> topic_name_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<std::string>("topic_name", "Name of the topic the markers are published to.", "grasp_strategy_sequences");
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("manifolds", "All the manifolds that should be published as markers.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        manifolds_ = inputs["manifolds"];

        topic_name_ = params["topic_name"];

        sequence_publisher_ = nh_.advertise< ::pregrasp_msgs::GraspStrategySequence>(*topic_name_, 10);
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        ::pregrasp_msgs::GraspStrategySequence sequence;

        int cnt = 0;
        for (posesets::PoseSetArray::const_iterator it = (*manifolds_)->begin(); it != (*manifolds_)->end(); ++it)
        {
            ::pregrasp_msgs::Milestone m;

            // turn PoseSet into message
            tf::poseTFToMsg(it->getOrigin(), m.motion_constraint.position);
            tf::vector3TFToMsg(it->getPositions(), m.motion_constraint.position_size);
//            it->getOrientatio
//            m.motion_constraint.orientations

            // wrench missing!

            // now add termination predicate as intersection to next pose set
            if (cnt++ < (*manifolds_)->size())
            {
                posesets::PoseSet p(*it);
                p.intersect((*manifolds_)->at(cnt));

                tf::poseTFToMsg(p.getOrigin(), m.termination_predicate.position);
                tf::vector3TFToMsg(p.getPositions(), m.termination_predicate.position_size);
            }

            sequence.milestones.push_back(m);
        }


        sequence_publisher_.publish(sequence);

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PublishGraspStrategySequence, "PublishGraspStrategySequence", "Publish manifolds as markers.")
