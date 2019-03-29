/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <tinyxml.h>

#include <ecto/ecto.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf_conversions/tf_eigen.h>

#include "Eigen/Core"

#include <pregrasp_msgs/GraspStrategy.h>
#include <pregrasp_msgs/GraspStrategyArray.h>
#include <geometry_graph_msgs/Graph.h>

#include <ecto_rbo_grasping/PoseSet.h>


namespace ecto_rbo_grasping
{

struct CreateGeometryGraph
{
    typedef std::vector< ::pregrasp_msgs::GraspStrategy> Strategies;
    typedef std::vector< std::pair<pregrasp_msgs::GraspStrategy, pregrasp_msgs::GraspStrategy> > StrategyGraph;
    typedef ::Eigen::MatrixXi Edges;

    ros::Time last_broadcast_, last_broadcast2_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> wall_grasps_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> wall_grasp_manifolds_;
    
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> corner_grasps_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> corner_grasp_manifolds_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> surface_grasps_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> surface_grasp_manifolds_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> edge_grasps_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> edge_grasp_manifolds_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> positioning_motions_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> positioning_motion_manifolds_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> landing_motions_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> landing_motion_manifolds_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pushing_motions_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> pushing_motion_manifolds_;

    ecto::spore<double> cartesian_threshold_;
    ecto::spore<double> angular_threshold_;
    ecto::spore<bool> ignore_manifold_intersection_;

    ecto::spore<geometry_graph_msgs::GraphConstPtr> graph_message_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<double>("angular_threshold", "", 0.5);
        params.declare<double>("cartesian_threshold", "", 0.05);
        params.declare<bool>("ignore_manifold_intersection", "wether to ignore the manifolds connectivty check and just connect all grasps to each other", false);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("edge_pregrasp_messages", "All edge grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("surface_pregrasp_messages", "All surface grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("wall_pregrasp_messages", "All wall grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("corner_pregrasp_messages", "All wall grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pushing_pregrasp_messages", "All sliding motions that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("landing_pregrasp_messages", "All caging motions that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("positioning_pregrasp_messages", "All free-space motions that are considered.").required(false);

        inputs.declare< ::posesets::PoseSetArrayConstPtr>("edge_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("surface_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("wall_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("corner_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("pushing_motion_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("landing_motion_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("positioning_motion_manifolds", "All the planar manifolds found.").required(false);

        outputs.declare<geometry_graph_msgs::GraphConstPtr>("graph_message", "A graph representation of the environment.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        wall_grasps_ = inputs["wall_pregrasp_messages"];
        corner_grasps_ = inputs["corner_pregrasp_messages"];
        edge_grasps_ = inputs["edge_pregrasp_messages"];
        surface_grasps_ = inputs["surface_pregrasp_messages"];
        pushing_motions_ = inputs["pushing_pregrasp_messages"];
        landing_motions_ = inputs["landing_pregrasp_messages"];
        positioning_motions_ = inputs["positioning_pregrasp_messages"];

        edge_grasp_manifolds_ = inputs["edge_grasp_manifolds"];
        wall_grasp_manifolds_ = inputs["wall_grasp_manifolds"];
        corner_grasp_manifolds_ = inputs["wall_grasp_manifolds"];
        surface_grasp_manifolds_ = inputs["surface_grasp_manifolds"];
        pushing_motion_manifolds_ = inputs["pushing_motion_manifolds"];
        landing_motion_manifolds_ = inputs["landing_motion_manifolds"];
        positioning_motion_manifolds_ = inputs["positioning_motion_manifolds"];

        angular_threshold_ = params["angular_threshold"];
        cartesian_threshold_ = params["cartesian_threshold"];
        ignore_manifold_intersection_ = params["ignore_manifold_intersection"];

        graph_message_ = outputs["graph_message"];

        last_broadcast_ = last_broadcast2_ = ros::Time::now();
    }

    bool isGoalNode(int strategy_type, int grasp_type)
    {
        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_CORNER_GRASP ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_APPROACH_THEN_SQUEEZE ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE )
            return true;

        return false;
    }
    
    std::string createNodeLabel(const pregrasp_msgs::GraspStrategy& g)
    {
        if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
            return "Positioning"; //"Positionings";
        else if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return "Landing"; //"Landings";
        else if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
            return "Slide"; //"Pushes";
        else if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE)
            return "SurfaceGrasp";
        else if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
            return "WallGrasp"; //"WallGrasps";
        else if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_CORNER_GRASP)
            return "CornerGrasp"; //"CornerGrasp";

        else if (g.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE)
            return "EdgeGrasp"; //"EdgeGrasps";

        return "Unknown";
    }

    bool isConnected(const ::posesets::PoseSet& first_strategy, const ::posesets::PoseSet& second_strategy)
    {
        return first_strategy.isIntersecting(second_strategy);
    }

    bool isConnected(const ::posesets::PoseSet& first_manifold, const ::posesets::PoseSet& second_manifold,
                     const ::pregrasp_msgs::GraspStrategy& first_strategy, const ::pregrasp_msgs::GraspStrategy& second_strategy)
    {
        // if it's the same thing do not consider it to be connected
        if (first_strategy.strategy == second_strategy.strategy)
            return false;
        
        // if the first one is a grasp, we're done
        if (first_strategy.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP || 
            first_strategy.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE || 
            first_strategy.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE)
            return false;

        //Also don't go back to the start node
        if(second_strategy.strategy == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
          return false;

        /*
        if (first_strategy.strategy == pregrasp_msgs::GraspStrategy::STRATEGY_POSITION && second_strategy.strategy != pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return false;
        */
        

        //we completely ignore the manifold connectivity check, if this option is set to true
        if(ignore_manifold_intersection_)
          return true;

        return first_manifold.isIntersecting(second_manifold);
    }

    bool isConnected(const ::pregrasp_msgs::GraspStrategy& first_strategy, const ::pregrasp_msgs::GraspStrategy& second_strategy)
    {
        if (first_strategy.strategy == second_strategy.strategy)
            return false;

        if (first_strategy.strategy == pregrasp_msgs::GraspStrategy::STRATEGY_POSITION && second_strategy.strategy != pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return false;

        // check distance
        tf::Pose p1, p2, o1, o2;
        tf::poseMsgToTF(first_strategy.pregrasp_pose.pose.pose, p1);
        tf::poseMsgToTF(second_strategy.pregrasp_pose.center.pose, p2);
        tf::poseMsgToTF(first_strategy.object.pose.pose, o1);
        tf::poseMsgToTF(second_strategy.object.center.pose, o2);
        double angular_distance = tf::angleShortestPath(p1.getRotation(), p2.getRotation());
        double cartesian_distance = p1.getOrigin().distance(p2.getOrigin());

//        if (second_strategy.pregrasp_pose.size.empty())
//        {
////            ROS_INFO("cartesian dist: %f", cartesian_distance);


//            if (cartesian_distance < (*cartesian_threshold_))
//            {
////                ROS_INFO("angular dist: %f", angular_distance);
//            }

//            return (angular_distance < (*angular_threshold_) && cartesian_distance < (*cartesian_threshold_));
//        }
//        else if (first_strategy.pregrasp_pose.image_size.empty())
//        {
////            ROS_INFO("AHA");
//            return (cartesian_distance < second_strategy.pregrasp_pose.size.at(0) && angular_distance < second_strategy.pregrasp_pose.size.at(1));
//        }

//        ROS_INFO("KUCKUCK %zu %zu %i", first_strategy.pregrasp_pose.image_size.size(), second_strategy.pregrasp_pose.size.size(), second_strategy.strategy);
//        ROS_INFO("Is empty? image_size: %i size: %i", first_strategy.pregrasp_pose.image_size.empty(), second_strategy.pregrasp_pose.size.empty());

        double object_cartesian_distance = o1.getOrigin().distance(o2.getOrigin());
        double object_angular_distance = tf::angleShortestPath(o1.getRotation(), o2.getRotation());

        bool object_inside_preimage = (first_strategy.object.image_size.at(0) <= second_strategy.object.size.at(0)) &&
                                      (first_strategy.object.image_size.at(1) <= second_strategy.object.size.at(1)) &&
                                      (first_strategy.object.image_size.at(2) <= second_strategy.object.size.at(2));

//        std::cout << "'" << second_strategy.pregrasp_pose.size.at(0) << "'";

        // homotopy vs. pre-image?
        return (cartesian_distance < second_strategy.pregrasp_pose.size.at(0) || cartesian_distance < first_strategy.pregrasp_pose.image_size.at(0)) &&
               (angular_distance < second_strategy.pregrasp_pose.size.at(1) || angular_distance < first_strategy.pregrasp_pose.image_size.at(1)) &&
                object_inside_preimage;
    }

    int process(const ecto::tendrils& /*inputs*/, const ecto::tendrils& /*outputs*/)
    {
        // collect all received types of motions in a single vector
        std::vector<pregrasp_msgs::GraspStrategyArrayConstPtr> all_motions;
        std::vector< ::posesets::PoseSetArrayConstPtr> all_manifolds;
        std::vector<std::string> all_identifiers;
        if (positioning_motions_.user_supplied())
        {
            all_identifiers.push_back("positioning motion");
            all_motions.push_back(*positioning_motions_);
            all_manifolds.push_back(*positioning_motion_manifolds_);
        }
        if (landing_motions_.user_supplied())
        {
            all_identifiers.push_back("landing motion");
            all_motions.push_back(*landing_motions_);
            all_manifolds.push_back(*landing_motion_manifolds_);
        }
        if (pushing_motions_.user_supplied())
        {
            all_identifiers.push_back("pushing motion");
            all_motions.push_back(*pushing_motions_);
            all_manifolds.push_back(*pushing_motion_manifolds_);
        }
        if (surface_grasps_.user_supplied())
        {
            all_identifiers.push_back("surface grasp");
            all_motions.push_back(*surface_grasps_);
            all_manifolds.push_back(*surface_grasp_manifolds_);
        }
        if (edge_grasps_.user_supplied())
        {
            all_identifiers.push_back("edge grasp");
            all_motions.push_back(*edge_grasps_);
            all_manifolds.push_back(*edge_grasp_manifolds_);
        }
        if (wall_grasps_.user_supplied())
        {
            all_identifiers.push_back("wall grasp");
            all_motions.push_back(*wall_grasps_);
            all_manifolds.push_back(*wall_grasp_manifolds_);
        }
        
        if (corner_grasps_.user_supplied())
        {
            all_identifiers.push_back("corner grasp");
            all_motions.push_back(*corner_grasps_);
            all_manifolds.push_back(*corner_grasp_manifolds_);
        }

        size_t total = 0;
        ROS_INFO("The geometry graph creator received:");
        for (size_t i = 0; i < all_identifiers.size(); ++i)
        {
            ROS_INFO("%zu %s(s).", all_motions[i]->strategies.size(), all_identifiers[i].c_str());
            total += all_motions[i]->strategies.size();
        }

        if (total == 0)
            return ecto::OK;

        // find all paths in a directed acyclic graph
        StrategyGraph dag;
        Strategies dag_search_front;

        // put all strategies in a nodes vector
        Strategies nodes;
        for (size_t i = 0; i < all_motions.size(); ++i)
            for (Strategies::const_iterator it = all_motions[i]->strategies.begin(); it != all_motions[i]->strategies.end(); ++it)
                nodes.push_back(*it);

        // do the same for the manifolds
        ::posesets::PoseSetArray manifolds;
        for (size_t i = 0; i < all_manifolds.size(); ++i)
            for (::posesets::PoseSetArray::const_iterator it = all_manifolds[i]->begin(); it != all_manifolds[i]->end(); ++it) {
                std::cout << all_identifiers[i].at(0) << " ";
                manifolds.push_back(*it);
            }
        std::cout << std::endl;

        // create the connectivity matrix
        Edges edges(manifolds.size(), manifolds.size());
        for (size_t i = 0; i < manifolds.size(); ++i)
        {
            for (size_t j = 0; j < manifolds.size(); ++j)
            {
                if (i == j || !isConnected(manifolds[i], manifolds[j], nodes[i], nodes[j]))
                {
                    edges(i, j) = 0;
                    std::cout << "- ";
                }
                else
                {
                    edges(i, j) = 1;
                    std::cout << "1 ";
                }
            }
            std::cout << std::endl;
        }
        
        // copy to ROS msg
        geometry_graph_msgs::GraphPtr graph_message(new geometry_graph_msgs::Graph);
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            geometry_graph_msgs::Node node;
            node.transform.translation.x = nodes[i].pregrasp_pose.pose.pose.position.x;
            node.transform.translation.y = nodes[i].pregrasp_pose.pose.pose.position.y;
            node.transform.translation.z = nodes[i].pregrasp_pose.pose.pose.position.z;
            node.transform.rotation = nodes[i].pregrasp_pose.pose.pose.orientation;
            node.label = createNodeLabel(nodes[i]);
            graph_message->nodes.push_back(node);
            
            // TODO: Maybe do this only once
            graph_message->header = nodes[i].pregrasp_pose.pose.header;
        }
        for (size_t i = 0; i < edges.rows(); ++i)
        {
            for (size_t j = 0; j < edges.cols(); ++j)
            {
                if (edges(i, j) > 0)
                {
                    geometry_graph_msgs::Edge edge;
                    edge.node_id_start = i;
                    edge.node_id_end = j;
                    graph_message->edges.push_back(edge);
                }
            }
        }
        (*graph_message_) = graph_message;

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::CreateGeometryGraph, "CreateGeometryGraph", "Combine grasps and non-prehensile actions into a graph.")
