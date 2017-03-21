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

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>


namespace ecto_rbo_grasping
{

struct PlanGrasps
{
    typedef std::vector< ::pregrasp_msgs::GraspStrategy> Strategies;
    typedef std::vector< std::pair<pregrasp_msgs::GraspStrategy, pregrasp_msgs::GraspStrategy> > StrategyGraph;
//        typedef std::vector< std::pair<int, int> > Edges;
    typedef ::Eigen::MatrixXi Edges;
    typedef std::vector<int> Path;
    typedef std::vector< Path > Paths;

    ros::Time last_broadcast_, last_broadcast2_;

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> wall_grasps_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> wall_grasp_manifolds_;

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

    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<double>("angular_threshold", "", 0.5);
        params.declare<double>("cartesian_threshold", "", 0.05);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("edge_pregrasp_messages", "All edge grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("surface_pregrasp_messages", "All surface grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("wall_pregrasp_messages", "All wall grasps that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pushing_pregrasp_messages", "All sliding motions that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("landing_pregrasp_messages", "All caging motions that are considered.").required(false);
        inputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("positioning_pregrasp_messages", "All free-space motions that are considered.").required(false);

        inputs.declare< ::posesets::PoseSetArrayConstPtr>("edge_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("surface_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("wall_grasp_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("pushing_motion_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("landing_motion_manifolds", "All the planar manifolds found.").required(false);
        inputs.declare< ::posesets::PoseSetArrayConstPtr>("positioning_motion_manifolds", "All the planar manifolds found.").required(false);

        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "All the grasps that should be used.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        wall_grasps_ = inputs["wall_pregrasp_messages"];
        edge_grasps_ = inputs["edge_pregrasp_messages"];
        surface_grasps_ = inputs["surface_pregrasp_messages"];
        pushing_motions_ = inputs["pushing_pregrasp_messages"];
        landing_motions_ = inputs["landing_pregrasp_messages"];
        positioning_motions_ = inputs["positioning_pregrasp_messages"];

        edge_grasp_manifolds_ = inputs["edge_grasp_manifolds"];
        wall_grasp_manifolds_ = inputs["wall_grasp_manifolds"];
        surface_grasp_manifolds_ = inputs["surface_grasp_manifolds"];
        pushing_motion_manifolds_ = inputs["pushing_motion_manifolds"];
        landing_motion_manifolds_ = inputs["landing_motion_manifolds"];
        positioning_motion_manifolds_ = inputs["positioning_motion_manifolds"];

        angular_threshold_ = params["angular_threshold"];
        cartesian_threshold_ = params["cartesian_threshold"];

        pregrasp_messages_ = outputs["pregrasp_messages"];

        last_broadcast_ = last_broadcast2_ = ros::Time::now();
    }

    void createGraspMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.type = ::visualization_msgs::Marker::MESH_RESOURCE;
        msg.ns = "Grasps";

        // hydro changed the scaling; 0.01 makes the grasps invisible
        msg.scale.x = 1.;
        msg.scale.y = 1.;
        msg.scale.z = 1.;

        tf::Point a, b;
        tf::pointMsgToTF(grasp.pregrasp_pose.center.pose.position, a);
        tf::pointMsgToTF(grasp.pregrasp_pose.pose.pose.position, b);
        tf::pointTFToMsg(0.5 * (a + b), msg.pose.position);
        msg.pose.orientation = grasp.pregrasp_pose.pose.pose.orientation;

        switch (grasp.pregrasp_configuration)
        {
        case pregrasp_msgs::GraspStrategy::PREGRASP_CYLINDER:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.obj";
            msg.color.r = 1.0;
            msg.color.g = msg.color.b = 0;
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_BOX:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_cylindrical.obj";
            msg.color.g = 1.0;
            msg.color.r = msg.color.b = 0;
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_SPHERE:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.obj";
            msg.color.b = msg.color.r = 1.0;
            msg.color.g = 0;
        case pregrasp_msgs::GraspStrategy::PREGRASP_DISK:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_spherical.obj";
            msg.color.b = 1.0;
            msg.color.r = msg.color.g = 0;
            break;
        case pregrasp_msgs::GraspStrategy::PREGRASP_HOOK:
            msg.mesh_resource = "package://barrett_hand_262_msgs/data/barrett_hook.obj";
            msg.color.b = 1.0;
            msg.color.r = msg.color.g = 0;
            break;
        }

        if (grasp.strategy == pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
        {
            msg.color.g = 1.0;
            msg.color.r = msg.color.b = 0.0;
        }
        else if (grasp.strategy == pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
        {
            msg.color.r = msg.color.b = 1.0;
            msg.color.g = 0.0;
        }
    }

    void scaleGeometryMsgVector3(::geometry_msgs::Vector3& v, double factor)
    {
        v.x *= factor;
        v.y *= factor;
        v.z *= factor;
    }

    void createArrowMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.type = ::visualization_msgs::Marker::ARROW;

        msg.ns = "arrows";

        msg.scale.x = 0.01; // shaft diameter
        msg.scale.y = 0.02; // head diameter
        msg.scale.z = 0.05; // head length

        tf::poseTFToMsg(tf::Pose::getIdentity(), msg.pose);

        msg.points.push_back(grasp.pregrasp_pose.center.pose.position);
        msg.points.push_back(grasp.pregrasp_pose.pose.pose.position);

        switch (grasp.strategy)
        {
        case pregrasp_msgs::GraspStrategy::STRATEGY_PUSH:
            msg.color.g = 1.0;
            msg.color.r = msg.color.b = 0;
            break;
        case pregrasp_msgs::GraspStrategy::STRATEGY_LAND:
            msg.color.b = 1.0;
            msg.color.r = msg.color.g = 0;
            break;
        case pregrasp_msgs::GraspStrategy::STRATEGY_POSITION:
            msg.color.b = 1.0;
            msg.color.r = msg.color.g = 0;
            break;
        }

        msg.color.a = 1.0;
    }

    void createPreImageMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.type = ::visualization_msgs::Marker::CUBE;

        msg.ns = "preimages";

        msg.color.r = 0.0;
        msg.color.g = 1.0;
        msg.color.b = 0.0;
        msg.color.a = 0.5;

        if (grasp.pregrasp_pose.size.empty())
        {
            msg.scale.x = msg.scale.y = msg.scale.z = 0.1;
        }
        else
        {
            msg.scale.x = 2 * grasp.pregrasp_pose.size[0];
            msg.scale.y = 2 * grasp.pregrasp_pose.size[0];
            msg.scale.z = 2 * grasp.pregrasp_pose.size[0];
        }

        msg.pose = grasp.pregrasp_pose.center.pose;
    }

    void createImageMarker(::visualization_msgs::Marker& msg, const pregrasp_msgs::GraspStrategy& grasp)
    {
        msg.type = ::visualization_msgs::Marker::CUBE;

        msg.ns = "images";

        msg.color.r = 0.0;
        msg.color.g = 0.0;
        msg.color.b = 1.0;
        msg.color.a = 0.5;

        if (grasp.pregrasp_pose.size.empty())
        {
            msg.scale.x = msg.scale.y = msg.scale.z = 0.1;
        }
        else
        {
            msg.scale.x = 2 * grasp.pregrasp_pose.image_size[0];
            msg.scale.y = 2 * grasp.pregrasp_pose.image_size[0];
            msg.scale.z = 2 * grasp.pregrasp_pose.image_size[0];
        }

        msg.pose = grasp.pregrasp_pose.pose.pose;
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

    bool isPartOfSolution(int edge_start, int edge_end, const Paths& solution_paths)
    {
        for (size_t i = 0; i < solution_paths.size(); ++i)
        {
            for (size_t j = 0; j < solution_paths[i].size() - 1; ++j)
            {
                if (solution_paths[i][j] == edge_start && solution_paths[i][j+1] == edge_end)
                    return true;
            }
        }
        return false;
    }

    bool isPartOfSolution(int node_index, const Paths& solution_paths)
    {
        for (size_t i = 0; i < solution_paths.size(); ++i)
        {
            for (size_t j = 0; j < solution_paths[i].size(); ++j)
            {
                if (solution_paths[i][j] == node_index)
                    return true;
            }
        }
        return false;
    }

    bool isGoalNode(int strategy_type, int grasp_type)
    {
        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_APPROACH_THEN_SQUEEZE ||
            strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE )
            return true;

        return false;
    }

    std::string createInt(int strategy_type, int grasp_type)
    {
        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
            return "1";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return "2";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
            return "3";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
            return "4";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SQUEEZE)
            return "5";

        return "7";
    }

    std::string createRGBHexString(int strategy_type, int grasp_type)
    {
        ::std_msgs::ColorRGBA color = createColor(strategy_type, grasp_type);

        const unsigned red = 255.0 * color.r;
        const unsigned green = 255.0 * color.g;
        const unsigned blue = 255.0 * color.b;
        char hexcol[16];

        snprintf(hexcol, sizeof hexcol, "%02x%02x%02x", red, green, blue);

        return std::string("#") + std::string(hexcol);

//        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
//            return "#339933";
//        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
//            return "#A200FF";
//        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
//            return "#F09609";
//        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
//            return "#FF0097";
//        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE)
//            return "#1BA1E2";
//        return "#A05000";
    }


    std::string createString(int strategy_type, int grasp_type)
    {
        if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_POSITION)
            return "P"; //"Positionings";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return "L"; //"Landings";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_PUSH)
            return "S"; //"Pushes";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_WALL_GRASP)
            return "WG"; //"WallGrasps";
        else if (strategy_type == ::pregrasp_msgs::GraspStrategy::STRATEGY_SLIDE_TO_EDGE)
            return "EG"; //"EdgeGrasps";

        return "Unknown";
    }

    void publishRVizMarkers2(const Strategies& nodes, const Paths& paths, const std::string& frame_id)
    {
        // TODO
        static ros::NodeHandle nh_;
        static ros::Publisher marker_publisher2 = nh_.advertise< ::visualization_msgs::MarkerArray>("/grasp_tree", 10);
        ::visualization_msgs::MarkerArray msgs;

        int id = 0;

        for (Paths::const_iterator it = paths.begin(); it != paths.end(); ++it)
        {
            ::visualization_msgs::Marker msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = ros::Time::now();
            msg.action = ::visualization_msgs::Marker::ADD;
            msg.type = ::visualization_msgs::Marker::LINE_STRIP;
            msg.lifetime = ros::Time::now() - last_broadcast2_;
            msg.scale.x = 0.01;
            msg.id = id++;

            msg.ns = "plan_" + boost::lexical_cast<std::string>(id);

            for (Path::const_iterator jt = it->begin(); jt != it->end(); ++jt)
            {
                const ::pregrasp_msgs::GraspStrategy& g = nodes[*jt];
                msg.points.push_back(g.pregrasp_pose.center.pose.position);
                msg.points.push_back(g.pregrasp_pose.pose.pose.position);

                msg.colors.push_back(createColor(g.strategy, g.pregrasp_configuration));
                msg.colors.push_back(createColor(g.strategy, g.pregrasp_configuration));
            }
            msgs.markers.push_back(msg);

            // if it's a goal node add hand model
            const ::pregrasp_msgs::GraspStrategy& g = nodes[it->back()];
            if (isGoalNode(g.strategy, g.pregrasp_configuration))
            {
                createGraspMarker(msg, g);
                msg.ns = "plan_" + boost::lexical_cast<std::string>(id) + "_icon";
                msg.color = createColor(g.strategy, g.pregrasp_configuration);
                scaleGeometryMsgVector3(msg.scale, 0.5);
                msg.id = id++;

                msgs.markers.push_back(msg);
            }
        }

        last_broadcast2_ = ::ros::Time::now();
        marker_publisher2.publish(msgs);
    }

    void publishRVizMarkers(const Strategies& nodes, const std::string& frame_id, const std::string& topic_name)
    {
        static ros::NodeHandle nh_;
        static ros::Publisher marker_publisher = nh_.advertise< ::visualization_msgs::MarkerArray>(topic_name, 10);
        ::visualization_msgs::MarkerArray msgs;

        int id = 0;
        for (Strategies::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
        {
            ::visualization_msgs::Marker msg;
            msg.header.frame_id = frame_id;
            msg.header.stamp = ros::Time::now();
            msg.action = ::visualization_msgs::Marker::ADD;
            msg.lifetime = ros::Time::now() - last_broadcast_;
            msg.id = id++;

            // draw pre-image
            msg.id = id++;
            createPreImageMarker(msg, *it);
            msgs.markers.push_back(msg);

            // draw image
            msg.id = id++;
            createImageMarker(msg, *it);
            msgs.markers.push_back(msg);

            // draw arrow
            msg.id = id++;
            createArrowMarker(msg, *it);
            msgs.markers.push_back(msg);

            // draw grasp symbol
            msg.id = id++;
            createGraspMarker(msg, *it);
            msgs.markers.push_back(msg);

            // add a bobble for both actions
//            msg.id = id++;
//            msg.type = ::visualization_msgs::Marker::SPHERE;
//            msg.scale.x = msg.scale.y = msg.scale.z = 0.05;
//            msg.pose.position = first_action;
//            msgs.markers.push_back(msg);

//            msg.id = id++;
//            msg.pose.position = second_action;
//            msgs.markers.push_back(msg);
        }

        last_broadcast_ = ::ros::Time::now();
        marker_publisher.publish(msgs);
    }

    void createHAXML(const Strategies& nodes, const Paths& final_solutions, const std::string& file_name_prefix)
    {
        // iterate over all solution paths
        for (size_t i = 0; i < final_solutions.size(); ++i)
        {
            // write hybrid automaton definition for each path
            std::string file_name = file_name_prefix + boost::lexical_cast<std::string>(i) + std::string(".xml");
            TiXmlDocument doc;
            
            TiXmlElement * root = new TiXmlElement("HybridAutomaton");
            root->SetAttribute("current_control_mode", "mode_0");
            doc.LinkEndChild( root );
            
            // add starting mode
            TiXmlElement * initial_mode = new TiXmlElement("ControlMode");
            initial_mode->SetAttribute("name", "mode_0");
            root->LinkEndChild(initial_mode);
            TiXmlElement * initial_set = new TiXmlElement("ControlSet");
            initial_set->SetAttribute("name", "");
            initial_set->SetAttribute("type", "rxControlSet");
            initial_mode->LinkEndChild(initial_set);
            
            for (size_t j = 0; j < final_solutions[i].size(); ++j)
            {
                // add control switch
                //<ControlSwitch name="to_stretch" source="Go_stretch" target="Move_joints"><JumpCondition controller="stretch_ctrl" jump_criterion="0" epsilon="0.5" goal_is_relative="0"><Sensor type="JointConfigurationSensor" /></JumpCondition></ControlSwitch>
                TiXmlElement * cswitch = new TiXmlElement("ControlSwitch");
                cswitch->SetAttribute("source", std::string("mode_") + boost::lexical_cast<std::string>(j));
                cswitch->SetAttribute("target", std::string("mode_") + boost::lexical_cast<std::string>(j+1));
                root->LinkEndChild(cswitch);
                
                TiXmlElement * jump = new TiXmlElement("JumpCondition");
                //jump->SetAttribute("controller", "test_controller");
                cswitch->LinkEndChild(jump);
                
                TiXmlElement * sensor = new TiXmlElement("Sensor");
                sensor->SetAttribute("type", "JointConfigurationSensor");
                jump->LinkEndChild(sensor);
                
                // add control mode
                TiXmlElement * mode = new TiXmlElement("ControlMode");
                mode->SetAttribute("name", std::string("mode_") + boost::lexical_cast<std::string>(j+1));
                root->LinkEndChild(mode);
                
                TiXmlElement * set = new TiXmlElement("ControlSet");
                set->SetAttribute("name", "");
                set->SetAttribute("type", "rxControlSet");
                mode->LinkEndChild(set);
                
                //<Controller type="InterpolatedJointController" name="stretch_ctrl" goal="[7,1]0;0;0;0;0;0;0" goal_is_relative="0" kp="[7,1]300;200;150;20;10;10;10" kv="[7,1]2;4;2;0.8;0.2;0.2;0.02" completion_times="[1,1]10" v_max="[0,0]" a_max="[0,0]" priority="0" interpolation_type="linear" />
                TiXmlElement * cntrl = new TiXmlElement("Controller");
                cntrl->SetAttribute("type", "InterpolatedJointController");
                cntrl->SetAttribute("goal", "");
                set->LinkEndChild(cntrl);
            }
            
            doc.SaveFile(file_name.c_str());
        }
    }

    void createDotFile(const Strategies& nodes, const Edges& edges, const Paths& final_solutions, const std::string& file_name)
    {
        std::ofstream dot_file(file_name.c_str());
        dot_file << "digraph g {" << std::endl;
        dot_file << "splines = true;" << std::endl;
        dot_file << "node[style=filled, fontname=\"Helvetica\", colorscheme=accent7, color=1];" << std::endl;
        for (size_t i = 0; i < nodes.size(); ++i)
        {
            std::string transparency = (isPartOfSolution(i, final_solutions)) ? "FF" : "30";
            dot_file << "node" << i << "[label=\"" << createString(nodes[i].strategy, nodes[i].pregrasp_configuration) << i <<
                                    "\", color=\"" << createRGBHexString(nodes[i].strategy, nodes[i].pregrasp_configuration) << transparency <<
                                    "\", peripheries=" << (isGoalNode(nodes[i].strategy, nodes[i].pregrasp_configuration) ? 2 : 0) << "]" << std::endl;
        }
        for (size_t i = 0; i < edges.rows(); ++i)
        {
            for (size_t j = 0; j < edges.cols(); ++j)
            {
                if (edges(i, j) == 1)
                {
                    std::string transparency = (isPartOfSolution(i, j, final_solutions)) ? "FF" : "30";
                    dot_file << "node" << i << " -> " << "node" << j << " [color=\"#000000" << transparency << "\"]" << std::endl;
                }
            }
        }
        dot_file << "}" << std::endl;
        dot_file.close();


        // output dot file for connectivity graph
//        std::ofstream dot2_file("ece_solution_paths.dot");
//        dot2_file << "strict digraph g {" << std::endl;
//        dot2_file << "rankdir=LR;" << std::endl;
//        dot2_file << "node[style=filled, fontname=\"Helvetica\", colorscheme=accent7, color=1];" << std::endl;
//        for (size_t j = 0; j < final_solutions.size(); ++j)
//        {
//            for (size_t i = 0; i < final_solutions[j].size(); ++i)
//            {
//                int index = final_solutions[j][i];
//                dot2_file << "node" << index << "[label=\"" << createString(nodes[index].strategy, nodes[index].pregrasp_configuration) << index <<
//                                        "\", color=" << createInt(nodes[index].strategy, nodes[index].pregrasp_configuration) <<
//                                          ", peripheries=" << (isGoalNode(nodes[index].strategy, nodes[index].pregrasp_configuration) ? 2 : 0)  << "]" << std::endl;
//            }
//        }
//        for (size_t j = 0; j < final_solutions.size(); ++j)
//        {
//            for (size_t i = 0; i < final_solutions[j].size() - 1; ++i)
//            {
//                dot2_file << "node" << final_solutions[j][i] << " -> " << "node" << final_solutions[j][i+1] << ";" << std::endl;
//            }
//        }
//        dot2_file << "}" << std::endl;
//        dot2_file.close();

        // output dot file for one solution path / hybrid automaton
//        std::ofstream dotha_file("hybrid_automaton.dot");
//        dotha_file << "digraph g {" << std::endl;
//        dotha_file << "overlap = false;" << std::endl;
//        dotha_file << "splines = false;" << std::endl;
//        dotha_file << "rankdir = \"LR\";" << std::endl;
//        dotha_file << "node [fontsize = \"16\", shape = \"record\"];" << std::endl;
//        for (size_t i = 0; i < final_solutions[1].size(); ++i)
//        {
//            int index = final_solutions[1][i];
//            ::pregrasp_msgs::GraspStrategy& g = nodes[index];
//            ::posesets::PoseSet& p = manifolds[index];
//            const ::tf::Transform& t = p.getOrigin();
//            dotha_file << "node" << index << "[label=\"" << createString(g.strategy, g.pregrasp_configuration) <<
//                          " | (" << t.getOrigin().x() << ", " << t.getOrigin().y() << ", " << t.getOrigin().z() <<
//                          ") (" << t.getRotation().x() << ", " << t.getRotation().y() << ", " << t.getRotation().z() << ", " << t.getRotation().w() << ")" <<
//                          " | "
//                          "\"];" << std::endl;
//        }
//        for (size_t i = 0; i < final_solutions[1].size() - 1; ++i)
//        {
//            int index1 = final_solutions[1][i];
//            int index2 = final_solutions[1][i+1];
//            dotha_file << "node" << index1 << " -> node" << index2 << ";" << std::endl;
//        }
//        dotha_file << "}" << std::endl;
//        dotha_file.close();
    }

    bool isConnected(const ::posesets::PoseSet& first_strategy, const ::posesets::PoseSet& second_strategy)
    {
        return first_strategy.isIntersecting(second_strategy);
    }

    bool isConnected(const ::posesets::PoseSet& first_manifold, const ::posesets::PoseSet& second_manifold,
                     const ::pregrasp_msgs::GraspStrategy& first_strategy, const ::pregrasp_msgs::GraspStrategy& second_strategy)
    {
        if (first_strategy.strategy == second_strategy.strategy)
            return false;

        if (first_strategy.strategy == pregrasp_msgs::GraspStrategy::STRATEGY_POSITION && second_strategy.strategy != pregrasp_msgs::GraspStrategy::STRATEGY_LAND)
            return false;

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

        size_t total = 0;
        ROS_INFO("The planner received:");
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

        // do a breadth-first search beginning with the positioning
        Paths solutions, intermediate_solutions, final_solutions;
        Path p1;        
        p1.push_back(0);
        intermediate_solutions.push_back(p1);
        do
        {
            solutions = intermediate_solutions;
            intermediate_solutions.clear();

            for (size_t i = 0; i < solutions.size(); ++i)
            {
                Path& p = solutions[i];
                int node_index = p.back();
                if (!isGoalNode(nodes[node_index].strategy, nodes[node_index].pregrasp_configuration))
                {
                    // go through all neighbors and add for each one a new path
                    for (size_t j = 0; j < nodes.size(); ++j)
                    {
                        if (std::find(p.begin(), p.end(), j) != p.end())
                            continue;

                        if (edges(node_index, j) == 1)
                        {
                            Path new_p(p);
                            new_p.push_back(j);
                            intermediate_solutions.push_back(new_p);
                        }
                    }
                }
                else
                {
                    final_solutions.push_back(p);
                }
            }
        } while(!intermediate_solutions.empty());

        ROS_INFO("No. of solution paths: %zu", final_solutions.size());

        // output dot file for connectivity graph
        createDotFile(nodes, edges, final_solutions, "ece_graph.dot");

        // filter only those strategies that are part of the final solution
        std::vector<int> helper;
        Strategies solution_nodes;
        for (Paths::const_iterator it = final_solutions.begin(); it != final_solutions.end(); ++it)
        {
            for (Path::const_iterator jt = it->begin(); jt != it->end(); ++jt)
            {
                if (std::find(helper.begin(), helper.end(), *jt) == helper.end())
                {
                    solution_nodes.push_back(nodes[*jt]);
                    helper.push_back(*jt);
                }
            }
        }

        ROS_INFO("There are %zu nodes in the grasp_tree.", solution_nodes.size());

        // TODO publishRVizMarkers(solution_nodes, (*pushing_motions_)->header.frame_id, "/grasp_plan");
        // TODO publishRVizMarkers2(nodes, final_solutions, (*pushing_motions_)->header.frame_id);

//        createHAXML(nodes, final_solutions, "ha_");

        // generate grasp strategy array as output
        pregrasp_msgs::GraspStrategyArrayPtr pregrasp_messages(new ::pregrasp_msgs::GraspStrategyArray);
        if (!final_solutions.empty())
        {
            Path& p = final_solutions[0];
            /* OLD WAY
            pregrasp_messages->header = nodes[p[0]].pregrasp_pose.pose.header;
            for (size_t i = 0; i < p.size(); ++i)
            {
                pregrasp_messages->strategies.push_back(nodes[p[i]]);
            }
            */

            // up to now: Use the fields [pregrasp_pose.pose > pregrasp_pose.center > object.pose > object.center]
            if (!p.empty() && p.size() <= 2)
            {
                pregrasp_messages->header = nodes[p[0]].pregrasp_pose.pose.header;
                pregrasp_messages->strategies.push_back(nodes[p[0]]);

                if (p.size() > 1)
                {
                    pregrasp_messages->strategies[0].pregrasp_pose.pose.pose.orientation = nodes[p[1]].pregrasp_pose.pose.pose.orientation;
                    pregrasp_messages->strategies[0].object = nodes[p[1]].pregrasp_pose;
                }
            }

        }
        (*pregrasp_messages_) = pregrasp_messages;

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_grasping, ecto_rbo_grasping::PlanGrasps, "PlanGrasps", "Combine grasps and non-prehensile actions into motion sequences that result in grasps .")
