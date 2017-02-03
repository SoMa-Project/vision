/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/*
 * PlasmYamlFactory.cpp
 *
 *  Created on: Aug 9, 2016
 *      Author: clemens
 */

#include "ecto_rbo_dbg/PlasmYamlFactoryRViz.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>

#include <ecto/tendrils.hpp>
#include <ecto/edge.hpp>
#include <ecto/cell.hpp>
#include <ecto/vertex.hpp>
#include <ecto/util.hpp>
#include <ecto/graph/types.hpp>

#include <ecto_ros/ecto_ros.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <ecto_opencv/

#include <pregrasp_msgs/GraspStrategyArray.h>

#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> UnalignedAffine3f;

using namespace ecto;

PlasmYamlFactoryRViz::PlasmYamlFactoryRViz(const ecto::plasm_ptr& plasm) :
    PlasmYamlFactory(plasm)
{
    import_modules.push_back("ecto_rbo_dbg");
    import_modules.push_back("ecto_rbo_dbg_py");
    import_modules.push_back("ecto_rbo_dbg_py.rviz_publishers");
}

PlasmYamlFactoryRViz::~PlasmYamlFactoryRViz()
{
}

std::string PlasmYamlFactoryRViz::randomString(size_t length)
{
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(length,0);
    std::generate_n( str.begin(), length, randchar );
    return str;
}

void PlasmYamlFactoryRViz::addCell(const std::string& type, const ecto::cell_ptr& parent, const std::string& parent_output, const ecto::cell::ptr& child_cell, const std::string& child_input)
{
    // Generate new cell with random name and add to plasm
    std::string newcell_name = std::string("RViz_publisher_") + randomString(6);

    ecto::cell::ptr newcell;
    try
    {
        newcell = ecto::registry::create(type);
    }
    catch (const ecto::except::EctoException& e)
    {
        std::cerr << ecto::except::diagnostic_string(e) << std::endl;

        //std::cerr << ecto::name_of<UnalignedAffine3f>() << std::endl;
        /*
        std::cerr << "All available cells:" << std::endl;
        for (std::map<std::string, ecto::registry::entry_t>::iterator it = ecto::registry::cellmap.begin(); it != ecto::registry::cellmap.end(); ++it)
            std::cerr << it->first << std::endl;
        */
        //throw std::exception();
    }

    newcell->name(newcell_name);
    newcell->declare_params();
    newcell->declare_io();

    newcell->parameters["namespace"]->set_default_val(parent->name() + std::string("_") + parent_output);

    this->plasm->insert(newcell);
    this->cells_by_name[newcell_name] = newcell;

    // connect it
    try
    {
        std::cout << "Connecting " << parent->name() << "::" << parent_output << " and " << newcell_name << std::endl;
        plasm->connect(parent, parent_output, newcell, "input");

        std::cout << "Connecting " << newcell_name << "::" << "output" << " and " << child_cell->name() << "::" << child_input << std::endl;
        plasm->connect(newcell, "output", child_cell, child_input);

    }
    catch (const ecto::except::EctoException& e)
    {
        std::cerr << ecto::except::diagnostic_string(e) << std::endl;
    }
}

void PlasmYamlFactoryRViz::addRVizCellsToOutputs(const std::vector<std::string>& cell_names)
{
    // add marker publisher cell
    ecto::cell::ptr marker_pub;
    try
    {
        std::string marker_pub_name("rviz_marker_publisher");
        marker_pub = ecto::registry::create("ecto_rbo_dbg::RVizMarkerPublisher");
        marker_pub->name(marker_pub_name);
        marker_pub->declare_params();
        marker_pub->declare_io();
        this->plasm->insert(marker_pub);
        this->cells_by_name[marker_pub_name] = marker_pub;
    }
    catch (const ecto::except::EctoException& e)
    {
        std::cerr << ecto::except::diagnostic_string(e) << std::endl;
    }

    std::map<std::string, std::string> outputs_to_rviz_cells;

    outputs_to_rviz_cells["Eigen::Transform<float, 3, 2, 2>"]
            = "ecto_rbo_dbg::RVizMessageConverter<Eigen::Transform<float, 3, 2, 2> >";
    outputs_to_rviz_cells["std::vector<Eigen::Transform<float, 3, 2, 2>, std::allocator<Eigen::Transform<float, 3, 2, 2> > >"]
            = "ecto_rbo_dbg::RVizMessageConverter<std::vector<Eigen::Transform<float, 3, 2, 2>, std::allocator<Eigen::Transform<float, 3, 2, 2> > > >";
    outputs_to_rviz_cells["std::vector<Eigen::Matrix<float, 4, 1, 2, 4, 1>, std::allocator<Eigen::Matrix<float, 4, 1, 2, 4, 1> > >"]
            = "ecto_rbo_dbg::RVizMessageConverter<std::vector<Eigen::Matrix<float, 4, 1, 2, 4, 1>, std::allocator<Eigen::Matrix<float, 4, 1, 2, 4, 1> > > >";
    outputs_to_rviz_cells["std::vector<Eigen::Matrix<float, 3, 1, 2, 3, 1>, std::allocator<Eigen::Matrix<float, 3, 1, 2, 3, 1> > >"]
            = "ecto_rbo_dbg::RVizMessageConverter<std::vector<Eigen::Matrix<float, 3, 1, 2, 3, 1>, std::allocator<Eigen::Matrix<float, 3, 1, 2, 3, 1> > > >";
    outputs_to_rviz_cells["std::vector<boost::shared_ptr<pcl::ModelCoefficients const>, std::allocator<boost::shared_ptr<pcl::ModelCoefficients const> > >"]
            = "ecto_rbo_dbg::RVizMessageConverter<std::vector<boost::shared_ptr<pcl::ModelCoefficients const>, std::allocator<boost::shared_ptr<pcl::ModelCoefficients const> > > >";

    outputs_to_rviz_cells[ecto::name_of<pregrasp_msgs::GraspStrategyArrayConstPtr>()]
            = "ecto_rbo_dbg::RVizMessageConverter<" + ecto::name_of<pregrasp_msgs::GraspStrategyArrayConstPtr>() + " >";

//    std::map<std::string, std::string> double_outputs_to_rviz_cells;
//    outputs_to_rviz_cells["std::vector<pcl::PointIndices, std::allocator<pcl::PointIndices> >"]
//            = "ecto_rbo_dbg_py.rviz_publishers.PublishClusters";

    if (cell_names.empty())
    {
        int counter = 0;

        // iterate over outputs of all cells
        for (CellsByName::const_iterator it = cells_by_name.begin(); it != cells_by_name.end(); ++it)
        {
            // check if cell exists -- throw exception
            ecto::cell_ptr cell = it->second;
            std::cout << "Cell: " << it->first << std::endl;
            if (cell->outputs.size() > 0)
            {
                for (ecto::tendrils::iterator out = cell->outputs.begin(); out != cell->outputs.end(); ++out)
                {
                    const std::string& type = out->second->type_name();
                    std::cout << type << std::endl;

                    std::map<std::string, std::string>::iterator newCell = outputs_to_rviz_cells.find(type);
                    if (newCell != outputs_to_rviz_cells.end())
                    {
                        addCell(newCell->second, cell, out->first, marker_pub, std::string("input_") + std::to_string(counter++));
                    }

                    //
                }
            }
        }
    }
    else
    {
        // iterate over outputs of desired cells
        for (std::vector<std::string>::const_iterator it = cell_names.begin(); it != cell_names.end(); ++it)
        {

            ecto::cell_ptr cell = cells_by_name[*it];
        }
    }
}
