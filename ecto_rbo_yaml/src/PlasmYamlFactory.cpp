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
 *  Created on: Nov 26, 2012
 *      Author: clemens
 */

#include "ecto_rbo_yaml/PlasmYamlFactory.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>

#include <ecto/tendrils.hpp>
#include <ecto/edge.hpp>
#include <ecto/cell.hpp>
#include <ecto/vertex.hpp>
#include <ecto/graph/types.hpp>

#include <ecto_ros/ecto_ros.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <ecto_opencv/

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

const char* const PlasmYamlFactory::DYNAMIC_RECONFIGURE_TAG = "dynamic_reconfigure_parameters";

using namespace ecto;

PlasmYamlFactory::PlasmYamlFactory(const ecto::plasm_ptr& plasm) :
    plasm(plasm)
{
    import_modules.push_back("ecto_pcl");
    import_modules.push_back("ecto_pcl_ros");
    import_modules.push_back("ecto_ros");
    import_modules.push_back("ecto_ros.ecto_ros_main");
    import_modules.push_back("ecto_ros.ecto_sensor_msgs");
    import_modules.push_back("ecto_ros.ecto_std_msgs");
    import_modules.push_back("ecto_ros.ecto_geometry_msgs");
    import_modules.push_back("ecto_opencv");
    import_modules.push_back("ecto_opencv.imgproc");
    import_modules.push_back("ecto_opencv.highgui");

    //import_modules.push_back("ecto_rbo_grasping");
    import_modules.push_back("ecto_rbo_pcl");
//        boost::python::import("ecto_object_recognition");
}

PlasmYamlFactory::~PlasmYamlFactory()
{
}

void PlasmYamlFactory::save(const std::string& name)
{
    YAML::Emitter out;
    out.SetSeqFormat(YAML::Flow);

    // iterate through all cells
    std::vector<ecto::cell_ptr> all_cells = plasm->cells();

    out << YAML::BeginMap;
    for (size_t i = 0; i < all_cells.size(); ++i)
    {
        out << YAML::Key << all_cells[i]->name();
        out << YAML::Value;

        out << YAML::BeginMap;
        out << YAML::Key << "type";
        out << YAML::Value << all_cells[i]->type();

        // store connectivity
        out << YAML::Key << "inputs";
        out << YAML::Value << YAML::BeginMap;

        ecto::graph::graph_t& g = plasm->graph();
        ecto::graph::graph_t::vertex_iterator begin_vertices, end_vertices;
        boost::tie(begin_vertices, end_vertices) = boost::vertices(g);
        while (begin_vertices != end_vertices && g[*begin_vertices]->cell() != all_cells[i])
        {
            ++begin_vertices;
        }
        ecto::graph::graph_t::in_edge_iterator b_in, e_in;
        boost::tie(b_in, e_in) = boost::in_edges(*begin_vertices, g);
        while (b_in != e_in)
        {
            ecto::graph::edge_ptr in_edge = g[*b_in];
            ecto::cell_ptr from_module = g[boost::source(*b_in, g)]->cell();

            out << YAML::Key << in_edge->to_port();
            out << YAML::Value << from_module->name() + std::string("/") + in_edge->from_port();

            ++b_in;
        }
        out << YAML::EndMap;

        // iterate through cell's parameter list
        for (ecto::tendrils::iterator it = all_cells[i]->parameters.begin(); it != all_cells[i]->parameters.end(); ++it)
        {
            out << YAML::Key << it->first;

            if (it->second->is_type<bool>())
            {
                out << YAML::Value << it->second->get<bool>();
            }
            else if (it->second->is_type<int>())
            {
                out << YAML::Value << it->second->get<int>();
            }
            else if (it->second->is_type<double>())
            {
                out << YAML::Value << it->second->get<double>();
            }
            else if (it->second->is_type<float>())
            {
                out << YAML::Value << it->second->get<float>();
            }
            else if (it->second->is_type<std::string>())
            {
                out << YAML::Value << it->second->get<std::string>();
            }
            else if (it->second->is_type<UnalignedVector3f>())
            {
                UnalignedVector3f v = it->second->get<UnalignedVector3f>();
                out << YAML::Value << YAML::BeginSeq << v[0] << v[1] << v[2] << YAML::EndSeq;
            }
            else if (it->second->is_type<UnalignedVector4f>())
            {
                UnalignedVector4f v = it->second->get<UnalignedVector4f>();
                out << YAML::Value << YAML::BeginSeq << v[0] << v[1] << v[2] << v[3] << YAML::EndSeq;
            }
            else if (it->second->is_type<UnalignedAffine3f>())
            {
                UnalignedAffine3f a = it->second->get<UnalignedAffine3f>();
                out << YAML::Value << YAML::BeginSeq << a(0, 0) << a(0, 1) << a(0, 2) << a(0, 3) << a(1, 0) << a(1, 1)
                    << a(1, 2) << a(1, 3) << a(2, 0) << a(2, 1) << a(2, 2) << a(2, 3) << YAML::EndSeq;
            }
            else if (it->second->is_type< std::vector<std::string> >())
            {
                std::vector<std::string> list = it->second->get< std::vector<std::string> >();
                out << YAML::Value << YAML::BeginSeq;
                for (size_t i = 0; i < list.size(); ++i)
                    out << list[i];
                out << YAML::EndSeq;
            }
            else
            {
                out << YAML::Value << "UNKNOWN!";
            }
        }

        out << YAML::EndMap;
    }
    out << YAML::EndMap;

    //  std::cout << "RESULTAT" << std::endl;
    //  std::cout << out.c_str() << std::endl;

    std::ofstream file(name.c_str());
    file << out.c_str();
    file.close();
}

void PlasmYamlFactory::load(const std::string& name)
{
    try
    {
        Py_Initialize();

        // add something to the python path
        //boost::filesystem::path workingDir = boost::filesystem::complete("./").normalize();
        PyObject* sysPath = PySys_GetObject("path");
        for (std::vector<std::string>::const_iterator it = python_path.begin(); it != python_path.end(); ++it)
        {
            PyList_Insert(sysPath, 0, PyString_FromString(it->c_str()));
        }

        ROS_INFO("PythonPath:");
        for (int i = 0; i < PyList_Size(sysPath); ++i)
        {
            PyObject* item = PyList_GetItem(sysPath, i);
            std::string path_for_python_modules = PyString_AsString(item);
            ROS_DEBUG_STREAM("  " << path_for_python_modules);
        }

        for (std::vector<std::string>::const_iterator it = import_modules.begin(); it != import_modules.end(); ++it)
        {
            boost::python::import(it->c_str());
        }
    }
    catch (...)
    {
        PyErr_Print();

        throw std::exception();
    }

    YAML::Node doc = YAML::LoadFile(name.c_str());
    
    std::vector<std::string> dyn_params = loadDynamicReconfigureParameters(&doc);
    std::map<std::string, std::string> connections;

    for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it)
    {
        std::string cell_name(it->first.as<std::string>());

        if (cell_name == DYNAMIC_RECONFIGURE_TAG)
            continue;

        std::string cell_type(it->second["type"].as<std::string>());

        std::cout << "Instantiating " << cell_name << " of type " << cell_type << "." << std::endl;

        ecto::cell::ptr cell;
        try
        {
            cell = ecto::registry::create(cell_type);
        }
        catch (const ecto::except::EctoException& e)
        {
            std::cerr << ecto::except::diagnostic_string(e) << std::endl;
            throw std::exception();
        }

        cell->name(cell_name);
        cell->declare_params();

        // set parameters
        for (YAML::const_iterator jt = it->second.begin(); jt != it->second.end(); ++jt)
        {
            std::string param_key(jt->first.as<std::string>());

            if (param_key == "type")
                continue;

            if (param_key == "inputs")
            {
                for (YAML::const_iterator kt = jt->second.begin(); kt != jt->second.end(); ++kt)
                {
                    std::string to_port(kt->first.as<std::string>());
                    std::string from_port(kt->second.as<std::string>());
                    connections[cell_name + std::string("/") + to_port] = from_port;
                }
                continue;
            }

            // remove dollar
            bool is_reconfigurable = false;
            if (boost::starts_with(param_key, "$"))
            {
                is_reconfigurable = true;
                param_key = param_key.substr(1);
            }

            tendrils::iterator p = cell->parameters.find(param_key);

            std::cout << "Setting parameter: " << param_key << std::endl;

            if (p != cell->parameters.end())
            {
                std::string dyn_param_type, param_range;

                if (p->second->is_type<bool>())
                {
                    bool b = jt->second.as<bool>();
                    *p->second << b;

                    dyn_param_type = "bool_t";
                    param_range = (b) ? ", True" : ", False";
                }
                else if (p->second->is_type<int>())
                {
                    int n = jt->second.as<int>();
                    *p->second << n;

                    dyn_param_type = "int_t";
                    int max_range = std::max(1, n * 30);
                    param_range = std::string(", ") + boost::lexical_cast<std::string>(n) + std::string(", 0, ") + boost::lexical_cast<std::string>(max_range);
                }
                else if (p->second->is_type<unsigned>())
                {
                    unsigned n = jt->second.as<unsigned>();
                    *p->second << n;

                    dyn_param_type = "int_t";
                    int max_range = std::max(1, static_cast<int>(n) * 30);
                    param_range = std::string(", ") + boost::lexical_cast<std::string>(n) + std::string(", 0, ") + boost::lexical_cast<std::string>(max_range);
                }
                else if (p->second->is_type<double>())
                {
                    double d = jt->second.as<double>();
                    *p->second << d;

                    dyn_param_type = "double_t";
                    double min_range = std::min(0.0, d * 30.0);
                    double max_range = std::max(1.0, d * 30.0);
                    if (d == 0) min_range = -30.0;
                    param_range = std::string(", ") + boost::lexical_cast<std::string>(d) + std::string(", ") + boost::lexical_cast<std::string>(min_range) + std::string(", ") + boost::lexical_cast<std::string>(max_range);
                }
                else if (p->second->is_type<float>())
                {
                    float f = jt->second.as<float>();
                    *p->second << f;

                    dyn_param_type = "double_t";
                    double max_range = std::max(1.0, f * 30.0);
                    param_range = std::string(", ") + boost::lexical_cast<std::string>(f) + std::string(", 0, ") + boost::lexical_cast<std::string>(max_range);
                }
                else if (p->second->is_type<std::string>())
                {
                    std::string s = jt->second.as<std::string>();
                    *p->second << s;

                    std::cout << " with value: " << s << std::endl;

                    dyn_param_type = "str_t";
                    param_range = std::string(", '") + s + std::string("'");
                }
                else if (p->second->is_type<UnalignedVector3f>())
                {
                    UnalignedVector3f v;
                    v[0] = jt->second[0].as<float>();
                    v[1] = jt->second[1].as<float>();
                    v[2] = jt->second[2].as<float>();
                    *p->second << v;
                }
                else if (p->second->is_type<UnalignedVector4f>())
                {
                    UnalignedVector4f v;
                    v[0] = jt->second[0].as<float>();
                    v[1] = jt->second[1].as<float>();
                    v[2] = jt->second[2].as<float>();
                    v[3] = jt->second[3].as<float>();
                    *p->second << v;
                }
                else if (p->second->is_type<UnalignedAffine3f>())
                {
                    UnalignedAffine3f a;
                    a(0, 0) = jt->second[0].as<float>();
                    a(0, 1) = jt->second[1].as<float>();
                    a(0, 2) = jt->second[2].as<float>();
                    a(0, 3) = jt->second[3].as<float>();
                    a(1, 0) = jt->second[4].as<float>();
                    a(1, 1) = jt->second[5].as<float>();
                    a(1, 2) = jt->second[6].as<float>();
                    a(1, 3) = jt->second[7].as<float>();
                    a(2, 0) = jt->second[8].as<float>();
                    a(2, 1) = jt->second[9].as<float>();
                    a(2, 2) = jt->second[10].as<float>();
                    a(2, 3) = jt->second[11].as<float>();
                    *p->second << a;
                }
                else if (p->second->is_type< std::vector<std::string> >())
                {
                    std::cout << "Reading a vector of strings as parameter:";
                    std::vector<std::string> list_of_strings;
                    for (size_t i = 0; i < jt->second.size(); ++i)
                    {
                        list_of_strings.push_back(jt->second[i].as<std::string>());
                        std::cout << ", " << list_of_strings.back();
                    }

                    std::cout << std::endl;


                    *p->second << list_of_strings;
                }
                else
                {
                    std::cerr << "UNKNOWN conversion for parameter '" << p->first << "'!" << std::endl;
                }

                //            if (std::find(dyn_params.begin(), dyn_params.end(), param_key) != dyn_params.end() && !dyn_param_type.empty())
                if (is_reconfigurable && !dyn_param_type.empty())
                {
                    dynamic_parameters.insert(std::make_pair(std::make_pair(cell_name, param_key), std::make_pair(dyn_param_type, param_range)));
                }
            }
            else
            {
                std::cerr << "Parameter " << param_key << " doesn't exist!" << std::endl;
            }
        }

        cell->declare_io();
        //cell->configure();

        plasm->insert(cell);
        cells_by_name[cell_name] = cell;
    }

    // create connectivity
    for (std::map<std::string, std::string>::iterator cit = connections.begin(); cit != connections.end(); ++cit)
    {
        size_t output = cit->second.find_last_of("/");
        size_t input = cit->first.find_last_of("/");

        std::cout << "Connecting (" << cit->second.substr(0, output) << ", " << cit->second.substr(output + 1)
                  << ") to (" << cit->first.substr(0, input) << ", " << cit->first.substr(input + 1) << ")" << std::endl;

        if (cells_by_name.find(cit->second.substr(0, output)) == cells_by_name.end())
            std::cerr << "Cell " << cit->second.substr(0, output) << " does NOT exist!" << std::endl;


        if (cells_by_name.find(cit->first.substr(0, input)) == cells_by_name.end())
            std::cerr << "Cell " << cit->first.substr(0, input) << " does NOT exist!" << std::endl;

        try
        {
            plasm->connect(cells_by_name[cit->second.substr(0, output)], cit->second.substr(output + 1),
                    cells_by_name[cit->first.substr(0, input)], cit->first.substr(input + 1));
        }
        catch (const ecto::except::EctoException& e)
        {
            std::cerr << ecto::except::diagnostic_string(e) << std::endl;

            ecto::cell_ptr cell_a = cells_by_name[cit->second.substr(0, output)];
            ecto::cell_ptr cell_b = cells_by_name[cit->first.substr(0, input)];
            std::cerr << "First cell has " << cell_a->outputs.size() << " outputs: " << std::endl;
            for (ecto::tendrils::iterator tendril_it = cell_a->outputs.begin(); tendril_it != cell_a->outputs.end(); ++tendril_it)
            {
                std::cerr << "    " << tendril_it->first << " " << tendril_it->second->type_name() << " " << tendril_it->second->doc() << std::endl;
            }

            std::cerr << "Second cell has " << cell_b->inputs.size() << " inputs: " << std::endl;
            for (ecto::tendrils::iterator tendril_it = cell_b->inputs.begin(); tendril_it != cell_b->inputs.end(); ++tendril_it)
            {
                std::cerr << "    " << tendril_it->first << " " << tendril_it->second->type_name() << " " << tendril_it->second->doc() << std::endl;
            }

            throw std::exception();
        }
    }
}

std::vector<std::string> PlasmYamlFactory::loadDynamicReconfigureParameters(const YAML::Node* node)
{
    std::vector<std::string> dyn_params;

    /*
     This is the old separate reconfigure_param tag --> now replaced by $ in front of param name
  const YAML::Node* param_node = node->FindValue(DYNAMIC_RECONFIGURE_TAG);
  if (param_node)
  {
    for (YAML::Iterator it = param_node->begin(); it != param_node->end(); ++it)
    {
      std::string s;
      *it >> s;
      dyn_params.push_back(s);
    }
  }
  */

    return dyn_params;
}

PlasmYamlFactory::CellsByName& PlasmYamlFactory::getCellsByName()
{
    return cells_by_name;
}

PlasmYamlFactory::DynamicParams& PlasmYamlFactory::getDynamicParameters()
{
    return dynamic_parameters;
}

void PlasmYamlFactory::addToPythonPath(const std::string& path)
{
    python_path.push_back(path);
}

void PlasmYamlFactory::setPythonPath(const std::vector<std::string>& path)
{
    python_path = path;
}

std::vector<std::string> PlasmYamlFactory::getPythonPath()
{
    return python_path;
}

void PlasmYamlFactory::addModule(const std::string& module)
{
    import_modules.push_back(module);
}
