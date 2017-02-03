/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/*
 * ecto_yaml_node.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: clemens
 */

#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <ecto/ecto.hpp>

#include "ecto_rbo_yaml/PlasmYamlFactory.h"

void printCallbackFunction(std::ofstream& file, const PlasmYamlFactory::DynamicParams& dyn_params)
{
  for (PlasmYamlFactory::DynamicParams::const_iterator it = dyn_params.begin(); it != dyn_params.end(); ++it)
  {
    file << "  cells_by_name[\"" << it->first.first << "\"]->parameters[\"" << it->first.second << "\"] << config." << it->first.first << "___" << it->first.second << ";" << std::endl;
  }
}

void printCfgFile(std::ofstream& file, const PlasmYamlFactory::DynamicParams& dyn_params)
{
  file << ""
          "#!/usr/bin/env python\n"
          "PACKAGE = '" << ROS_PACKAGE_NAME << "'\n"
          "import math\n"
          "\n"
          "from dynamic_reconfigure.parameter_generator_catkin import *\n"
          "\n"
          "gen = ParameterGenerator()\n"
          "\n";

  int level = 0;

  for (PlasmYamlFactory::DynamicParams::const_iterator it = dyn_params.begin(); it != dyn_params.end(); ++it)
  {
      file << "gen.add('" << it->first.first << "___" << it->first.second << "', " << it->second.first << ", " << level++ << ", 'some description'" << it->second.second << ")" << std::endl;
  }

  file << ""
          "\n"
          "exit(gen.generate(PACKAGE, 'ecto_yaml_dynamic_reconfigure', 'PlasmParameters'))\n";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./ecto_yaml_dynamic_reconfigure_preparation plasm.yaml" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Loading " << argv[1] << " ..." << std::endl;

  ecto::plasm::ptr my_plasm(new ecto::plasm);
  PlasmYamlFactory factory(my_plasm);
  factory.addToPythonPath(ros::package::getPath(ROS_PACKAGE_NAME) + "/build/gen/py/ecto_rpimp/");
  factory.load(argv[1]);

  PlasmYamlFactory::DynamicParams& dyn_params = factory.getDynamicParameters();

  std::string foldername = ros::package::getPath(ROS_PACKAGE_NAME);

  // generate cfg file
  std::string cfg_filename(foldername + "/cfg/PlasmParameters.cfg");
  ROS_INFO_STREAM("Writing " << cfg_filename << "...");
  std::ofstream cfg_file(cfg_filename.c_str());
  printCfgFile(cfg_file, dyn_params);
  cfg_file.close();

  // generate callback include file
  std::string inc_filename(foldername + "/src/ecto_yaml_dynamic_reconfigure.h.inl");
  ROS_INFO_STREAM("Writing " << inc_filename << "...");
  std::ofstream inc_file(inc_filename.c_str());
  printCallbackFunction(inc_file, dyn_params);
  inc_file.close();

  ROS_INFO("Done! Now make again.");

  return EXIT_SUCCESS;
}
