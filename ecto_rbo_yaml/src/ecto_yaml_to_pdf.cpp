/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/*
 * ecto_yaml_to_pdf.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: clemens
 */

#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>

#include <ecto/ecto.hpp>

#include <ros/console.h>
#include <ros/package.h>

#include "ecto_rbo_yaml/PlasmYamlFactory.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " plasm.yaml");
    return EXIT_FAILURE;
  }

  ROS_INFO_STREAM("Loading " << argv[1] << " ...");

  ecto::plasm::ptr my_plasm(new ecto::plasm);
  PlasmYamlFactory factory(my_plasm);
  factory.addToPythonPath(ros::package::getPath(ROS_PACKAGE_NAME) + "/build/gen/py/ecto_rpimp/");
  factory.load(argv[1]);

  std::string plasm_name(argv[1]);
  size_t substring_start = plasm_name.find_last_of("/\\");
  plasm_name = plasm_name.substr(substring_start + 1, plasm_name.find_last_of('.') - substring_start - 1);

  // create dot file
  std::string dot_filename(plasm_name + ".dot");
  std::ofstream file(dot_filename.c_str());
  my_plasm->viz(file);
  file.close();

  // convert dot file into pdf
  std::string command("dot -Tpdf " + dot_filename + " -o " + plasm_name + ".pdf");
  system(command.c_str());

  // remove dot file
  remove(dot_filename.c_str());
}
