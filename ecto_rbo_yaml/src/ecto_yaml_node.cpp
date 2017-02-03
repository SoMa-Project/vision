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

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <ecto/ecto.hpp>
#include <ecto/scheduler.hpp>

#include <log4cxx/logger.h>

#include "ecto_rbo_yaml/PlasmYamlFactory.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig) {
  g_request_shutdown = 1;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ecto_yaml_node", ros::init_options::NoSigintHandler);

  std::vector<std::string> arguments;
  ros::removeROSArgs(argc, argv, arguments);

  if (arguments.size() < 2) {
    ROS_ERROR("Usage: ./ecto_yaml_node plasm.yaml");
    return EXIT_FAILURE;
  }

  ROS_INFO_STREAM("Loading " << arguments[1] << " ...");

  //log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  //my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  ros::NodeHandle nh;

  ecto::plasm::ptr my_plasm(new ecto::plasm);
  PlasmYamlFactory factory(my_plasm);
  factory.addToPythonPath(ros::package::getPath(ROS_PACKAGE_NAME) + "/build/gen/py/ecto_rpimp/");
  factory.load(arguments[1]);

  // the spinner is needed to keep the ros::message_queue popping
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ecto::scheduler scheduler(my_plasm);

  signal(SIGINT, mySigIntHandler);

  ros::Rate r(20);

  while (!g_request_shutdown) {
    scheduler.execute(1);
    r.sleep();
  }

  spinner.stop();
  scheduler.stop();

  ROS_INFO_STREAM("Stats: \n" << scheduler.stats());

  factory.save("autosave.yaml");

  return EXIT_SUCCESS;
}
