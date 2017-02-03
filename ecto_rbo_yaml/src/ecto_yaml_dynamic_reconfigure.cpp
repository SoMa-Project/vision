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
 *  Created on: Nov 29, 2012
 *      Author: clemens
 */

#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <ecto/ecto.hpp>
#include <ecto/python.hpp>
#include <ecto/ecto.hpp>
#include <ecto/tendrils.hpp>
#include <ecto/scheduler.hpp>

#include <dynamic_reconfigure/server.h>

#include "ecto_rbo_yaml/PlasmParametersConfig.h"
#include "ecto_rbo_yaml/PlasmYamlFactory.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
sig_atomic_t volatile g_request_configure = 0;

boost::shared_ptr<ecto::scheduler> scheduler;
PlasmYamlFactory::CellsByName cells_by_name;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  ROS_FATAL("SHUTDOWN REQUEST!!!");
  g_request_shutdown = 1;
}

void callback(ecto_rbo_yaml::PlasmParametersConfig& config, uint32_t level) {
  if (!scheduler)
    return;

  g_request_configure = 1;

  ROS_INFO("TRYING TO STOP!!");

  while (scheduler->state() == ecto::scheduler::EXECUTING)
  {
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("STOPPED SUCCESSFULLY");

#include "ecto_yaml_dynamic_reconfigure.h.inl"

  ROS_INFO("TRYING TO RESTART!");

  g_request_configure = 0;
//  scheduler->execute_async();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ecto_yaml_dynamic_reconfigure", ros::init_options::NoSigintHandler);

  std::vector<std::string> arguments;
  ros::removeROSArgs(argc, argv, arguments);

  if (arguments.size() < 2) {
    std::cerr << "Usage: ./ecto_yaml_dynamic_reconfigure plasm.yaml" << std::endl;
    return EXIT_FAILURE;
  }

  ROS_INFO_STREAM("Loading " << arguments[1] << " ...");

  ecto::plasm::ptr my_plasm(new ecto::plasm);
  PlasmYamlFactory factory(my_plasm);
  factory.addToPythonPath(ros::package::getPath(ROS_PACKAGE_NAME) + "/build/gen/py/ecto_rpimp/");
  factory.load(arguments[1]);

  ros::init(argc, argv, "ecto_yaml_dynamic_reconfigure", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  //ros::CallbackQueue my_queue;
  //nh.setCallbackQueue(&my_queue);

  cells_by_name = factory.getCellsByName();

  dynamic_reconfigure::Server<ecto_rbo_yaml::PlasmParametersConfig> server;
  dynamic_reconfigure::Server<ecto_rbo_yaml::PlasmParametersConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // the spinner is needed to keep the ros::message_queue popping
  ros::AsyncSpinner spinner(0);
  //ros::AsyncSpinner spinner(0, &my_queue);
  //ros::MultiThreadedSpinner spinner(4);
  spinner.start();

  scheduler.reset(new ecto::scheduler(my_plasm));

  signal(SIGINT, mySigIntHandler);

  ros::Rate r(20);

  while (!g_request_shutdown) {
    if (!g_request_configure)
      scheduler->execute(1);

    r.sleep();

    ROS_INFO("One Iteration.");
  }

  spinner.stop();
  scheduler->stop();

  ROS_INFO_STREAM("Stats: \n" << scheduler->stats());

  factory.save("autosave.yaml");

  return EXIT_SUCCESS;
}
