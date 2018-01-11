#!/usr/bin/env python
import argparse
import os
import rospkg
import sys
import threading  

import ecto
import ecto_pcl
import ecto_rbo_pcl
import ecto_ros
import numpy as np
import plasm_yaml_factory
import rospy
from geometry_graph_msgs.msg import Graph
from pregrasp_msgs.srv import ComputeECGraph
from pregrasp_msgs import srv

class VisionServer:
    def __init__(self, ecto_plasm, ecto_cells):
        self.ecto_plasm = ecto_plasm
        self.ecto_cells = ecto_cells
        self.ecto_scheduler = ecto.Scheduler(self.ecto_plasm)

        rospy.Subscriber('geometry_graph', Graph, self.callback_vision_result)
        rospy.loginfo('Subscribed to the geometry_graph topic.')

    def handle_compute_ec_graph(self, req):
        """Callback running the vision pipeline.
    
        Args:
            req: The request of the ComputeECGraph service.
    
        Returns:
            The response of the service defined in ComputeECGraph
        """
        try:
            return self.run_vision(req)
        except Exception as e:
            rospy.logerr("!!! Object recognition crashed !!!")
            rospy.logerr(e)

            return

    def callback_vision_result(self, data):
        self.outputgraph = data

    def run_vision(self, req):
        """Request callback for running vision
        """

        rospy.loginfo("!!! running vision !!!")
        self.outputgraph = None

        # start scheduler; iterate exactly once over the ecto plasm
        self.ecto_scheduler.execute(niter=1)

        # Get geometry graph
        self.outputgraph.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

        return srv.ComputeECGraphResponse(self.outputgraph)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml_file', type=str)
    parser.add_argument('--debug', action='store_true', default=False)
    parser.add_argument('--showgraph', action='store_true', default=False)
    parser.add_argument('--service', action='store_true', default=False)
    myargv = rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(myargv)

    rospy.init_node('plasm_yaml_ros_service')
    
    ecto_ros.init(myargv, 'plasm_yaml_ros_service', anonymous = False)

    rospack = rospkg.RosPack()
    if any([args.yaml_file.startswith(x) for x in [".", "/"]]):
        yml_file = [args.yaml_file]
    else:
        yml_file = map (lambda x: os.path.join(rospack.get_path("ecto_rbo_yaml"), "data/", x),
                [args.yaml_file]
                )

    ecto_plasm, ecto_cells = plasm_yaml_factory.load(yml_file, debug=args.debug)

    rospy.loginfo("Plasm initialized: %s" % (yml_file))

    if args.showgraph:
        ecto.view_plasm(ecto_plasm, yml_file if type(yml_file) == str else " ".join(yml_file))

    #while not rospy.is_shutdown():

    server = VisionServer(ecto_plasm, ecto_cells)

    if args.service:
        run_sub = rospy.Service('computeECGraph', ComputeECGraph, server.handle_compute_ec_graph)
        rospy.spin()
    else:
        server.ecto_scheduler.execute()

    print server.ecto_scheduler.stats()
