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
from geometry_graph_msgs.msg import ObjectList
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Transform
from pregrasp_msgs.srv import ComputeECGraph
from pregrasp_msgs import srv

import tf
from tf import transformations as tra

class VisionServer:
    def __init__(self, ecto_plasm, ecto_cells):
        self.ecto_plasm = ecto_plasm
        self.ecto_cells = ecto_cells
        self.ecto_scheduler = ecto.Scheduler(self.ecto_plasm)

        rospy.Subscriber('geometry_graph', Graph, self.callback_vision_result_graph)
        rospy.loginfo('Subscribed to the geometry_graph topic.')

        rospy.Subscriber('objects', ObjectList, self.callback_vision_result_objects)
        rospy.loginfo('Subscribed to the objects topic.')

    def handle_compute_ec_graph(self, req):
        """Callback running the vision pipeline.
    
        Args:
            req: The request of the ComputeECGraph service.
    
        Returns:
            The response of the service defined in ComputeECGraph
        """
        try:
            return self.run_vision(req)
        except rospy.ServiceException as e:
            rospy.logerr("Vision: %s" % e)
            raise e
        except Exception as e:
            rospy.logerr("!!! vision crashed !!!")
            rospy.logerr(e)
            raise rospy.ServiceException("Vision crashed: %s" % e)

    def callback_vision_result_graph(self, data):
        self.outputgraph = data

    def callback_vision_result_objects(self, data):
        self.found_objects = data

    def run_vision(self, req):
        """Request callback for running vision
        """

        rospy.loginfo("!!! running vision !!!")
        self.outputgraph = None
        self.found_objects = None

        # start scheduler; iterate exactly once over the ecto plasm
        self.ecto_scheduler.execute(niter=1)

        start_time = rospy.now()
        timeout = rospy.Duration(60.0) # Break if vision takes longer than 1min.
        while self.outputgraph is None or self.found_objects is None:
            if start_time < start_time + timeout:
                rospy.sleep(0.1)
            else:
                raise rospy.ServiceException("Vision service call timeout (execution longer than {0}s)".format(timeout.secs))

        return srv.ComputeECGraphResponse(self.outputgraph, self.found_objects)

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
        run_sub = rospy.Service('compute_ec_graph', ComputeECGraph, server.handle_compute_ec_graph)
        rospy.spin()
    else:
        server.ecto_scheduler.execute()

    print(server.ecto_scheduler.stats())
