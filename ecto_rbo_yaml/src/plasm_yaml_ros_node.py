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
from pregrasp_msgs.srv import ComputeECGraph
from pregrasp_msgs import srv


def handle_compute_ec_graph(req):
    """Callback running the vision pipeline.

    Args:
        req: The request of the ComputeECGraph service.

    Returns:
        The response of the service defined in ComputeECGraph
    """
    try:
        return run_vision(req)
    except Exception as e:
        rospy.logerr("!!! Object recognition crashed !!!")
        rospy.logerr(e)

        return


def run_vision(req):
    """Request callback for running vision
    """

    global ecto_plasm, ecto_cells, ecto_scheduler
    global tf_listener
    rospy.loginfo("!!! running vision !!!")
    # start scheduler; iterate exactly once over the ecto plasm
    ecto_scheduler.execute(niter=1)

    return srv.ComputeECGraphResponse()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml_file', type=str)
    parser.add_argument('--debug', action='store_true', default=False)
    parser.add_argument('--showgraph', action='store_true', default=False)
    parser.add_argument('--service', action='store_true', default=False)
    myargv = rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(myargv)

    # rospy.loginfo("Do mockup: %s" % (DO_MOCKUP))
    # rospy.loginfo("Record data: %s" % (RECORD_DATA))

    rospy.init_node('plasm_yaml_ros_service')
    
    ecto_ros.init(myargv, 'plasm_yaml_ros_service', anonymous = False)

    rospack = rospkg.RosPack()
    if any([args.yaml_file.startswith(x) for x in [".", "/"]]):
        yml_file = [args.yaml_file]
    else:
        yml_file = map (lambda x: os.path.join(rospack.get_path("ecto_rbo_yaml"), "data/", x),
                # ["input_asus.yaml", "cropping.yaml", "segment_detect_regiongrowing.yaml"])
                # ["input_cropping_pcd.yaml", "segment_detect_regiongrowing.yaml"])                
                # ["input_cropping_pcd.yaml", "segment_detect_bayesian.yaml"]
                # ["input_cropping_bag.yaml", "segment_detect_bayesian.yaml"]
                # ["input_cropping_bag.yaml", "save_images_and_masks.yaml"]
                [args.yaml_file]
                )

    global ecto_plasm, ecto_cells, ecto_scheduler, tf_listener
    ecto_plasm, ecto_cells = plasm_yaml_factory.load(yml_file, debug=args.debug)

    ################ 
    # workaround to do initialization stuff (loading PCD model, building KD tree) 
    # before the first iteration of the full plasm
#    ecto_plasm_static = ecto.Plasm()
    # some fake real data to feed into the 'distances_to_shelf' cell
    #dummy_pcd_reader = ecto_rbo_pcl.PCDReader(filename=os.path.join(rospack.get_path("object_recognition"), "../data/pointcloud_dummy.pcd"))

    #ecto_plasm_static.connect([
        # trigget the PCD reader
        # ecto_cells['pcd_reader']['output'] >> dummy_converter[:],
    #    dummy_pcd_reader['output'] >> ecto_cells['distances_to_shelf']['input'],
    #    ecto_cells['pcd_reader']['output'] >> ecto_cells['distances_to_shelf']['model_pc'],
    #    ])
#    ecto_scheduler = ecto.Scheduler(ecto_plasm_static)
#    ecto_scheduler.execute(niter=1)

#    print ecto_scheduler.stats()
#    ################

    rospy.loginfo("Plasm initialized: %s" % (yml_file))

    if args.showgraph:
        ecto.view_plasm(ecto_plasm, yml_file if type(yml_file) == str else " ".join(yml_file))
    
    ecto_scheduler = ecto.Scheduler(ecto_plasm)
    #while not rospy.is_shutdown():

    if args.service:
        run_sub = rospy.Service('computeECGraph', ComputeECGraph, handle_compute_ec_graph)
        rospy.spin()
    else:
        ecto_scheduler.execute()


    print ecto_scheduler.stats()
