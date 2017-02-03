#!/usr/bin/env python

import roslib; roslib.load_manifest('ecto_rbo_pcl')
import rospy

import sys
import os
import os.path
import time

try:
  from ecto_rbo_pcl_py.PassThroughXYZ import PassThroughXYZ
except:
  print "[WARN] Could not find ecto_rbo_ml module. Did you install the ROS package correctly?"
  # probably you did not install the scripts correctly
  # as we only use python scripts here we can directly use them
  # append src folder to import ecto_rbo_ml python modules
  sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
  from ecto_rbo_pcl_py.PassThroughXYZ import PassThroughXYZ

import ecto
import ecto_pcl
import ecto_rbo_pcl

################################################################################
# this is the name of the pcd file we will load
pcdfile = os.path.join(os.path.dirname(__file__),
                       'bunny_original.pcd')

pcdfile_out = os.path.join(os.path.dirname(__file__),
                       'bunny_cropped_%04d.pcd')

x_min, x_max = 0, 1 
y_min, y_max = -5, 5
z_min, z_max = -0.7, 1.6

################################################################################

plasm = ecto.Plasm()

# PCD reader for input data
reader = ecto_pcl.PCDReader("PCD input reader", filename=pcdfile)
pt = PassThroughXYZ(keep_organized=True,
                      x_min=x_min,x_max=x_max,
                      y_min=y_min,y_max=y_max,
                      z_min=z_min,z_max=z_max,
                      )
writer = ecto_pcl.PCDWriter("PCD output writer", 
                    filename_format=pcdfile_out)

viewer = ecto_pcl.CloudViewer("viewer",
                              window_name="PCD Viewer")

plasm.connect(
              [
               reader[:] >> pt[:],
               pt['output'] >> writer['input'],
               pt['output'] >> viewer['input']
               ]
              )

if __name__=="__main__":
    # Instantiate a schedule for the plasm
    sched = ecto.Scheduler(plasm)
    # Start and iterate once
    sched.execute(niter=1)
    #sleep 5 seconds and exit.
    time.sleep(5)
