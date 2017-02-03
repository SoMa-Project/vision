#!/usr/bin/env python

import ecto, ecto_pcl
import ecto_rbo_pcl
import sys
import time
import os

plasm = ecto.Plasm()

# this is the name of the pcd file we will load
pcdfile = os.path.join(os.path.dirname(__file__),
                       'bunny_original.pcd')

# alternatively read a pcd file from sys in
if len(sys.argv) > 1:
    pcdfile = sys.argv[1]

# instantiate PCDReader cell with the given filename
reader = ecto_pcl.PCDReader("Reader",
                            filename=pcdfile)

# instantiate the cloud viewer cell
viewer = ecto_pcl.CloudViewer("viewer",
                              window_name="PCD Viewer")

# instantiate the normal estimation. we use radius search with 0.1
normals = ecto_rbo_pcl.NormalEstimation("normals",
                              radius_search=0.1)

# instantiate the PCDCacher. We set use_feature_input to true
# to indicate that we are going to store normals. The filename
# contains a %04u because you can store several subsequent point clouds
# which will be enumerated (useful for online processing)
cacher = ecto_rbo_pcl.PCDCacher("cacher", 
                              filename_format="bunny_cached_%04u.pcd",
                              use_feature_input=True)

# CONNECt the plasm
plasm.connect(
  [
    reader[:] >> normals["input"],
    reader[:] >> normals["search_surface"], # you can also specify a smaller search surface; we use the whole input
    normals["output"] >> cacher["feature_input"],
    reader[:] >> viewer[:]
  ]
)

if __name__=="__main__":
    # Instantiate a schedule for the plasm
    sched = ecto.Scheduler(plasm)
    # Start and iterate once
    sched.execute(niter=1)
    #sleep 5 seconds and exit.
    time.sleep(5)
