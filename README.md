# ecto_rbo

ROS packages providing various ecto cells and plasms for perception.

## Install 

* Install all ROS packages that start with 'ros-indigo-ecto', as well as with 'openni2':
```
sudo apt-get install ros-indigo-ecto* ros-indigo-openni*
```


* Compile this package. CAUTION: This step can take up to 30 minutes and might freeze your computer. It's best to do it overnight or over lunchbreak.
```
catkin build ecto_rbo
```

* To compile the package ecto_rbo_grasping you will have to solve a list of dependencies:

First install CGAL:
```
sudo apt-get install libcgal-dev
```

You will also have to install Wild Magic from this [link](https://tubcloud.tu-berlin.de/s/oCwzZnOAMSZN46c). Download and execute:
```
make CFG=ReleaseDynamic -f makefile.wm5
```
And export the respective WP5_PATH
```
export WM5_PATH=/your_path/GeometricTools/WildMagic5/SDK
```

You will also need to install GDIAM. Download the old version libgdiam-1.01.tar.gz from (https://tubcloud.tu-berlin.de/s/4I8Txq4dJ7653HC) and follow the building instructions:
```
tar -xzf libgdiam-1.0.1.tar.gz 
~/$ cd libgdiam/
~/libgdiam$ mkdir build
~/libgdiam$ cd build/
~/libgdiam/build$ cmake ..
~/libgdiam/build$ make test
```
copy the libgdiam.so lib from the build folder to /usr/local/lib where it is expected by the ecto_rbo package.



## Example

```
roslaunch openni2_launch openni2.launch depth_registration:=true
# [go to camera, driver, and choose 8]
rosrun rqt_reconfigure rqt_reconfigure
rviz -d demo.rviz
rosrun ecto_rbo_yaml plasm_yaml_ros_node.py demo_vision.yaml --debug
#(not necessary for test but can help understanding)
rosrun ecto_rbo_yaml ecto_yaml_to_pdf ecto_rbo_yaml/data/demo_vision.yaml 
#(in some cases the above one is not working, then try this) 
rosrun ecto_rbo_yaml plasm_yaml_ros_node.py `rospack find ec_grasp_planner`/data/geometry_graph_example3.yaml --show 

```

Expected outcome:
* Clear a table, place an apple on it, and a wall (a rectangular prism object with height > 15 cm).
* You should see frames in the centroids of the table, wall, and the object.

## Documentation 

You can generate documentation for each package using sphinx:

```
sudo pip install catkin_sphinx
roscd ecto_rbo_pcl
sphinx-build -b html ./doc/source/ ./doc/build
firefox ./doc/build/index.html
```

You can do this for all packages that are part of ecto_rbo.

