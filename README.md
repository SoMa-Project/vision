# ecto_rbo

ROS packages providing various ecto cells and plasms for perception.

## Install 

* First compile the ec_grasp_planner repository: https://github.com/soma-project/ec_grasp_planner

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

You will also have to install Wild Magic 5 from the thirdparty folder. In `thirdparty/GeometricTools/WildMagic5` execute:
```
make CFG=ReleaseDynamic -f makefile.wm5
```
And export the respective WP5_PATH
```
export WM5_PATH=/your_path/vision/thirdparty/GeometricTools/WildMagic5/SDK
```

You will also need to install GDIAM 1.0.1 from the thirdparty folder. In `thirdparty/libgdiam` execute:
```
mkdir build && cd build
cmake ..
make
```
copy the `libgdiam.so` lib from the build folder to `/usr/local/lib` where it is expected by the ecto_rbo package by `sudo cp libgdiam.so /usr/local/lib`.


* For a particular grasping scenery (grasping out of an ifco) the vision depends on another repository ifco_pose_estimator: https://github.com/SoMa-Project/ifco_pose_estimator.git

## Examples
In the following you can try the vision repository on two szenarios shown underneath:

## Example 1: General Scene (Table + Wall + object)
Prepare scene:
* Clear a table, place an apple on it, and a wall (a rectangular prism object with height > 15 cm).Table + wall + object


TODO: table top + wall


Launch camera or play bag file
* Plug in a rgb-d camera or download and launch the example .bag file.

```
# with kinect: plug the camera into your computer
roslaunch openni2_launch openni2.launch depth_registration:=true
# set camera resolution to QVGA
rosrun dynamic_reconfigure dynparam set /camera/driver ir_mode 7
rosrun dynamic_reconfigure dynparam set /camera/driver color_mode 7
rosrun dynamic_reconfigure dynparam set /camera/driver depth_mode 7


# with .bag file: 
# example of table top scenario (link follows)
# use ros sim time
rosparam set use_sim_time true
roslaunch openni2_launch openni2.launch depth_registration:=false
rosbag play -l xxx.bag  (or any other bag)
```

Execute vision
```
rosrun ecto_rbo_yaml plasm_yaml_ros_node.py `rospack find ecto_rbo_yaml`/data/demo_vision.yaml --debug
```

## Example 2: Ifco Scene (Table + Ifco Container + Object(s))

Prepare scene:
* Clear a table, place an ifco tote (57.5 x 37.5 x 17.5 cm) on it with horizontal alignment (57.5 cm side of ifco) towards the camera. Place an apple inside the tote.


![Alt text](/readme_/IfcoContainerScene.png?raw=true "Title")

Choose IFCO detection method 
* Choose one of the IFCO detection methods by setting the rosparam to one of 1/2/3,  default is 1

```
# detection_method=1 for normal estimation based plane ifco detection
rosparam set detection_method 1
# detection_method=2 for static ifco transform
rosparam set detection_method 2
roslaunch ecto_rbo_pcl staticTFforIfco.launch
# detection_method=3 for ICP ifco detection
rosparam set detection_method 3
```

Launch camera or play bag file
* Plug in a rgb-d camera or download and launch the example .bag file.

```
# with kinect: plug the camera into your computer
roslaunch openni2_launch openni2.launch depth_registration:=true
# set camera resolution to QVGA
rosrun dynamic_reconfigure dynparam set /camera/driver ir_mode 7
rosrun dynamic_reconfigure dynparam set /camera/driver color_mode 7
rosrun dynamic_reconfigure dynparam set /camera/driver depth_mode 7


# with .bag file: 
# example of ifco bag https://tubcloud.tu-berlin.de/s/yKQrraTdSsb54TC
# example of table top scenario (link follows)
# use ros sim time
rosparam set use_sim_time true
roslaunch openni2_launch openni2.launch depth_registration:=false
rosbag play -l Ifco_vision_test.bag  (or any other bag)
```

Execute vision
```
rosrun ecto_rbo_yaml plasm_yaml_ros_node.py `rospack find ecto_rbo_yaml`/data/demo_ifco.yaml --debug
```


## Documentation 

You can generate documentation for each package using sphinx:

```
sudo pip install catkin_sphinx
roscd ecto_rbo_pcl
sphinx-build -b html ./doc/source/ ./doc/build
firefox ./doc/build/index.html
```

You can do this for all packages that are part of ecto_rbo.

