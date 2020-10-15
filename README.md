# ecto_rbo

ROS packages providing various ecto cells and plasms for perception.

### Compile in Ubuntu 18.04 and ROS Melodic

* First, install some dependencies. Start with installing CGAL:
```
sudo apt-get install libcgal-dev
```

You will also have to install Wild Magic 5 from the thirdparty folder. In `vision/thirdparty/GeometricTools/WildMagic5` execute:
```
make CFG=ReleaseDynamic -f makefile.wm5
```
And export the respective WP5_PATH
```
export WM5_PATH=/your_path/vision/thirdparty/GeometricTools/WildMagic5/SDK
```

You will also need to install GDIAM 1.0.1 from the thirdparty folder. In `vision/thirdparty/libgdiam` execute:
```
mkdir build && cd build
cmake ..
make
```
copy the `libgdiam.so` lib from the build folder to `/usr/local/lib` where it is expected by the ecto_rbo package by `sudo cp libgdiam.so /usr/local/lib`.

* Then, compile the ec_grasp_planner repository: https://github.com/soma-project/ec_grasp_planner, build the geometry_graph_msgs:

`catkin build geometry_graph_msgs`
* Install `opencv_candidate`(https://github.com/wg-perception/opencv_candidate).

* Install `opencv` v3.2.0 and `opencv_contrib` v3.2.0:
```
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \-D CMAKE_INSTALL_PREFIX=/usr/local \-D INSTALL_C_EXAMPLES=ON \-D INSTALL_PYTHON_EXAMPLES=ON \-D WITH_TBB=ON \-D WITH_V4L=ON \-D WITH_QT=ON \-D WITH_OPENGL=ON \-D WITH_CUDA=ON \-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \-D BUILD_EXAMPLES=ON ..
```

* Build `ecto`, `ecto_ros`, `ecto_pcl` from source and install dependencies (http://github.com/plasmodic/ecto.git),
use fork of `ecto_opencv` that is already migrated to 18.04: https://github.com/zweistein/ecto_opencv/tree/migration_To_Ubunt18.04

```
$ sudo apt-get install libboost-python-dev libboost-filesystem-dev libboost-system-dev \
        libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen \
        python-sphinx
# remove flag if it is not required to istall it
$ catkin build -j 4 -DCMAKE_INSTALL_PREFIX=/usr/local
````
If compilation failes due to `tr1`:
`": fatal error: boost/tr1/unordered_map.hpp: No such file or directory #include <boost/tr1/unordered_map.hpp>"`

Remove `tr1` and replace `std::tr1::unordered_map` with `boost::unordered_map` in each files:
1, entered to find these two files using the following command ecto directory
```
$ grep -r 'tr1'
./src/lib/util.cpp:#include <boost/tr1/unordered_map.hpp>
./src/lib/util.cpp:typedef std::tr1::unordered_map<std::string, std::string> dict_t;
./src/lib/plasm/impl.hpp:#include <boost/tr1/unordered_map.hpp>
```

* Build the remaining packages of this repository.

## Install (Ubunut 14.04)

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

You also need to build geometry_graph_msgs from https://github.com/SoMa-Project/ec_grasp_planner.git:
`catkin build build geometry_graph_msgs`

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

