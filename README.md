# ecto_rbo

ROS packages providing various ecto cells and plasms for perception.

### Compile in Ubuntu 18.04 and ROS Melodic

* First, install some dependencies. Start with `pip install requirements.txt` (those include the required packages for all SOMA repos).

* Install CGAL with `sudo apt-get install libcgal-dev`
* Install Wild Magic 5  and GDIAM from the thirdparty folder:
   * Wild Magic 5:
   ```
   cd vision/thirdparty/GeometricTools/WildMagic5
   make CFG=ReleaseDynamic -f makefile.wm5
   export WM5_PATH=/your_path/vision/thirdparty/GeometricTools/WildMagic5/SDK
   ```
   * GDIAM 1.0.1:
   ```
   cd vision/thirdparty/libgdiam
   mkdir build && cd build
   cmake ..
   make
   ```
   * copy the `libgdiam.so` lib from the build folder to `/usr/local/lib` where it is expected by the ecto_rbo package by `sudo cp libgdiam.so /usr/local/lib`.
   
* Install openni2_launch: `sudo apt install ros-melodic-openni2-launch`

* Then, clone the ec_grasp_planner repository: https://github.com/soma-project/ec_grasp_planner and build the geometry_graph_msgs: `catkin build geometry_graph_msgs`

* Install `opencv_candidate`(https://github.com/wg-perception/opencv_candidate).

* Install opencv version 3.2. using the following steps:
    * clone `opencv` from version Tag 3.2.0 (https://github.com/opencv/opencv/tree/3.2.0)
    * clone `opencv_contrib` from version Tag 3.2.0 (https://github.com/opencv/opencv_contrib/tree/3.2.0)
    * build and install using:
    ```
    cd opencv
    mkdir build && cd build
    sudo apt-get install libboost-python-dev libboost-filesystem-dev libboost-system-dev \
        libboost-thread-dev python-setuptools python-gobject python-gtk2 graphviz doxygen \
        python-sphinx
    cmake -D CMAKE_BUILD_TYPE=RELEASE \-D CMAKE_INSTALL_PREFIX=/usr/local \-D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \-D WITH_TBB=ON \-D WITH_V4L=ON \-D WITH_QT=ON \-D WITH_OPENGL=ON \
    -D WITH_CUDA=ON \-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \-D BUILD_EXAMPLES=ON ..
    make -j4
    sudo make install
    ```

* Build `ecto`, `ecto_ros`, `ecto_pcl` from source and install dependencies (http://github.com/plasmodic/ecto.git)

* Build `ecto_opencv` using the fork of that is already migrated to 18.04 (https://github.com/zweistein/ecto_opencv/tree/migration_To_Ubunt18.04)

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

* Build the remaining packages of this repository (`catkin build ecto_rbo`).

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

<img src="readme_/Example1_rgb.png?raw=true" height="250" /> <img src="readme_/Example1_Rviz.png?raw=true" height="250" />

The image on the right shows the depth point cloud, the detected wall/table polygons and the detected closest object centroid in RVIZ. 
The bag file of this scene can be found here: /nas/Videos/Vision/vision_example.bag.

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
# The example bag file is saved on the tub-NAS (as stated above), for access contact a tub-member. 
# use ros sim time
rosparam set use_sim_time true
roslaunch openni2_launch openni2.launch depth_registration:=false
rosbag play -l vision_example.bag  (or any other bag)
# visualize in rviz:
rosrun rviz rviz -d `rospack find ecto_rbo_yaml`/cfg/vision_example.rviz
```

Execute vision
```
rosrun ecto_rbo_yaml plasm_yaml_ros_node.py `rospack find ecto_rbo_yaml`/data/demo_vision.yaml --debug
```

---
If you want to create your own bag file, follow these steps:
1. prepare the scene 
2. start the openni2_camera drivers 
```
roscore
roslaunch openni2_launch openni2.launch depth_registration:=true
```
3. adjust your camera (you can use RVIZ for visualization), then run
```
rosbag record -O recorded_file /camera/depth_registered/points
```
The `-O` argument tells rosbag record to log to a file named recorded_file.bag, and the topic argument causes rosbag record to only subscribe to the topic `/camera/depth_registered/points`. When all topics are recorded it fails sometimes leaving an empty bag file. 

---

## Example 2: Ifco Scene (Table + Ifco Container + Object(s))

Prepare scene:
* Clear a table, place an ifco tote (57.5 x 37.5 x 17.5 cm) on it with horizontal alignment (57.5 cm side of ifco) towards the camera. Place objects inside the tote.


<img src="readme_/IfcoContainerRGB.png?raw=true" height="250" /> <img src="readme_/IfcoContainerRviz.png?raw=true" height="250" />

The image on the right shows the depth point cloud, the detected ifco frame and the detected object centroids in RVIZ. 
The bag file of this scene can be found here: /nas/Videos/Vision/ifco_example.bag.

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
# The example bag file is saved on the tub-NAS (as stated above), for access contact a tub-member. 
# use ros sim time
rosparam set use_sim_time true
roslaunch openni2_launch openni2.launch depth_registration:=false
rosbag play -l ifco_example.bag  (or any other bag)
# visualize in rviz:
rosrun rviz rviz -d `rospack find ecto_rbo_yaml`/cfg/ifco_example.rviz
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

