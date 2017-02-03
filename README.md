# ecto_rbo

ROS packages providing various ecto cells and plasms for perception.

## How to test

Install ros-indigo-ecto-*

PROCESS:
roscore
roslaunch openni2_launch openni2.launch depth_registration:=true  
rosrun rqt_reconfigure rqt_reconfigure [choose 8]
rviz -d demo.rviz
rosrun ecto_rbo_yaml plasm_yaml_ros_node.py demo_vision.yaml --debug
rosrun ecto_rbo_yaml ecto_yaml_to_pdf.py ecto_rbo_yaml/data/demo_vision.yaml

## How to create documentation

You can generate documentation for each package using sphinx:

```
sudo pip install catkin_sphinx
roscd ecto_rbo_pcl
sphinx-build -b html ./doc/source/ ./doc/build
firefox ./doc/build/index.html
```

You can do this for all packages that are part of ecto_rbo.

