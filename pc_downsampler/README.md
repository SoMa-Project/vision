# POINT CLOUD DOWNSAMPLER

A ROS node to downsample a generic resolution point cloud (sensor_msgs::PointCloud2) taken from a specified topic (e.g. /kinect2/sd/points) to a any lower resolution (x_res * y_res) keeping the PointCloud2 organized. The node pubishes the resulting cloud (sensor_msgs::PointCloud2) to another specified topic.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

The only dependencies are ROS Indigo or newer and PCL.

### Installing

To install this package just clone and catkin build.

## Running the downsampler 

### For UNIPI setup (Kinect One) 

Launch the pc_downsampler node (this downsamples the point cloud taken from /kinect2/sd/points to a QVGA resolution and publishes to /down_sampled_points).

```
roslaunch pc_downsampler pc_downsampler_kinect2.launch
```

### For other setups

```
rosrun pc_downsampler pc_downsampler input_topic:=/your_input_topic output_topic:=/your_output_topic
```

Or please create a new launch file.

### To set different output resolution

Modify the following lines in src/main.cpp :

```
int x_res = 320;
int y_res = 240;
```

This might be modified in a following version.
