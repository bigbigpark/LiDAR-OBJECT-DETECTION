# LiDAR-OBJECT-DETECTION

Simple algorithm to detect the object of road environment using 3D LiDAR <br/>

<br/>

## Tested Environment

- Ubuntu 18.04
- ROS Melodic

<br/>

## Summary

* Subscribe `sensor_msgs/PointCloud2` msg
* Curb Detection from the point cloud
* Publish bounding box(`visualization_msgs/MarkerArray`)

![](/lidar-object.gif)

<br/>

## How to use

Clone, build and run

~~~bash
$ git clone https://github.com/bigbigpark/LiDAR-OBJECT-DETECTION.git
~~~

~~~bash
$ catkin build
~~~

~~~bash
$ roslaunch lidar_object_detection object_detect.launch
~~~

<br/>

## Parameter configuration

You can easily modify topic name of `sensor_msgs/PointCloud2` by changing **object_detect.launch** <br/>

~~~xml
<node name="object_detection" pkg="lidar_object_detection" type="object_detection" output="screen" respawn="true">
    <remap from="/input" to="/os_cloud_node/points"/>
</node>
~~~

Here, change your topic <br/>

<br/>

## TODO

- [ ] Robust object detection
- [ ] Get orientation of the object