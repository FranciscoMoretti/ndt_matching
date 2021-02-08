## Install ROS2
See: https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/.  
**Note:** Stop at "Install additional RMW implementations".

## Clone, build and run this package
```sh
cd
mkdir ndt_ws/src -p
cd ndt_ws/src
git clone https://github.com/dejanpan/ndt_matching.git
cd ..
source /opt/ros/dashing/setup.bash
colcon build
./build/ndt_matching/ndt_node
```

## Organization
The package contains an [executable](src/ndt_node.cpp) and a [library](src/ndt_lib.cpp). The library should contain
an implemented algorithm and the executable, a ROS2 node, should subscribe to two
PointCloud2 messages and publishes one PoseStamped message.

Two PointCloud2 messages will be played back by the [rosbag2](https://github.com/ros2/rosbag2) tool and can be
together with the Pose message visualized in [rviz2](https://github.com/ros2/rviz/tree/crystal) tool.

## Demo steps
For each step use a different terminal.

### Start roscore
```
/opt/ros/melodic/setup.bash
roscore
```

### Start rosbridge
Build rosbrige and then run it from it's src directory.
```
source ~/ndt_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Start the node
Unless it is already running from a previous step
```
source /opt/ros/dashing/setup.bash
./build/ndt_matching/ndt_node
```

### Run RViz
```
source /opt/ros/melodic/setup.bash
cd ~/ndt_ws/src/ndt_matching
rviz -d ndt_config.rviz
```

### Publish the pcd map
It'll be published every 10 seconds. And wait until the node says it could read it.
```
source /opt/ros/melodic/setup.bash
rosrun pcl_ros pcd_to_pointcloud map.pcd 10 _frame_id:=map cloud_pcd:=map
```
The map should be visible in RViz as a PointCloud2.

![How the map should look in RViz](pictures/pointcloud_map.png?raw=true "Map in RViz")

### Play the rosbag
Reproduce the rosbag so that the vehicle starts receiving inputs from it's sensors.
For this step use a lower rate so that the algorithm has time to process the data and doesn't lays behind of the vehicle. Sorry for the low rate, 
the first time it worked at 0.1. It's in dire need of some optimization!
```
 source /opt/ros/melodic/setup.bash
 rosbag play lidar_data.bag -r 0.01 /filtered_points:=/points_raw /localizer_pose:=/initial_pose
```

## Result
The demo will show an red arrow which is published by this node on `/pose_estimation` following a grey arrow that is the rosbag data on `/ndt_pose`.

![Lidar data and two poses in RViz](pictures/starting_pose.png?raw=true "Demo in RViz")
