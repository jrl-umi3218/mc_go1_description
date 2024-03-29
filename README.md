# Go1 robot description for [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
This package has additional convex and rsdf files for mc_go1 robot module used by [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/) control framework.
go1 robot has urdf and meshes in the original [go1_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go1_description) package.

It contains the following directories:
 - `convex/`: convex hulls (generated from pointclouds sampled from the dae meshes)
 - `rsdf/`: Special urdf-like format describing surfaces attached to links on the robot
 - `scripts/`: This folder has a script that generates urdf and convex

## Installation

On an environment with ROS and catkin properly setup:

```
$ cd ~/catkin_ws/src
$ svn export https://github.com/unitreerobotics/unitree_ros/trunk/robots/go1_description


$ git clone 
$ cd 

$ cd ~/catkin_ws
$ catkin_make
$ cd build
$ make install
```

If your catkin environment is sourced `source ~/catkin_ws/install/setup.bash`, the robot model will be available to all ROS tools, and mc_rtc robot module.

To display the robot, you can use:

```
$ roslaunch mc_go1_description display_go1.launch
```

 - If you have mc_rtc and the corresponding robot module installed, you can use the `convexes:=True` or `surfaces:=True` arguments to display the robot convexes and surfaces.

***

## When updating convex files

### Dependencies

 - [go1_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go1_description)
 - [mesh_sampling](https://github.com/jrl-umi3218/mesh_sampling)
 - qhull-bin

### Generation of convex files
To run the conversion, simply run

```
$ source ~/catkin_ws/devel/setup.bash
$ cd ~/catkin_ws/src/mc_go1_description/scripts
$ ./generate_convex.sh ~/catkin_ws/src/go1_description
  Running generate_convex.sh script from directory ~/catkin_ws/src/mc_go1_description/scripts
      :
      :
  Successfully generated convex from fetch_description package in /tmp/mc_go1_description
$ mv /tmp/mc_go1_description/convex/go1 ~/catkin_ws/src/mc_go1_description/convex/.
```
