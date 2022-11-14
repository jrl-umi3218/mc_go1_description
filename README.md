# Aliengo robot description for [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
This package has additional convex and rsdf files for mc_aliengo robot module used by [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/) control framework.
Aliengo robot has urdf and meshes in the original [aliengo_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description) package.

It contains the following directories:
 - `convex/`: convex hulls (generated from pointclouds sampled from the dae meshes)
 - `rsdf/`: Special urdf-like format describing surfaces attached to links on the robot
 - `scripts/`: This folder has a script that generates urdf and convex

## Installation

On an environment with ROS and catkin properly setup:

```
$ cd ~/catkin_ws/src
$ svn export https://github.com/unitreerobotics/unitree_ros/trunk/robots/aliengo_description
    (or '$ git clone https://github.com/unitreerobotics/unitree_ros.git')
$ git clone https://github.com/mc_aliengo_description
$ cd mc_aliengo_description/scripts
$ python ./generate_urdf.py @ALIENGO_DESCRIPTION_PATH@/urdf/aliengo.urdf
$ cd ~/catkin_ws
$ catkin_make
$ cd build
$ make install
```

If your catkin environment is sourced `source ~/catkin_ws/install/setup.bash`, the robot model will be available to all ROS tools, and mc_rtc robot module.

To display the robot, you can use:

```
$ roslaunch mc_aliengo_description display_aliengo.launch
```

 - If you have mc_rtc and the corresponding robot module installed, you can use the `convexes:=True` or `surfaces:=True` arguments to display the robot convexes and surfaces.

***

## When updating convex files

### Dependencies

 - [aliengo_description](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/aliengo_description)
 - [mesh_sampling](https://github.com/arntanguy/mesh_sampling)
 - qhull-bin

### Generation of convex files
To run the conversion, simply run

```
$ source ~/catkin_ws/devel/setup.bash
$ cd ~/catkin_ws/src/@MC_ALIENGO_DESCRIPTION_PATH@/scripts
$ ./generate_convex.sh /home/@USERNAME@/catkin_ws/src/@ALIENGO_DESCRIPTION_PATH@
  Running generate_convex.sh script from directory /home/@USERNAME@/catkin_ws/src/@MC_ALIENGO_DESCRIPTION_PATH@/scripts
      :
      :
  Successfully generated convex from fetch_description package in /tmp/mc_aliengo_description
$ mv /tmp/mc_aliengo_description/convex/aliengo ~/catkin_ws/src/@MC_ALIENGO_DESCRIPTION_PATH@/convex/.
```
