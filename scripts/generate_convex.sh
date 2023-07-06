#!/bin/bash

# refering to https://github.com/isri-aist/mc_fetch_description/blob/main/scripts/generate_convex.sh
exit_if_error()
{
  if [ $? -ne 0 ]
  then
    echo "-- FATAL ERROR: $1"
    exit 1
  fi
}

# set configuration variables
export robot_name="go1"
export robot_desc_name="mc_${robot_name}_description"
export target_pkg_name="mc_${robot_name}_description"

export org_path=`rospack find ${robot_desc_name}`       # original robot_description package path (assuming this has dae mesh files)
export tmp_path="/tmp/generate_${target_pkg_name}"      # tmp_path were the files are generated
export gen_path="/tmp/${target_pkg_name}"               # path were the robot_description package gets generated

export sample_points=2000                   # Number of points to sample on each mesh (used for convex hull generation)

echo "Running generate_convex.sh script from directory `pwd`"

function generate_convexes()
{
    # List target mesh files
    daefiles=`find ${org_path}/meshes/ -type f -regex ".*dae$"`
    stlfiles=`find ${org_path}/meshes/ -type f -regex ".*STL$" ! -regex ".*_collision\.STL$"` # exclude *_collision.STL because they are duplicate with dae files
    targets="${daefiles} ${stlfiles}"
    echo ${targets}

    # Generate scaling_xyz executable
    scaling_xyz="tool/scaling_xyz"
    if [ ! -e ${scaling_xyz} ]; then
      tool/make.sh
    fi
    
    # Generate convexes (convert to qhull's pointcloud and compute convex hull file)
    for mesh in ${targets}
    do
        mesh_name=`basename -- "$mesh"`
        mesh_name="${mesh_name%.*}"
        echo "-- Generating convex hull for ${mesh}"
        mkdir -p ${tmp_path}/qc/${robot_name}
        mkdir -p ${gen_path}/convex/${robot_name}
        gen_cloud=${tmp_path}/qc/${robot_name}/$mesh_name.qc
        gen_convex=${gen_path}/convex/${robot_name}/${mesh_name}-ch.txt
        mesh_sampling ${mesh} ${gen_cloud} --type xyz --samples ${sample_points}
        exit_if_error "Failed to sample pointcloud from mesh ${mesh} to ${gen_cloud}"
        ${scaling_xyz} ${mesh} ${gen_cloud}
        exit_if_error "Failed to scaling process of ${gen_cloud}"
        qconvex TI ${gen_cloud} TO ${gen_convex} Qt o f
        exit_if_error "Failed to compute convex hull pointcloud from point cloud ${gen_cloud} to ${gen_convex}"
    done
}

generate_convexes

echo
echo "Successfully generated convex from ${robot_desc_name} package in ${gen_path}"

