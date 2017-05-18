#!/bin/bash

robot=$1

base=`rospack find cob_moveit_config`
mkdir -p $base/robots/$robot/moveit/config

cat <<SRDF_TEMPLATE  | sed "s/ROBOT/$robot/g" > $base/robots/$robot/moveit/config/$robot.srdf
<robot name="ROBOT">
</robot>
SRDF_TEMPLATE

cat <<CONFIG_TEMPLATE  | sed "s/ROBOT/$robot/g" > $base/robots/$robot/moveit/.setup_assistant
moveit_setup_assistant_config:
  URDF:
    package: cob_hardware_config
    relative_path: robots/ROBOT/urdf/ROBOT.urdf.xacro
  SRDF:
    relative_path: config/ROBOT.srdf
CONFIG_TEMPLATE
