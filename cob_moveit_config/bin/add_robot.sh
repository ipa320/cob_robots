#!/bin/bash

robot=$1

base=`rospack find cob_moveit_config`
mkdir -p $base/$robot/config

cat <<SRDF_TEMPLATE  | sed "s/ROBOT/$robot/g" > $base/$robot/config/$robot.srdf
<robot name="ROBOT">
</robot>
SRDF_TEMPLATE

cat <<CONFIG_TEMPLATE  | sed "s/ROBOT/$robot/g" > $base/$robot/.setup_assistant
moveit_setup_assistant_config:
  URDF:
    package: cob_hardware_config
    relative_path: ROBOT/urdf/ROBOT.urdf.xacro
  SRDF:
    relative_path: config/ROBOT.srdf
CONFIG_TEMPLATE
