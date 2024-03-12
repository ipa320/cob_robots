# Copyright 2024 Fraunhofer IPA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Nadia Hammoudeh Garcia

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameters_type import ParameterValue
import os
import xacro
import subprocess

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            description="Robot name.",
            choices=["cob4-25"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pkg_hardware_config",
            default_value="cob_hardware_config",
            description="Name of the package that contains the robot configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=[LaunchConfiguration("robot"),".urdf.xacro"],
            description="File containing the robot description.",
        )
    )
   
    # Initialize Arguments
    robot = LaunchConfiguration("robot")
    pkg_hardware_config = LaunchConfiguration("pkg_hardware_config")
    description_file = LaunchConfiguration("description_file")

    xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # xacro_command =  [
    #         "xacro",
    #         PathJoinSubstitution([FindPackageShare(pkg_hardware_config), "robots", robot, "urdf", description_file]),
    #     ]
    # robot_description = {"robot_description": Command(xacro_command)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_hardware_config), "robots", robot, "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
