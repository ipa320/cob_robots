#!/bin/bash

export MY_CATKIN_WORKSPACE=~/git/care-o-bot
source /etc/cob.bash.bashrc

if [ $(lsb_release -sc) == "trusty" ]; then
    export MY_ROS_DISTRO="indigo"
elif [ $(lsb_release -sc) == "xenial" ]; then
    export MY_ROS_DISTRO="kinetic"
elif [ $(lsb_release -sc) == "focal" ]; then
    export MY_ROS_DISTRO="noetic"
fi

. /opt/ros/$MY_ROS_DISTRO/setup.sh

if [ -e $MY_CATKIN_WORKSPACE/devel/setup.bash ]; then
    source $MY_CATKIN_WORKSPACE/devel/setup.bash
elif [ -e /u/robot/git/care-o-bot/devel/setup.bash ]; then
    source /u/robot/git/care-o-bot/devel/setup.bash
else
    source /opt/ros/$MY_ROS_DISTRO/setup.bash
fi

export ROS_IP=`hostname -I | awk '{print $1}'`

exec "$@"
