#!/bin/bash

export MY_CATKIN_WORKSPACE=~/git/care-o-bot
source /etc/cob.bash.bashrc

. /opt/ros/indigo/setup.sh

if [ -e $MY_CATKIN_WORKSPACE/devel/setup.bash ]; then
    source $MY_CATKIN_WORKSPACE/devel/setup.bash
elif [ -e /u/robot/git/care-o-bot/devel/setup.bash ]; then
    source /u/robot/git/care-o-bot/devel/setup.bash
else
    source /opt/ros/indigo/setup.bash
fi

exec "$@"
