`cob_bringup` is a package that collects together the scripts,
[roslaunch](https://index.ros.org/p/roslaunch) files, and dependencies
that are required to bring a Care-O-bot robot into a running state.

## Hardware Requirements

To use this package you need a real Care-O-bot, for a simulated one use
[cob_bringup_sim](https://index.ros.org/p/cob_bringup_sim).

### Starting a real Care-O-bot

For further information how to operate Care-O-bot, please refer to user
chapter in the
[Care-O-bot Manual](https://github.com/ipa320/setup/raw/master/manual/Care-O-bot_manual.pdf).

Before running, you have to
[login to the robot](Robots/Care-O-bot/Real robot Tutorials/Login to the robot).


```
export ROBOT=cob4-2
export ROBOT_ENV=ipa-apartment
roslaunch cob_bringup robot.launch
```
