## Load the parameters

Inside this package there is the launch to load the parameters in
parameter server in the name space /script_server , for example
upload_param_cob3-3.launch:



```
<?xml version="1.0"?>
<launch>

        <!-- send ROBOT parameters to parameter server -->
        <rosparam command="load" ns="/script_server/tray" file="$(find cob_default_robot_config)/$(env ROBOT)/tray_joint_configurations.yaml"/>
        <rosparam command="load" ns="/script_server/torso" file="$(find cob_default_robot_config)/$(env ROBOT)/torso_joint_configurations.yaml"/>
        <rosparam command="load" ns="/script_server/head" file="$(find cob_default_robot_config)/$(env ROBOT)/head_joint_configurations.yaml"/>
        <rosparam command="load" ns="/script_server/arm" file="$(find cob_default_robot_config)/$(env ROBOT)/arm_joint_configurations.yaml"/>
        <rosparam command="load" ns="/script_server/sdh" file="$(find cob_default_robot_config)/$(env ROBOT)/sdh_joint_configurations.yaml"/>
        <rosparam command="load" ns="/script_server/light" file="$(find cob_default_robot_config)/$(env ROBOT)/light_configurations.yaml"/>
        <rosparam command="load" ns="/script_server/sound" file="$(find cob_default_robot_config)/$(env ROBOT)/sound_configurations.yaml"/>

</launch>
```



## Parameter definition/ Examples

The package is divided into folders, a folder per robot, inside each
robot folder you can find the `.yaml`-files.

The `tray_joint_configurations.yaml`-file could look similar to this:



```
joint_names: ["torso_tray_joint"]

up: [[0.0]]
down: [[-3.1415926]]
```



Each entry is defined as a list of lists. The inner list contains all
joint positions for a certain point. The outer list contains all points
belonging to a certain trajectory. All commands are trajectories with
one or more points.

Here you can see an example of a `arm_joint_configurations.yaml` file
with points and trajectories: 

```
joint_names: ["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]

# single positions
home: [[0,0,0,0,0,0,0]]
folded: [[-1.16, -1.91, -2.53, -1.78, -2.84, -0.97, 1.76]]
pregrasp: [[-1.3813, -1.9312, -1.7251, -1.4565, 0.7169, 1.0560, -2.1230]]
wavein: [[1.5, 0.5, 0.0, -0.5, 0.0, 0.5, 0.0]]
waveout: [[1.5, 0.5, 0.0, 0.5, 0.0, -0.5, 0.0]]

# trajectories
wave: [waveout, wavein, waveout, wavein]
```



## Command_gui_buttons / Example

Also it is a .yaml file with the buttons of the command_gui
(cob_default_robot_config)/\$(env ROBOT)/command_gui_buttons.yaml it is
defined for each robot. The typical format of this file is:



```
group:
  group_name: <group_name>
  component_name: <component_name>
  buttons: [[button_name1,function_name1,parameter_name2,
    [button_name2,function_name2,parameter_name2]]
```



A sample of this file is: 

```
group1:
  group_name: base
  component_name: base
  buttons: [[stop,trigger,stop],
    [init,trigger,init],
    [recover,trigger,recover]]
group2:
  group_name: torso
  component_name: torso
  buttons: [[stop,trigger,stop],
    [init,trigger,init],
    [recover,trigger,recover],
    [enable joystick,mode,velocity],
    [home,move,home],
    [front,move,front],
    [back,move,back],
    [left,move,left],
    [right,move,right],
    [shake,move,shake],
    [nod,move,nod]]
group3:
  group_name: tray
  component_name: tray
  buttons: [[stop,trigger,stop],
    [init,trigger,init],
    [recover,trigger,recover],
    [enable joystick,mode,velocity],
    [up,move,up],
    [down,move,down]]
```
