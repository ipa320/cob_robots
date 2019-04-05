^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.11 (2019-04-05)
-------------------
* Merge pull request `#775 <https://github.com/ipa320/cob_robots/issues/775>`_ from fmessmer/add_cob4-25
  add cob4-25
* add cob4-25
* Contributors: Felix Messmer, fmessmer

0.6.10 (2019-03-14)
-------------------
* Merge pull request `#769 <https://github.com/ipa320/cob_robots/issues/769>`_ from fmessmer/add_cob4-20
  add cob4-20 ipa 340
* add cob4-20 ipa 340
* Contributors: Florian Weisshardt, fmessmer

0.6.9 (2018-07-21)
------------------
* update maintainer
* Merge pull request `#747 <https://github.com/ipa320/cob_robots/issues/747>`_ from ipa-fxm/add_cob4-13_cardiff
  add cob4-13 cardiff
* add cob4-13 cardiff
* Contributors: Florian Weisshardt, fmessmer, ipa-fxm

0.6.8 (2018-01-07)
------------------
* Merge pull request `#744 <https://github.com/ipa320/cob_robots/issues/744>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#733 <https://github.com/ipa320/cob_robots/issues/733>`_ from ipa-fxm/add_cob4-16_uh
  add cob4-16 uh
* add cob4-16 uh
* Merge pull request `#730 <https://github.com/ipa320/cob_robots/issues/730>`_ from ipa-fxm/moveit_setup_assistant_xacro
  fix setup assistant jade xacro support
* Merge pull request `#729 <https://github.com/ipa320/cob_robots/issues/729>`_ from ipa-fxm/remove_add_robot_helper
  remove add_robot helper
* fix setup assistant jade xacro support
* remove add_robot helper
* Merge pull request `#723 <https://github.com/ipa320/cob_robots/issues/723>`_ from ipa-fxm/move_cob4-2
  move cob4-2 to unity-robotics
* move cob4-2 to unity-robotics
* Merge pull request `#707 <https://github.com/ipa320/cob_robots/issues/707>`_ from ipa-fxm/update_maintainer
  update maintainer
* Merge pull request `#709 <https://github.com/ipa320/cob_robots/issues/709>`_ from ipa-nhg/cob4-10
  Full configuration cob4-10
* harmonize configuration with current status
* Merge github.com:ipa320/cob_robots into indigo_dev
  Conflicts:
  cob_default_robot_config/robots/cob4-8/script_server/command_gui_buttons.yaml
* setup cob4-10
* update maintainer
* Merge pull request `#686 <https://github.com/ipa320/cob_robots/issues/686>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, Florian Weisshardt, Nadia Hammoudeh García, ipa-fxm, ipa-nhg, ipa-uhr-mk

0.6.7 (2017-07-31)
------------------
* switch back to default moveit_config for raw3-1
* Add configs file and update srdf to operate the robot with MoveIt!
* setup cob4-8
* updated the srdf model
* fix CONFIG_TEMPLATE in add_robot
* restructure moveit config
* restructure cob_hardware_config
* Stomp planner (`#631 <https://github.com/ipa320/cob_robots/issues/631>`_)
  * merged stomp configuration with actual indigo_dev
  * controllers for moveit namespace corrected
  * stomp configuration for raw3-1 created and tested
  * few corrections before pull request
  * twist controller config for raw3-1
  * changes from pull request
  * new change from pull request
  * whole-body planning group: robot
  * stomp configuration for robot group
  * pull request changes
  * stomp plannning yaml file correct group names
  * twist controller config file updated to include input limits parameters
  * finalizing PR
* cob4-7 hardware updates
* remove cob4-10
* update collision matrix in moveit_config
* ompl planning file was fixed from last merge process
* update collision matrix in moveit configs
* remove cob4-1
* remove unsupported robots - launch and config
* [WIP] Use grouped low level components for simulation (`#583 <https://github.com/ipa320/cob_robots/issues/583>`_)
  * refactored generic canopen&config into canopen_generic.launch
  * refactored base driver+config into canopen_base.launch
  * added components/cob4_head_camera.launch
  * added components/cam3d_openni2.launch
  * added components/cam3d_r200_rgbd.launch
  * introduce sim arg for components
  * use sim arg in robot.xml
  * remove nodes started within robot.xml from default_controllers_robot.launch
  * introducing legacy components
  * reorganize and sim toggle for more components
  * adjust cob4-1 to latest changes
  * use new structure for cob3-2
  * use new structure for cob3-6
  * use new structure for cob3-9
  * use new structure for cob4-2
  * use new structure for remaining cob4s
  * travis fixes
  * syntax styling
  * use new structure for raws
  * more travis fixes
  * harmonize old vs. new behavior cob4-1
  * guarantee same hw behavior as before
  * add flip argument
* move setup_assistant launch file
* adjust version + add to meta-package
* moved cob_moveit_config
* Contributors: Bruno Brito, Felix Messmer, MattiaRacca, ipa-bfb-sc, ipa-cob4-5, ipa-cob4-8, ipa-fxm, ipa-nhg

0.6.4 (2016-04-01)
------------------
* make 'robot' argument optenv
* add support for fake_execution and sensor input, more consistent with latest moveit_setup_assistant structure
* add octomap updater sensor configuration
* remove schunk arm moveit configs
* update moveit_configs
* explicit name for the collision_monitor plugin
* Contributors: ipa-fxm

0.6.3 (2015-08-31)
------------------

0.6.2 (2015-08-29)
------------------
* migration to package format 2
* remove trailing whitespaces
* sort dependencies
* Contributors: ipa-fxm

0.6.1 (2015-06-17)
------------------
* updates to moveit config for cob4-2
* update joint limits to cope with cob4-2 arms
* fix install tag
* updating joint_limits for cob4-2
* update moveit_config cob4-2
* update moveit_config cob3-6
* Update .setup_assistant
* moveit_config for cob3-9
* set planning_time and planning_attempts for better moveit performance
* update moveit config for cob4-1
* update moveit config for cob4-2
* Merge pull request `#48 <https://github.com/ipa320/cob_manipulation/issues/48>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* updating controller namespaces
* updating self-collision matrix
* rename controller according to new structure
* missing dependency
* adapt controller namespace
* remove support for cob3-7
* remove support for cob3-5
* remove support for cob3-4
* remove support for cob3-2
* remove support for cob3-1
* update moveit_config dependencies
* tune joint_limits
* install tags
* moveit_configs for lwa4d and lwa4p_extended
* Contributors: Florian Weisshardt, ipa-cob4-2, ipa-fxm

0.6.0 (2014-09-18)
------------------
* Merge branch 'hydro_dev' into hydro_release_candidate
* 0.5.1
* add changelogs
* Contributors: Florian Weisshardt, ipa-fxm

0.5.2 (2014-08-28)
------------------
* update cob_moveit_config package for all robots
* cob4-1 moveit config
* changes due to renaming from sdh to gripper
* Contributors: Felix Messmer, ipa-fxm

0.5.1 (2014-03-26)
------------------
* Merge branch 'hydro_dev' into hydro_release_candidate
* update package maintainer
* catkin_lint and install tags
* add changelogs
* fix launch files
* backup from cob3-3
* use sensor info with moveit
* next try
* next try
* fix dependencies
* update package.xml
* catkinize cob_kinematics + use kdl instead of lookat-IK + update moveit_configs
* fix parameter namespace
* started catkinizing
* update moveit_config
* update cob_moveit_configs for all cobs
* lookat_ik_plugin
* updated moveit_config for lookat
* update moveit_config
* back to pick_config
* merge with fmw-ja
* Merge branch 'groovy_dev' of https://github.com/ipa-fmw-ja/cob_manipulation into combine
* started to merge pick-n-place with lookat
* commit before getting nasty
* backup
* merge
* different robot_description for moveit
* fixed namespaces for some parameters
* integration of openrave
* test sensor input for planning_scene
* merge
* introducing cob_moveit_interface, making cob_object_handler obsolete
* new moveit config with base_placement and lookat group
* JSF: Added collision object action to add/remove from remote code
* able to plan for group base again - needs moveit_ros 0.4.4 - still missing controller for execution
* adding launchfile parameter for debugging
* using IKFast plugin - fixing pick() with grasp_list
* added controller for sdh to moveit_config
* updated launch files
* add endeffector for lookat to get interactive marker
* modified moveit_config for cob3-3 to include lookat component
* fixed controller setttings
* loaf rviz config in demo
* moved rviz launch file
* added rviz config
* moveit config for cob3-6 updated
* moveit config for cob3-3 updated
* updated srdf
* updated srdf after upper/lower arm fixup
* updated srdf
* updated SRDF
* switched to IKfast
* rviz demo with debug flag
* updated raw3-1 config
* updated groups
* updated to latest URDF changes
* fixed controller naming
* fixed controller_manager parameters
* added namespace for controller parameters
* new config for raw3-1 using universal_robot ur_description
* added initial version of plan/execute launch file
* updated launch files according to template
* added missing arg
* updated raw3-1 config
* added controller settings
* added first versions of generic launch files
* added projection evaluators
* switched back to kdl solver for raw3-1
* setup assistant launch file
* added cob_moveit_config
* Contributors: Florian Weisshardt, Jan Fischer, Jannik Abbenseth, Mathias Lüdtke, Witalij Siebert, ipa-fxm, rohit chandra
