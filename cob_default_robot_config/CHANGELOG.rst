^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_default_robot_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.8 (2018-01-07)
------------------
* Merge pull request `#744 <https://github.com/ipa320/cob_robots/issues/744>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#733 <https://github.com/ipa320/cob_robots/issues/733>`_ from ipa-fxm/add_cob4-16_uh
  add cob4-16 uh
* add cob4-16 uh
* Merge pull request `#728 <https://github.com/ipa320/cob_robots/issues/728>`_ from ipa-nhg/cob47-setup
  setup cob4-7
* setup cob4-7
* Merge pull request `#723 <https://github.com/ipa320/cob_robots/issues/723>`_ from ipa-fxm/move_cob4-2
  move cob4-2 to unity-robotics
* move cob4-2 to unity-robotics
* Merge pull request `#721 <https://github.com/ipa320/cob_robots/issues/721>`_ from ipa-mjp/correct_torso_param
  Correct torso param
* change torso joint name
* Merge pull request `#707 <https://github.com/ipa320/cob_robots/issues/707>`_ from ipa-fxm/update_maintainer
  update maintainer
* Merge pull request `#709 <https://github.com/ipa320/cob_robots/issues/709>`_ from ipa-nhg/cob4-10
  Full configuration cob4-10
* harmonize configuration with current status
* Merge pull request `#710 <https://github.com/ipa320/cob_robots/issues/710>`_ from ipa-nhg/indigo_dev
  Add head positon buttons for cob4-8 command_gui
* Merge github.com:ipa320/cob_robots into indigo_dev
  Conflicts:
  cob_default_robot_config/robots/cob4-8/script_server/command_gui_buttons.yaml
* add command_gui buttons for head - aalto
* setup cob4-10
* update maintainer
* Merge pull request `#686 <https://github.com/ipa320/cob_robots/issues/686>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* Merge pull request `#701 <https://github.com/ipa320/cob_robots/issues/701>`_ from ipa-fxm/config_cob4-8_aalto
  some fixes cob4-8
* some fixes cob4-8
* restore torso configs
* WIP migration to canopen
* use license apache 2.0
* Contributors: Benjamin Maidel, Felix, Felix Messmer, Florian Weisshardt, Nadia Hammoudeh García, ipa-cob4-8, ipa-fxm, ipa-mjp, ipa-nhg, ipa-uhr-mk

0.6.7 (2017-07-31)
------------------
* add initial config for cob4-10
* add initial cob4-11 serodi config
* cob4-8 setup
* setup cob4-8
* removed unsafed positions
* update cob4-5 setup
* final cleanup
* finalize cob4-9
* Setup cob4-9
* added head for cob4-7
* update cob4-5 configs
* added head for cob4-5
* fxm change requests
* restructure cob_default_robot_config
* cob4-7 hardware updates
* update cob4-paul-stuttgart
* remove cob4-10
* remove sound buttons
* arm speed
* added vacuum gripper
* updated the arm configuration (topics, default vel) in cob_default_hardware_config
* remove cob4-1
* upgrade cob4-2
* remove unsupported robots - launch and config
* harmonize default robot config
* remove home button for head
* remove home button for head
* activate 3dof head
* use test_depends where applicable
* use cob_supported_robots_ROBOTLIST in dependent packages
* manually fix changelog
* setup cob4-10
* cob4-7 setup: final test
* added back_left and back_right
* arm speed
* added vacuum gripper
* updated the arm configuration (topics, default vel) in cob_default_hardware_config
* build torso with arms
* Merge github.com:ipa320/cob_robots into indigo_dev
  Conflicts:
  cob_default_robot_behavior/CMakeLists.txt
* setup cob4-7
* Contributors: Mathias Lüdtke, Richard Bormann, cob4-10, cob4-11, cob4-7, hannes, ipa-cob4-5, ipa-cob4-8, ipa-fxm, ipa-nhg, robot

0.6.6 (2016-10-10)
------------------
* additional param files and modifications for raw3-6 ur10
* remove unsafe ship buttons from command gui
* add shipping pose for arms
* review configuration files
* clean behavior trigger services
* remove torso from cob4-5
* remove head (is static for cob4-5)
* added arms, hands and cameras
* disable head and sensorring for cob4-2
* disable head and sensorring
* unify head and torso poses
* updated urdf model--> addapt the joint configurations
* add 3dof head to cob4-2
* test Head 3dof
* removed unnecessary command gui buttons
* setup cob4-5
* add head to cob4-2
* use symlinks for cob4-1
* improve head and torso joint configurations
* Contributors: Benjamin Maidel, Florian Weisshardt, fmw-hb, ipa-cob4-5, ipa-fmw, ipa-fxm, ipa-nhg, msh

0.6.5 (2016-04-01)
------------------
* proper stop, init and recover via command_gui
* remove pick from command gui
* fix head positions
* add 3dof head for cob4-1 within simulation only
* update cob4-3 according to lastest updates in cob_robots (twist_mux, vel_smoother, laser_topics)
* adapt twist_mux topic names according to https://github.com/ipa320/orga/pull/1#issuecomment-159195427
* changed base_configurations to twist_mux input topic
* remove pick from knoeppkes
* use cob4-1 as cob4-2 without arms - copying configuration files
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into indigo_dev
* remove show gripper
* added cob4-3
* upload correct light params
* modified default colors, more yellow looking color
* fix behaviour
* remove lookat
* fix and consistent services and topics in base config
* arm calibration
* arm calibration and adapted the default positions
* divide pick trigger service
* wave without side start
* added led_off configuration for all robots equiped with lights
* changed base namespace from 'base_controller' to 'base' for cob4 and raw3
* all raws still use old namespace for base
* raws bases still use old namespace '/base_controller' instead of '/base/driver'
* corrected light_configuration.yaml
* added new behavior trigger services
* updated cob_teleop and renamed behaviour package
* more parameter updates for cob4-2
* merge
* robot test
* cob_behaviour
* right arm mount position and removed arm trajectories
* Contributors: Benjamin Maidel, ipa-bnm, ipa-cob3-9, ipa-cob4-2, ipa-fmw, ipa-fxm, ipa-nhg

0.6.4 (2015-08-29)
------------------
* migrate to package format 2
* remove obsolete autogenerated mainpage.dox files
* sort dependencies
* revies dependencies
* merge
* update arm configurations
* unify cob3-X config and launch
* Contributors: ipa-cob4-2, ipa-fxm

0.6.3 (2015-06-17)
------------------
* last update
* install tags and scanners config
* small changes
* setup cob3-2
* update
* added controllers
* added cob3-2
* adapt all light yaml files
* remove mimic yaml file
* use component namespaces for light, mimic and say
* use component namespaces for light, mimic and say
* add sensorring to dashboard and robot.xml
* remove torso and sensorring (untill working properly
* update joint configuration for grippers, add spread pose
* Merge branch 'indigo_dev' of https://github.com/ipa-cob4-2/cob_robots into indigo_dev_cob4-2
* add 2dof torso to cob4-2 including all configuration files
* added cob4-4
* Update upload_param_cob4-2.launch
* robot test
* add missing base_configurations
* add service_ns for light
* addedd missing default parameters and namespaces
* updates from raw3-1 robot user
* beautify CMakeLists
* add stop button for gripper
* add gripper for cob4-1
* added default_vel
* cob4-1 has no grippers
* fix action_name and service_ns
* adapt light settings for all robots
* more namespace adjustments for cob3-6 simulation
* more namespace adjustments for cob3-6 simulation
* more namespace adjustments for cob3-6 simulation
* more namespace adjustments for cob3-6 simulation
* renamed joints
* setup cob4-6
* setup cob46
* update cob3-9
* teached arm position
* setup cob3-9
* setup cob3-9
* setup cob3-9
* Contributors: ipa-cob3-2, ipa-cob3-9, ipa-cob4-2, ipa-cob4-4, ipa-cob4-6, ipa-fmw, ipa-fxm, ipa-nhg

0.6.2 (2015-01-07)
------------------

0.6.1 (2014-12-15)
------------------
* delete cob3-3
* adapt default velocity
* speedup default vel
* cleanup: cob4-1 with torso and head; cob4-2 without torso and head
* deleted sound.yaml
* cob3-9
* setup cob3-9 simulation
* setup cob3-9
* add service_ns for base
* cob3-9
* merge
* add grippers to dashboard
* update cob4-2 config
* updated command_gui buttons
* added accion_name and service_ns parameters
* default config for gripper_left
* default config for gripper_left
* added gripper_right
* config for gripper right
* added accion_name and service_ns parameters
* test raw3-3
* add side configuration and update folded configuration
* switch axis for arm_1 joints
* add parameters for action and service namespace to sss
* updates on cob4-2
* delete desire
* delete cob3-8
* delete cob3-7
* delete cob3-5
* delete cob3-4
* delete cob3-2
* delete cob3-1
* new ros_canopen driver version, adapted bringup configuration
* Contributors: Florian Weisshardt, ipa-cob3-9, ipa-cob4-2, ipa-fmw, ipa-nhg

0.6.0 (2014-09-18)
------------------
* setup cob4-2
* Contributors: ipa-nhg

0.5.4 (2014-08-28)
------------------
* Last update cob3-8
* cob3-8 setup
* setup cob3-8
* fixed dependencies
* cleaning up debs
* cob3-8 has pg70 as gripper
* Added cob3-8
* fixed dependencies
* cleaning up debs
* support for torso configs and init on raw3-3
* merge with ipa-bnm
* added default config to open/close gripper
* changes due to renaming and parameter optimization
* add cob4-2
* use arm_joint_configurations valid for current ur_model
* test and tweak head and lookat control for raw3-3
* merge with ipa320
* Renamed positions
* lookat component for cob4-1
* changes due to renaming from sdh to gripper and generic gazebo_services
* New maintainer
* cob4 fake diagnistics
* update cob4-1 torso and head positions
* Torso working
* support powerball head axis on raw3-3
* merge cob4 (cob_default_robot_config)
* add roslaunch and urdf tests
* fix filename
* Merge branch 'groovy_dev' of github.com:ipa-bnm/cob_robots into groovy_dev
  Conflicts:
  cob_default_robot_config/raw3-1/arm_joint_configurations.yaml
  cob_default_robot_config/raw3-1/command_gui_buttons.yaml
* added command gui button for new default pos
* added new default pos
* default positions for cob4-1
* Contributors: Alexander Bubeck, Florian Weisshardt, cob4-1, ipa-bnm, ipa-cob3-8, ipa-cob4-1, ipa-fmw, ipa-fxm, ipa-nhg, ipa-raw3-3

0.5.3 (2014-03-28)
------------------

0.5.2 (2014-03-27)
------------------

0.5.1 (2014-03-20)
------------------
* fix for catkin_make_isolated
* merge with groovy_dev
* setup tests
* fix desire dual sdh
* fixes while testing in simulation
* updates for raw3-1
* Added arm configuration for cob4
* gazebo controllers for cob4
* New structure cob repositories (cob_controller_configuration_gazebo)
* cob4 integration
* removing cob3-5b
* adapt tray posiitons
* Fixed tray powerball
* Fix tray powerball positions
* added vacuum cleaner launch files
* added some arm and torso positions for cob3-5b, fixed upload script refernce error to cob3-5
* setup for lwa4d arm on cob3-5b, correction of calibration entries in cob3-5
* added cob3-5b and adjusted default calibration of cob3-5 to good values
* adjust config for cob3-7
* merge with ipa320-groovy_dev
* gazebo controllers for cob3-7
* Update cob3-7
* Update cob3-7
* yaml files for canopen components
* update cob3-7
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots into review320_catkin
* Installation stuff
* extend tests to cob3-7, raw3-5 and raw3-6
* Merged with now rostest catkin looping, which Florian put upstream
* update tray positions for cob3-5
* fix launch tests
* add roslaunch tests
* Initial catkinization.
* update on cob3-5
* deleted files
* Parameters and launch files for cob3-7
* Added powerball tray
* add new voltage filter to cob3-6
* new joint configurations for frida
* adapt tray configs to new tray_powerball urdf
* added joint configurations yaml for raw3-3
* added new robot raw3-6
* new arm configs
* added raw3-5
* new default arm configuration for ur10
* fix torso joint names
* Updated .xml files in Groovy
* better default arm joint configuration
* changed raw3-3 description and configs for abb frida
* Revert "removed old packages"
  This reverts commit 23901cb1317a8ae8d477d22ad80f8efd986d9eae.
* removed old packages
* add raw3-3 and raw3-4 to brinup tests
* adapt arm configurations for cob3-5
* changed back previous changes
* adapt head parameters for cob3-1
* opt env for ROBOT
* moved launch files of cameras to right folder
* update hardware parameters for cob3-1 and ros fuerte
* add arm settings to cob3-6
* inserted configuration for blue color
* Added init and recover buttons in command_gui for arm
* beautify
* new joint configs for cob3-6
* remove not readable tray positions
* reduced number of tray joint goals and button for cob3-6
* updated safe arm goal
* adapted tray positions
* changes to include tray_powerball
* removed comamnd gui buttons
* raw3-1 torso calibration
* fixed light configuration
* fixed typo
* settings for raw3-4
* Merge branch 'master' of github.com:ipa320/cob_robots
* rename eyes to head
* add sdhmount position for all lbr robots
* new sdh_mount arm_joint_configuration on dashboard
* some simple arm_joint_configs for testing
* merge with ipa320
* fix tray position for lbr
* missing conf files for raw3-1
* merged on raw3-1
* robot specific changes for raw3-1
* config for cob3-1 simulation
* change desire arm_left and arm_right
* substitute env ROBOT with arg robot
* modified joint_config for overtray so that they hold joint_limits (soft_limit)
* fix syntax
* unify robot configs
* cleanup robot config for cob3-2 and cob3-5
* remove arm settings
* removed wrong configurations
* fixed joint_names for raw3-1
* add tests for cob3-5
* add default robot config for cob3-5
* use robot name directly, not env  ROBOT
* adjust light parameters for all robots
* adjust light parameters
* Updated desire config files
* setup the default robot configuration for desire
* add basic config and tests for cob3-1
* add cob3-1 upload_param.launch
* Desire configuration parameters
* merge with ipa320
* add default_robot_config for cob3-6
* add tests for cob3-6
* add raw3-2 test
* apply bringup launch changes to all robots
* changes before shipping raw3-1
* Merge branch 'master' of https://github.com/abubeck/cob_robots into abubeck
* changed for cameras on raw3
* almost final raw3-1 hardware setup
* reduced teleop config
* fix for init_all and recover_all
* Merge branch 'master' of git://github.com/abubeck/cob_robots into review-aub
* modifications for new universal robot driver
* new nav_positions, new_arm_configurations
* fix base stop
* some new joint_poses for raw_exhibitioin
* corrected raw3-1s arm joint configurations, suffix arm was missing
* add raw3-1 specific collision_velocity_filter_params, footprint_observer_params, local_costmap_params
* stop for base working
* urdf structure change: tray can be calibrated now
* changes to work with raw3
* modifications on robot with ur5 arm
* add some configuration for cob3-1
* adapt roslaunch checks
* renamed icob to raw and merged and cleaned up lots of things
* update stack description
* cob3-2 with schunk lwa
* cob3-2 update, calibration and urdf file
* cob3-2 updates
* fixed laserscanner for icob
* add tests for cob3-4
* setup cob3-4
* changed name of cob_dashboard to cob_commmand_gui
* front_left, front_right, back_left, back_right fixed. right and left were interchanged...
* move default robot config
* Contributors: Alexander Bubeck, Daniel Mäki, Felix Messmer, Florian Weisshardt, Florian Weißhardt, Jannik Abbenseth, Mathias Lüdtke, Richard Bormann, abubeck, cob3-1-pc1, cob3-2 admin, cob3-5, cob_hardware_test, ipa-bnm, ipa-cob3-3, ipa-cob3-5, ipa-cob3-6, ipa-cob3-7, ipa-fmw, ipa-fmw-ms, ipa-fmw-sh, ipa-fxm, ipa-mdl, ipa-nhg, ipa-tys, robot
