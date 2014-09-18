^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_default_robot_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
