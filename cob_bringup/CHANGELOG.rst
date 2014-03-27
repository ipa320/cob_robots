^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2014-03-27)
------------------
* fix robot_ip address
* add parameter remapping for robot_description
* Contributors: Felix

0.5.1 (2014-03-20)
------------------
* fix for catkin_make_isolated
* some install tag updates
* merge
* merge with groovy_dev
* Fixed small typo
* setup tests
* move rviz config to robot folder
* changed ns
* renamed phidgets.lauch to tray_sensors.launch and added launch and config files for real phidget driver
* seperated gripper launch file
* New structure
* merge with groovy_dev_cob4 + use hydro configurations for controller
* updates for raw3-1
* renamed canopen files
* merge with ipa-nhg
* created driver generic launch files
* created driver generic launch files
* New cob_controller_configuration_gazebo structure
* New structure cob repositories (cob_controller_configuration_gazebo)
* New struture for cob repositories
* tested on robot
* cob4 integration
* added laserscanners to launch file and added frida to raw3-3 urdf
* readded frida urdf
* change install path for hydro
* removing cob3-5b
* Merge pull request `#9 <https://github.com/ipa320/cob_robots/issues/9>`_ from ipa-fxm/groovy_dev
  bring groovy updates to hydro
* Bugfix to pass missing pkg_hardware_config parameter to joy.launch file
* cob3-6 update
* update cob3-6 config
* Fix tray powerball positions
* fix diagnostics and cob3-5b launch
* delete vacuum cleaner
* deactivated wifi diagnosis
* added vacuum cleaner launch files
* setup for lwa4d arm on cob3-5b, correction of calibration entries in cob3-5
* Cepstral mode sound
* added cob3-5b and adjusted default calibration of cob3-5 to good values
* bring in groovy updates
* adjust config for cob3-7
* kinect with registration and z_offset
* merge with ipa320-groovy_dev
* depth offset in parameter -- not used right now
* add arg to ur.launch
* merge
* set localhost in ur_solo
* set localhost in robot.xml
* Renamed ur_connector
* update cob3-7
* ur_connector launch and yaml files
* canopen launch and yaml files for torso and tray
* Update cob3-7
* merge with uncommited local_robot
* Update cob3-7
* canopen launch file
* new torso and tray for cob3-3
* update cob3-7
* Changed package and node for LMS100 laser.
* Corrected launch file.
* start relayboard in simulation
* start relayboard in simulation
* relayboard needs to be started in sim mode
* added right camera and pc aggregators
* removed wifi monitor and mounted ur10 on robot again, not tested in gazebo yet
* changed ip and added tf2
* changed env config to work for hydro
* added remapping to /joint_states
* startup phidget board
* fixed tab and spaces inconsistency
* ur instead of ur10
* replaced ur5 and ur10 with ur
* Rename ur10.launch to ur.launch
* Delete ur5.launch
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots into review320_catkin
* add parameters timeout for undercarriage_ctrl and min_input_rate for cob_base_velocity_smoother
* added prace gripper launch file
* New launch files for PRL+ 80 , torso and tray
* Installation stuff
* extend tests to cob3-7, raw3-5 and raw3-6
* Merged with now rostest catkin looping, which Florian put upstream
* fix launch tests
* add roslaunch tests
* change way the env.sh is resolved for custom env.sh settings
* Initial catkinization.
* update on cob3-5
* update for cob3-4
* Parameters and launch files for cob3-7
* disabled failing tests
* Merge pull request `#91 <https://github.com/ipa320/cob_robots/issues/91>`_ from ipa-cob3-5/groovy_dev
  cob3-5 updates
* fix launch file
* Merge branch 'groovy_dev' of github.com:ipa-cob3-5/cob_robots into groovy_dev
* fix powerball launch file for tray
* add tray sensors to cob3-5 and rename phidgets.yaml to tray_sensors.yaml
* add voltage filter
* adapt sdh config to driver update
* Merge branch 'groovy_dev' of github.com:ipa-cob3-5/cob_robots into groovy_dev
* Added powerball tray
* Merge branch 'groovy_dev' of github.com:ipa-cob3-3/cob_robots into groovy_dev
* fixes for cob3-3
* fix
* correct launch of frida driver
* use full name for voltage filter
* change to festival due to installation problems with cepstral
* add respawn to sdh because it crashed when pressing emergency stop
* specify image and depth mode for kinect
* add voltage filter to each robot
* add cam3d throttle node to cob3-6
* separated sdh and dsa into two launch files
* add new voltage filter to cob3-6
* added launch file for frida
* adjust tray sensors for cob3-6
* Groovy- add rviz configuration
* fixed renaming bug for raw3-6
* cob needs the relayboard in normal mode
* mrege
* filename for uploading navigation goals is now taking into account update default_env_config structure in cob_environments
* fixed filename for uploading navigation_goals
* corrections due to 3 and not 2 pc in raw3-3
* modifications for icra2013
* fix in raw3-6 launch
* added new robot raw3-6
* using args instead of env variables in launch files
* Fixed simulation error for raw3-1
* changes for icra
* fixed cob_base_velocity_smoother params upload and namespace
* start relayboard in sim mode on raw3-5
* fixes for bringup raw3-5
* modified raw3-5 launch file
* added launch file for lms100 laser front
* groovy migration
* startup laserscanners on raw3-5
* added launch files for lms100
* added missing ur10.launch
* added raw3-5
* rename dependency to ur_
* Removing shutdown scripts
* Adjustments to the voltage filter
* ur5_driver -> ur_driver; ur5_description -> ur_description
* switched from ur5 to ur10
* Reverted some changes
* added missing parameter
* Updated .xml files in Groovy
* rename launch file in default_env_confg
* Updated machine tags in .xml files
* Merge pull request `#63 <https://github.com/ipa320/cob_robots/issues/63>`_ from ipa-nhg/groovy_dev
  New branch groovy_dev
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots into groovy_dev
* fixed light_controller bringup
* enable kinect depth registration by default
* separate sdh launch
* Revert "removed old packages"
  This reverts commit 23901cb1317a8ae8d477d22ad80f8efd986d9eae.
* removed old packages
* Groovy migration
* Groovy migration
* add cam3d_throttle to cob3-5
* update deps
* removed image_flip because it is not generic for all robots
* moved to cob_cam3d_throttle package in cob_perception_common
* add arg for nodelet manager
* set default val for data_skip to 2, added image flip
* fix launch syntax
* added data_skip with max value (10)
* adapted for new openni driver
* deavtivate launch tests for cob3-1.xml due to electric incompatible machine tag attribute 'env-loader'
* add launch arg sim to light controller
* add raw3-3 and raw3-4 to brinup tests
* added launch arg sim to relayboad instead of having two launch files
* add monitors and set sound to cepstral
* allow multiple teleop and joy nodes
* Merge branch 'master' of github.com:b-it-bots/cob_robots
* do not launch kinect
* reduce throttle frequency
* fixed remote launch of nodes for ros fuerte
* fixed launch file
* add default values to be able to launch the node in a standalone fashion
* fixes for cob3-1
* add second kinect launch file
* include cob_lbr  and pc monitor for pc2
* update right pike to use new calibration_data repo
* moved launch files of cameras to right folder
* update hardware parameters for cob3-1 and ros fuerte
* added service interface to lbr
* using cepstral by default for cob3-6
* add arguments to cam3d_throttle launch file
* Added kinect
* added additional topics
* added param, fixed syntax error
* Merge branch 'master' of github.com:ipa320/cob_robots
* added cam3d throttle
* hwboard updated
* comment ntp monitor
* add battery and emergency monitor for cob3-6
* Merge branch 'master' of github.com:ipa320/cob_robots
* comment out tray for cob3-6
* disabled wifi-monitor on cob3-6
* changes to include tray_powerball
* add hard disk monitor
* use cepstral by default for cob3-3
* update deps
* separate monitoring
* add monitoring to cob3-3
* pkg_env_config can be set in robot.launch
* hwboard updated
* Changed from reboot to halt
* Idea for the shutting down script
* hwboard added
* updated hwboard
* updated hwboard
* startup cpp light node instead of python node
* added hwboard
* raw3-4 settings
* startup lightnode with cob_bringup
* startup lightnode with cob_bringup
* added relayboard message based on phidget
* added cob_voltage_control to bringup
* added launch files for battery board
* settings for raw3-4
* move relayboard back to pc1
* xml mismatch for doubled laser_top include
* add arg to laser_top
* fixes for raw3-1 config
* remove env config reference
* merged with restructured launch files
* merge with ipa320
* add hokuyo config for scan filter
* support torso names in joystick, add prefix to ur5
* upload default robot config in solo launch files
* new pc names on raw3-1 and working torso config for new urdf
* testing of hardware_test on cob3-3
* missing conf files for raw3-1
* Moved light to pc3
* beautifying
* fix naming of ROBOT to ROBOT_ENV
* warning for no ROBOT or ROBOT_ENV set
* move light to pc1
* fix test definitions
* substitute env ROBOT with arg robot
* substitute env ROBOT with arg robot
* substitute env ROBOT with arg robot
* merged with new fxm version
* merged
* changes from automatica
* removed wifi monitor
* add pc monitor config for all robots
* adjust pc_monitor diagnostics for different cores
* merge
* config update for cob3-6
* Merge branch 'master' of github.com:ipa320/cob_robots
* launch files testing possible again
* move sound and light to pc3
* remove cwd=node
* Added kinect.launch in cob3-2.xml
* Merge branch 'master' of github.com:ipa320/cob_robots
* Merge branch 'master' of github.com:ipa320/cob_robots
* add tests for cob3-5
* added pkg_hardware_config, pkg_robot_config and pkg_env_config args to launch files in cob_robots
* merge with ipa-fxm-lc
* Merge branch 'master' of github.com:ipa-fmw/cob_robots
* added pkg_hardware_config, pkg_robot_config and pkg_env_config args to launch files in bringup
* updates for cob3-2
* add safe base controller to base_solo.launch
* fix paths to point to calibration_data
* move tests to hardware_test package
* Merge branch 'master' of github.com:ipa-fmw-ms/cob_robots into max
* cleanup bringup launch files
* allow individual buttons for command gui
* bringup test for desire
* tray test working on robot
* added simulated tray sensors to simulation
* New configuration parameters and calibration  for cob3-2
* Merge pull request `#22 <https://github.com/ipa320/cob_robots/issues/22>`_ from ipa-nhg/master
  Fixed some parameter mistakes and merge conflict
* Fixed errors in cob3-6.xml, the definition of the machine names were wrong
* Fixed errors in cob3-6.xml, the definition of the machine names were wrong
* Setup xml file for desire
* Update launch file of desire as launch+xml
* merge
* add basic config and tests for cob3-1
* beautify
* testing for ipa-apartment
* Merge branch 'master' of github.com:ipa-fmw/cob_robots
* do not load default configuration  in dashboard launch file but in bringup launch file
* use ROBOT environment variable for wifi monitor
* wifi diagnostics monitor
* Desire configuration parameters
* merge
* merge error
* merge
* cob3-6 bringup files
* cob3-6 calibration parameters
* cob3-6 bringup file update
* add default rviz config
* fix typo in machine tags
* rename safety topic to safe
* add tests for cob3-6
* remove empty line
* merged with 320
* Merge branch 'master' of github.com:ipa320/cob_robots
* Light config
* integration of base_velocity_smoother_param.yaml files and update of base.launch
* update deps
* apply bringup launch changes to all robots
* restructure bringup launch files tested on cob3-3
* restructure bringup launch files to use args --> better testing possible, needs to be tested on hardware
* changes before shipping raw3-1
* Fixed merge conflict
* add collision_observer
* move camera ip adresses to hardware config
* first version of raw3-2 config
* almost final raw3-1 hardware setup
* merge
* defaut arg to localhost
* add cpu diagnostics
* add cpu diagnostics
* add raw3-1 specific collision_velocity_filter_params, footprint_observer_params, local_costmap_params
* add missing dependencies and update stack.xml
* move launch and config files to cob_robots
* commit hardware configuration files for cob3-2
* config files for light in cob_hardware_config
* Included in the bringup light.launch on pc3
* Included in the bringup light.launch on pc3
* added hztest_all.test
* added right_prosilica.test
* added left_prosilica.test
* added laser_top.test
* added laser_rear.test
* added kinect.test
* added hztest_all.test
* added laser front test
* modifications on robot with ur5 arm
* configurations from raw3-1 robot
* add some configuration for cob3-1
* Merge remote branch 'origin-ipa320/master' into automerge
* adapt roslaunch checks
* Merge remote branch 'origin-ipa320/master' into automerge
* fix for raw
* manifest.xml
* filled manifest
* update stack
* teleop with safe base movements
* integrate safe velocity controller by default
* change kinect frame namespace
* fix cob3-2 commit
* fix cob3-2 mergerequest
* cob3-2 updates
* remap for usage of cob_collision_velocity_filter
* fixed laserscanner for icob
* add tests for cob3-4
* move sound to pc3
* fix laser
* fix laser remapping
* include upload_param for env_config (nav goals for base)
* moved camera calibration yaml files from config to calibration folder
* new launch file for rviz and config file
* remove dep to cob_lbr
* setup cob3-4
* cob_scan filter: using multiple scan_ranges given in RAD
* cob_scan_filter
* simplify launch file
* parameters for left and right prosilica camera separeted from intrinsics calibration
* made sdh respawn again
* added diagnostic aggregator for actuator monitoring
* sick_s300 yaml files to be used with new scan-filter
* changed name of cob_dashboard to cob_commmand_gui
* remove diagnostics test
* using hardware_config
* lbr working on robot again
* remove machine files
* update stack
* merge
* fix robot bringup
* lbr config
* added launch tests
* fix test
* Merge remote branch 'origin-ipa-goa/master' into automerge
* changed teleop launch location
* added stereo namespace
* move tools
* modifications for tray and torso config to support new powercube chain structure
* added lbr launch files
* add trajectory controller to torso
* base and teleop running
* added camera config
* add dependency to cob_default_env_config
* update stack
* deactivate robot test due to hostnames which can not be resolved
* added default_env_config
* added teleop and diagnostics
* launch file for cob3-3
* using inifiles from hardware_config
* update stack
* moved cob_config to cob_hardware_config
* added bringup for cob3-3
* bringup started
* moved bringup to robots stack
* Contributors: Alexander Bubeck, Denis Štogl, Florian Weisshardt, Florian Weißhardt, Jannik Abbenseth, Nadia Hammoudeh García, Richard Bormann, Thiago de Freitas, Your full name, abubeck, calibration, cob3-1-pc1, cob3-1-pc2, cob3-2 admin, cob3-5, cpc-pk, fmw-ms, ipa-bnm, ipa-cob3-3, ipa-cob3-4, ipa-cob3-5, ipa-cob3-6, ipa-cob3-7, ipa-fmw, ipa-fmw-ms, ipa-fmw-sh, ipa-frm, ipa-fxm, ipa-goa, ipa-jsf, ipa-mdl, ipa-mig, ipa-nhg, ipa-raw3-3, ipa-tys, ipa-uhr-eh, ipa-uhr-fm, ipa320, ipa320-cob3-6, raw3-1 administrator, robot, unhelkar
