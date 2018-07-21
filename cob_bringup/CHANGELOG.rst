^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.9 (2018-07-21)
------------------
* update maintainer
* Merge pull request `#764 <https://github.com/ipa320/cob_robots/issues/764>`_ from fmessmer/dualdistro_compatible_env_sh
  select rosdistro in env.sh
* select rosdistro in env.sh
* Merge pull request `#760 <https://github.com/ipa320/cob_robots/issues/760>`_ from ipa-fxm/cob4-10_hw_upgrade
  cob4-10 hw upgrade
* cob4-10 hw upgrade
* Merge pull request `#759 <https://github.com/ipa320/cob_robots/issues/759>`_ from ipa-fxm/fix_grippers_onboard_driver
  re-add joint_names param + consistent launch structure for sdhx with remote driver
* add fake_diagnostics for grippers in sim
* prepare launch file for sdhx with remote driver
* fix joint_names params for robots with onboard gripper driver
* Merge pull request `#713 <https://github.com/ipa320/cob_robots/issues/713>`_ from bbrito/ur_launch_pkg_config
  adding pkg_hardware_config arg
* Merge pull request `#757 <https://github.com/ipa320/cob_robots/issues/757>`_ from ipa-fxm/migrate_unity_structure
  simplify config structure
* simplify config structure
* Merge pull request `#756 <https://github.com/ipa320/cob_robots/issues/756>`_ from HannesBachter/add_cob4-13_cardiff
  changes for cob4-13
* enable grippers in simulation
* undo post shipping changes
* add cob4-cardiff
* Merge pull request `#747 <https://github.com/ipa320/cob_robots/issues/747>`_ from ipa-fxm/add_cob4-13_cardiff
  add cob4-13 cardiff
* Merge pull request `#753 <https://github.com/ipa320/cob_robots/issues/753>`_ from ipa-bnm/feature/sdhx_local
  launch local sdhx driver on cob4-16 gripper computer
* added launch to start sdhx localy on raspbarry and removed launch from bringup
  fixed typo
* Merge pull request `#741 <https://github.com/ipa320/cob_robots/issues/741>`_ from ipa-fxm/cob-uh_final
  [WIP] cob-uh final
* Merge pull request `#750 <https://github.com/ipa320/cob_robots/issues/750>`_ from ipa-fxm/add_missing_components_cob4-18
  add light and em monitor
* add light and em monitor
* cob4-13 config fixes
* Merge pull request `#746 <https://github.com/ipa320/cob_robots/issues/746>`_ from ipa-fxm/add_cob4-18_323
  add cob4-18 323
* add grippers cob-uh
* add arms cob-uh
* add cob4-18 323
* add cob4-13 cardiff
* adding pkg_hardware_config arg
* Contributors: Benjamin Maidel, Bruno Brito, Felix Messmer, Florian Weisshardt, Richard Bormann, cob4-13, fmessmer, ipa-fmw, ipa-fxm

0.6.8 (2018-01-07)
------------------
* Merge pull request `#744 <https://github.com/ipa320/cob_robots/issues/744>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#743 <https://github.com/ipa320/cob_robots/issues/743>`_ from ipa-fxm/laser_range_filter
  introduce laser range filter
* introduce laser range filter
* Merge pull request `#740 <https://github.com/ipa320/cob_robots/issues/740>`_ from ipa-fxm/fix_cam3d_nodelet_namespaces
  fix nodelet and topic namespaces
* fix nodelet and topic namespaces
* Merge pull request `#731 <https://github.com/ipa320/cob_robots/issues/731>`_ from ipa-fxm/enhance_auto_recover_logic
  enhance auto_recover logic
* Merge pull request `#733 <https://github.com/ipa320/cob_robots/issues/733>`_ from ipa-fxm/add_cob4-16_uh
  add cob4-16 uh
* add cob4-16 uh
* enhance auto_recover logic
* Merge pull request `#728 <https://github.com/ipa320/cob_robots/issues/728>`_ from ipa-nhg/cob47-setup
  setup cob4-7
* setup cob4-7
* Merge pull request `#725 <https://github.com/ipa320/cob_robots/issues/725>`_ from ipa-fmw/cob4-11_add_light
  add light to cob4-11
* add light to cob4-11
* Merge pull request `#723 <https://github.com/ipa320/cob_robots/issues/723>`_ from ipa-fxm/move_cob4-2
  move cob4-2 to unity-robotics
* Merge pull request `#722 <https://github.com/ipa320/cob_robots/issues/722>`_ from ipa-mjp/uncomment_ur_arm
  uncomment ur arm
* move cob4-2 to unity-robotics
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into correct_torso_param
* uncomment ur_arm
* Merge pull request `#720 <https://github.com/ipa320/cob_robots/issues/720>`_ from ipa-fxm/fix_camera_coord_frames
  fix camera coord frames for all cameras and all robots for hw and sim
* fix frame_id
* fix image flip for 3dcs
* consistency for all robots
* fix frames for usb_camera and sick_3dcs
* add missing frames for asus
* add nodelet manager for simulation
* fix camera coord frames for asus and zr300 on cob4-7
* add static transforms for zr300
* remove serial number (only needed for multi-camera setup)
* fix torso zr300 camera
* add zr300 launch file
* use zr300 for torso_right camera
* Merge pull request `#719 <https://github.com/ipa320/cob_robots/issues/719>`_ from ipa-fxm/anon_machine_tag
  anon machine tags
* Merge pull request `#716 <https://github.com/ipa320/cob_robots/issues/716>`_ from ipa-fxm/spacenav_launch_args
  introduce launch args for parameters
* anon machine tags
* Merge pull request `#717 <https://github.com/ipa320/cob_robots/issues/717>`_ from ipa-fxm/ntp_monitor_toggle
  do not monitor ntp offset for base pcs
* do not monitor ntp offset for base pcs
* introduce launch args for parameters
* Merge pull request `#698 <https://github.com/ipa320/cob_robots/issues/698>`_ from ipa-fxm/add_ntp_monitor
  add ntp monitor
* Merge pull request `#714 <https://github.com/ipa320/cob_robots/issues/714>`_ from ipa-fxm/legacy_cleanup
  remove legacy stuff and cleanup dependencies
* remove legacy stuff and cleanup dependencies
* add ntp_server for additional pcs of cob4-10
* fix indentation
* add ntp monitor
* Merge pull request `#708 <https://github.com/ipa320/cob_robots/issues/708>`_ from ipa-fxm/feature/powerball_raw3-1
  Feature/powerball raw3 1
* Merge pull request `#707 <https://github.com/ipa320/cob_robots/issues/707>`_ from ipa-fxm/update_maintainer
  update maintainer
* Merge pull request `#712 <https://github.com/ipa320/cob_robots/issues/712>`_ from ipa-jba/feature/kinetic_raw
  single computer for raw, fix ports
* autoinit/autorecover launch file
* single computer for raw, fix ports
* Merge pull request `#709 <https://github.com/ipa320/cob_robots/issues/709>`_ from ipa-nhg/cob4-10
  Full configuration cob4-10
* harmonize configuration with current status
* support old mimic node
* Merge github.com:ipa320/cob_robots into indigo_dev
  Conflicts:
  cob_default_robot_config/robots/cob4-8/script_server/command_gui_buttons.yaml
* Configuration for cob4-10
* setup cob4-10
* turn on twist control, corrected axis
* actuate powerball via canopen
* remove unavailable components
* Merge pull request `#702 <https://github.com/ipa320/cob_robots/issues/702>`_ from ipa-fez/feature/raw3-1-canopen
  Migrate raw3-1 base to canopen
* pass loosened stuck_detector parameters for all raws
* setup cob4-10
* Merge pull request `#706 <https://github.com/ipa320/cob_robots/issues/706>`_ from ipa-fmw/feature/docking
  use scan unified and laser filter for docking
* update maintainer
* Merge pull request `#704 <https://github.com/ipa320/cob_robots/issues/704>`_ from ipa-bnm/feature/mimic_sim
  Add sim argument to mimic launch
* use scan unified and laser filter for docking
* add sim argument to mimic launch
* Merge pull request `#705 <https://github.com/ipa320/cob_robots/issues/705>`_ from ipa-fmw/feature/mimic
  fix mimic vs sound issue
* fix typo
* fix mimic for all robots
* adapt mimic changes to all mimic robots
* fix mimic vs sound issue
* Merge pull request `#686 <https://github.com/ipa320/cob_robots/issues/686>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* change disable_stuck_detector to enable_stuck_detector
* add setting to disable stuck detector for raws and disable it for raw3-1
* tabs vs. spaces
* set proper can device for raw3-1 base
* WIP migration to canopen
* use license apache 2.0
* Contributors: Benjamin Maidel, Felix, Felix Messmer, Florian Weisshardt, Nadia Hammoudeh García, Richard Bormann, cob4-11, ipa-fmw, ipa-fxm, ipa-mjp, ipa-nhg, ipa-uhr-mk, raw3-1, rob@work robot, robot

0.6.7 (2017-07-31)
------------------
* add missing bringup launch file for cob4-11
* add missing bringup launch file for cob4-10
* switch to mimic cpp implementation
* export display number to get mimic working
* use scan unified for docking
* renamed sensorring camera
* setup cob4-8
* switch back to python version of hz monitor
* Merge pull request `#667 <https://github.com/ipa320/cob_robots/issues/667>`_ from ipa-bnm/feature/local_changes
  local changes from cob4-7
* use sim arg for bms
* local changes from cob4-7
* space vs tabs
* integrate arg sim
* rename sick visionary launch file
* update cob4-5 setup
* merge
* finalize
* invert right wheels and change ordering of config (needed after retuning and `UM=2`)
* added reset_errors_before_recovery_parameter from ros_canopen
* steer_ctrl param handling
* final cleanup
* canopen config for raw3-3 base
* finalize cob4-9
* Setup cob4-9
* finalizing configs
* added head for cob4-7
* added head for cob4-5
* topic relays for additional sensor topics not available in simulation
* harmonize robots
* use diagnostic_updater base topic_status_monitor, fake simulation
* proper namespace for static_transform_broadcaster
* use mimic in simulation
* cleanup phidget launch
* adjust pc_monitor
* tested the update with the robot - it works
* fxm change requests
* merge with 320 and bugfix for raw3-1
* fix roslaunch_checks
* arg pkg_hardware_config
* refactoring env config
* restructure cob_hardware_config
* restructure cob_default_robot_config
* configuration via yaml file
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
* harmonize cob4-2 and cob4-7
* lower resolution for head camera
* add realsense static frames for simulation
* cob4-7 hardware updates
* unified ros control base driver and controller config
* added stuck_detector node for all cob4 bases
* update cob4-paul-stuttgart
* remove cob4-10
* Revert "added stuck_detector to bringup"
  This reverts commit 8c06a19ff64510837c9f127e3dc2d121c143972e.
* Merge branch 'tmp/disable_head' into indigo_dev
* added dependency to the camera plugins for the compressed topics
* Raw3 5 config for ros_canopen (`#609 <https://github.com/ipa320/cob_robots/issues/609>`_)
  * Updated raw3-5 launch and description
  * changes for test raw3-5
  * config for raw 3-5 with ros_canopen
  * uncommenting code and optimizing neutral positions
  * delete .dae and .urdf for raw3-5
  * Cleanded files
  * changed diagnostics_analyzers to match with cob4 config
* missed ns group
* changes as per review.
  removed the unused docker_control node.
* changes as per review.
  modified to the single line notation for fake_docking node.
* changes for using fake docking and power usage
* comment ur_modern_driver
* fix diagnostics
* payload default vaues added in the ur launch driver file
* fake_bms driver is publishing diagnostics
* harmonize namespaces of fake_bms
* made changes to keep the parameters under the bms namspace for the fake_bms node
* bms parameters is now being used by fake_bms driver for simulation
* incorporated changes to handle fake_bms and simulation
* make simulation work preliminarily
* Ur Modern Driver configuration
* add fake_diagnostics to all robots
* add fake_diagnostics again
* Merge branch 'stuck_detector' into tmp/disable_head
* added stuck_detector to bringup
* beautify naming of pc monitor
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into tmp/disable_head
* disabled head and sensorring
* remove trailing whitespaces
* image_proc for usb_cam in component
* replace fake_driver
* fix indentation
* fix for indentation issues
* fixes as per requested changes
* added fake power state publisher in order to support simulation
* adapt flexisoft sim for all cob4
* use simulated/fake components
* remove cob4-1
* upgrade cob4-2
* remove obsolete components and dependencies
* remove unsupported robots - launch and config
* framerate explanation comment
* do not use joystick in simulation
* head and sensorring on one bus
* use external and shared sync mode on cob4-10
* overwrite sync interval only in external sync mode
* added external sync mode, generate CAN config on-the-fly
* new bms config
* missing install tag
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
* use test_depends where applicable
* use cob_supported_robots_ROBOTLIST in dependent packages
* Merge pull request `#567 <https://github.com/ipa320/cob_robots/issues/567>`_ from ipa-fxm/restructure_moveit_config
  Restructure moveit config
* remove obsolete envlist from tests
* use mimic rotation
* move camera calibration files into sub-folders
* upload semantic description using new moveit_config structure
* manually fix changelog
* tabs vs spaces
* mimic support the rotation of the face
* unify xml robot files
* cleanup
* android required robot name as argument
* android requires the robot argument
* setup cob4-10
* cob4-7 setup: final test
* fake monitoring for simulation to work with msh scenario
* added phidgets
* Ur Modern Driver configuration
* added arm in bringup, corrected torso mounting angle
* switch cameras
* twist controller launch for bringup
* missing payload parameters for the arm controller
* Added controller for gazebo. Arm gripper removed
* realsense as default torso down camera
* build torso with arms
* add heartbeat for android gui
* rename fliped camear topic
* Merge github.com:ipa320/cob_robots into indigo_dev
  Conflicts:
  cob_default_robot_behavior/CMakeLists.txt
* update cob4-2.xml
* setup cob4-7
* update for raw3-1 torso driver configuration
* Contributors: Benjamin Maidel, Bruno Brito, Felix Messmer, Florian Weisshardt, Mathias Lüdtke, Nadia Hammoudeh García, Richard Bormann, andreeatulbure, cob4-7, fmw-ss, hannes, ipa-cob4-5, ipa-cob4-8, ipa-fxm, ipa-nhg, ipa-raw3-3, ipa-rmb, ipa-uhr-mk, msh, robot

0.6.6 (2016-10-10)
------------------
* renamed visionary_t sensor by sick
* Update usb_camera_node.launch
* update cob4-2.xml
* hd monitor active
* worker threads for openni2 and calibration for head cam
* corrected ur ip address
* fixed namespaces
* Fix usb_cam warning: set the pixel format to yuyv
* Merge github.com:ipa320/cob_robots into fix/env-loader-script
  Conflicts:
  cob_bringup/robots/raw3-6.launch
  cob_bringup/robots/raw3-6.xml
* expand env argument to all robots
* fixed raw3-4 ur bringup
* added env.sh plath as argument
* fix argument naming
* adapted ur.launch to actual ur package
* removed multiple robot_state_publishers by using own ur launch
* added ur10, phidgets, battery monitor, em monitor to robot bringup for raw3-6
* added configs for bringup
* reduce number of nodelet worker to not overload cpu
* add data skip launch argument for openni2 to limit CPU load
* add diagnostics hz monitor to cob4-1 and cob4-2 for cameras
* add nodelet version of realsense to bringup
* unify docking configuration, now only one station config file per robot
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into indigo_dev
  Conflicts:
  cob_bringup/robots/cob4-1.xml
* Merge branch 'indigo_dev' of github.com:ipa-fmw/cob_robots into indigo_dev
  Conflicts:
  cob_bringup/robots/cob4-1.xml
* add dependency to cob_phidget_em_state
* Merge branch 'feature/em_state_phidget' of github.com:ipa-bnm/cob_robots into indigo_dev
* Merge branch 'feature/power_state' of github.com:ipa-bnm/cob_robots into feature/power_state
* beautify
* tabs vs spaces
* use imageflip with torso_cam3d_down camera
* use docking on cob4-2
* tabs vs spaces
* Merge branch 'feature/power_state' into feature/em_state_phidget
* tabs vs spaces
* Merge pull request `#469 <https://github.com/ipa320/cob_robots/issues/469>`_ from ipa-cob4-5/indigo_dev
  Setup cob4-5
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into RemoveDistanceMoveit
  Conflicts:
  cob_bringup/package.xml
  cob_bringup/robots/cob4-1.xml
  cob_bringup/robots/cob4-2.xml
* disable roslaunch check for tools
* fix dependencies
* move hand launch file to bringup
* enable roslaunch tests for robot xmls
* Merge branch 'indigo_dev' of https://github.com/ipa-cob4-5/cob_robots into indigo_dev
* proper remapping
* typo
* bringup emstate from phidget node for raw3-1 raw3-3
* use powerstate from phidget node
* move docking config and launch to cob_hardware_config and cob_bringup
* set check to true for rosserial
* explicit dependency on cob_omni_drive_controller
* Setup cob4-5 : final launch file version
* new schunk_sdhx launch file
* Revert "respawn bms driver"
  This reverts commit a067a923f76fde4264dc42da1d1e987636200f58.
* include/configure stuck detector
* Merge branch 'indigo_dev' of github.com:ipa-cob4-5/cob_robots into indigo_dev
* add cob_hand_bridge to bringup dependencies
* Merge branch 'indigo_dev' of https://github.com/ipa-cob4-5/cob_robots into merge-cob4-5
  Conflicts:
  cob_bringup/package.xml
* added arms, hands and cameras
* harmonize cob4-1.xml and cob4-2.xml
* disable head and sensorring
* reduce framerate of usb camera to lower CPU load
* rename launch arguments
* fix remapping
* publish true with fake collission monitor
* fix diagnostics remapping for sound
* Merge branch 'Feature/SoftkineticParams' of github.com:ipa-nhg/cob_robots into feature/softkinetic
  Conflicts:
  cob_bringup/drivers/softkinetic.launch
  cob_bringup/robots/cob4-1.xml
* add missing dep to usb_cam
* tabs vs spaces
* Merge branch 'indigo_dev' into feature/usb_head_cam
* removed pkg_hardware_cfg from cob4-1.xml
* removed unused line
* cleanup
* tabs vs spaces
* typos
* use camera_name argument as frame_id and camera name
* changed default camera_name to usb_cam
* create softlink instead of copy
* added usb head cam launch file and added it to cob4 bringup
* moved power_state phidget driver to extra package
* removed bms launch + added power_state from phidget launch
* respawn bms driver
* cob4-2 imageflip on same nodeletmanager as cam
* removed data_skipping => higher framerate
* start image flip in same nodeletmanager as the cam
* changed softkinetic_params
* include base collision observer
* add dep to rostopic
* fix launch syntax
* use fake collission monitor for cob4-2 too
* use dummy state publisher instead of real collission monitor (not working reliably yet)
* removed unused arguments
* removed unnecesary argument
* remove in xml files the include
* update collision monitor launch file
* remove dependency to cob_obstacle_distance_moveit
* missed dependency
* robot test
* set softkinetic parameters
* Changed namespace of topics
* Renamed incoming command topic to command_in and removed obstacles topic
* test Head 3dof
* Cleaned up base_collision_observer.launch
* setup cob4-5
* Intermediate state
* Adapted base_collision_observer.launch
* add collision_monitor to cob4-1 and cob4-2
* rename launch file
* add obstacle_monitor launch file
* Merge pull request `#456 <https://github.com/ipa320/cob_robots/issues/456>`_ from ipa-fxm/cartesian_controller_updates
  prepare using robots with cartesian controller
* Merge pull request `#460 <https://github.com/ipa320/cob_robots/issues/460>`_ from ipa-fxm/add_obstacle_distance_moveit_monitor
  prepare obstacle_distance_monitor launch file
* move sound into namespace
* load sound parameter from yaml file
* load sound parameter from yaml file
* add dependencies
* prepare obstacle_distance_monitor launch file
* prepare using robots with cartesian controller
* Contributors: Benjamin Maidel, Denis Štogl, Felix Messmer, Florian Mirus, Florian Weisshardt, Marco Bezzon, Mathias Lüdtke, Nadia Hammoudeh García, bnm, fmw-hb, ipa-bnm, ipa-cob4-2, ipa-cob4-4, ipa-cob4-5, ipa-cob4-6, ipa-fmw, ipa-fxm, ipa-fxm-mb, ipa-nhg, msh, raw3-6, teddy

0.6.5 (2016-04-01)
------------------
* adjust launch file to current head-pc setup
* Merge pull request `#448 <https://github.com/ipa320/cob_robots/issues/448>`_ from ipa-nhg/BMSintegration
  added bms driver to bringup
* added bms driver to bringup
* MLR actual version
* Merge branch 'indigo_dev' of github.com:ipa320/cob_robots into feature_canopen_node_name
  Conflicts:
  cob_bringup/drivers/canopen_402.launch
* add missing image_flip nodes to simulation
* adjust launch and yamls
* unify battery_monitor and battery_light_monitor
* rename canopen node and adjust diagnostics
* restructure canopen driver yamls and remove canX yamls
* readded batter_light_monitor to cob4-1 bringup
* Merge branch 'indigo_dev' of github.com:ipa320/cob_robots into feature/battery_light_mode
  Conflicts:
  cob_bringup/robots/cob4-1.xml
  cob_bringup/robots/cob4-2.xml
  cob_bringup/robots/raw3-3.xml
* temporarily do not use head on cob4-2
* temporarily do not use head on cob4-1
* comment overkill
* changed service name remap to component name param
* Merge branch 'indigo_dev' of github.com:ipa-bnm/cob_robots into feature/battery_light_mode
* further tests with torso
* tabs vs spaces
* tabs vs spaces
* use launch arg to switch between old and new base driver
* tabs vs. spaces
* using canopen for base_solo
* update diagnostics analyzer
* add new_base_chain config for cob4-1
* launch ros_canopen for cob4-2 base
* twist_controller base commands cannot go through smoother
* Removed releyboard
* Merge pull request `#397 <https://github.com/ipa320/cob_robots/issues/397>`_ from ipa-nhg/NewTorsoPcs
  [cob4-2] New torso pcs
* remap battery_light_monitor topic and service name
* start battery_light_monitor on raw3-3 bringup
* load battery light config to param server
* Update cob4-1.launch
* added battery_light_monitor launch to cob4-1 bringup
* added battery light monitor to cob4-2s bringup
* Revert namespace of sick LMS1xx nodes
* Further files corrected
* Corrected odometry topic remapping, error done in 8868a5c
* Correct LMS1xx topic remapping
* Revert indentation changes.
* Change namespace of parameters for laser scanner driver to work properly.
* base collision observer setup
* Merge remote-tracking branch 'origin/raw3-5_battery_voltage' into update_raw3-5
* Merge branch 'indigo_dev' of github.com:iirob/cob_robots into indigo_dev
* review image_flip parameters
* updated base solo
* emergency_stop_state has to be a global topic
* emergency_stop_state has to be a global topic
* remove env config in all robot launch files
* parameterizable scaling factor
* provide twist_mux topic for base_active mode of twist_controller
* update cob4-3 according to lastest updates in cob_robots (twist_mux, vel_smoother, laser_topics)
* Merge branch 'indigo_dev' of github.com:ipa320/cob_robots into feature_cob4-1_without_arms
* Merge pull request `#383 <https://github.com/ipa320/cob_robots/issues/383>`_ from ipa-fxm/restructure_laser_topics_unifier
  Restructure laser topics unifier
* Merge pull request `#21 <https://github.com/ipa320/cob_robots/issues/21>`_ from ipa320/indigo_dev
  updates from ipa320
* Merge pull request `#36 <https://github.com/ipa320/cob_robots/issues/36>`_ from ipa320/indigo_dev
  updates from ipa320
* add missing exec_depends
* rename laser scanner topics
* prepare remapping for twist_mux in cartesian controller
* fix identation
* fix identation
* Merge pull request `#371 <https://github.com/ipa320/cob_robots/issues/371>`_ from ipa-bnm/fix/raw3-1_bringup
  fix raw3-1 bringup
* moved collision_velocity_filter to base namespace
* fix typo
* restructure laser topics
* added collision_velocity_filter to twist_mux
* removed yocs_velocity_smoother dependency
* readded group tag
* changed velocity smoother topic name
* added twist_mux and new velocity_smoother to controller launch
* added velocity_smoother launch file and velocity_smoother configs for all robots
* added twist_mux launch file and twist_mux configs for all robots
* Merge branch 'indigo_dev' into feature/twist_mux_vel_smoother
* added twist_mux and vel smoother dependency
* use correct pc names
* fix machine tag
* use cob4-1 as cob4-2 without arms - copying configuration files
* do not stabelize/deadband spacenav twist
* add scan_unifier for cob4-3
* added dependency to cob_scan_unifier
* Merge pull request `#364 <https://github.com/ipa320/cob_robots/issues/364>`_ from ipa-bnm/feature/scan_unifier
  added scan unifier to bringup layer
* added missing exec dependency to cob_default_robot_behaviour
* added cob4-3
* fixed launch tag
* added scan unifier to bringup layer
* changed name relayboard to powerboard
* indentation
* start cob_voltage_monitor instead of simulated relayboard
* remap input topics
* removed prosilica cams from raw3-1 startup
* correct topic remaps
* fix copy-and-paste comment
* remove old teleop leftover
* tabs vs spaces
* remove obsolete argument and remap
* Adapt cob4-6 configuration
* test sensorring cam3d on cob4-2
* removed leading / from tf frame names. They are no longer supported in tf2
* addapt cob4-4 configuration
* use relative namespaces
* added script_server bringup to all robots
* changed base namespace from 'base_controller' to 'base' for cob4 and raw3
* do not respawn phidgets, because if no phidget is connected the driver will restart all the time
* start cob_script_server at bringup because new teleop node needs it
* fix xml format in cartesian_controller.launch
* remove trailing whitespaces
* add nodes for debugging
* added new behavior trigger services
* add launch file for teleop_spacenav
* merge
* use local namespaces
* merge error
* merge error
* updated cob_teleop and renamed behaviour package
* new teleop node
* proper remapping for old_base_driver
* merge
* merge
* fix typo
* new trigger srv and addapted  android.launch file
* fix for int16 overflow in vl mode
* Merge branch 'cob_behaviour' of https://github.com/ipa-cob4-2/cob_robots into indigo_dev
* Adapted launch and params.
* cob_behaviour
* robot test
* added mimic.launch
* cob_behaviour
* last update
* Update raw3-4.xml
* teleop parameters
* defined teleop parameters
* setup cob4-4
* merge
* cob4-4 setup
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into indigo_dev
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_robots into raw3-5_battery_voltage
* Updated data for raw3-5
* Raw3-5 phidgets is read properly, data calcualtion/remapping is corrected.
* Enabled and corrected
* Change file name from laser_lms1xx to sick_lms1xx
* Corrected remapping and cleaned config file.
* laser_rear namespace corrected
* Merge branch 'hydro_dev' into indigo_dev
* Contributors: Benjamin Maidel, Denis Štogl, Felix Messmer, Florian Weisshardt, Marco Bezzon, Nadia Hammoudeh García, bnm, ipa-bnm, ipa-cob4-2, ipa-cob4-4, ipa-fmw, ipa-fxm, ipa-fxm-mb, ipa-nhg

0.6.4 (2015-08-29)
------------------
* renamed parameter
* making 'sim_enabled' a launch argument
* migrate to package format 2
* remove trailing whitespaces
* remove obsolete autogenerated mainpage.dox files
* Torso->can0
* sort dependencies
* revies dependencies
* renamed launch-argument to use_rplidar in raw3-3.xml
* fix indentation in raw3-3.xml
* merge
* include torso in bringup
* Separate launch file for cob_obstacle_distance.
* updates for cartesian_controller yaml
* torso setup
* moved base components of cob3-9 to correct machine tag
* cob_bringup: removed run-dependency of rplidar_ros and trigger start of rplidar-driver via launch-argument as suggested
* unify cob3-X config and launch
* use controller_manager spawn
* cob_bringup: added run_dependency for rplidar_ros
* added rplidar sensor to raw3-3 urdf and bringup
* Contributors: Florian Mirus, ipa-cob4-2, ipa-fxm, ipa-fxm-mb, ipa-nhg

0.6.3 (2015-06-17)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* last update
* install tags and scanners config
* small changes
* setup cob3-2
* fix run dependency
* added controllers
* adapt cob3-2
* added cob3-2
* fix launch xml syntax
* rename can_modul to can_device
* use component namespaces for light, mimic and say
* Merge remote-tracking branch 'origin-320/indigo_dev' into aggregated_robot_state_publisher_for_all_robots
* Merge branch 'indigo_dev' of github.com:ipa320/cob_robots into indigo_dev
* add sensorring to dashboard and robot.xml
* Merge pull request `#5 <https://github.com/ipa320/cob_robots/issues/5>`_ from ipa-fxm/aggregated_robot_state_publisher_for_all_robots
  aggregated robot_state_publisher for all robots, fixed machine tag in la...
* remove torso and sensorring (untill working properly
* aggregated robot_state_publisher for all robots, fixed machine tag in launch files
* adapt flexisoft config for updated driver with diagnostics
* Merge branch 'indigo_dev' of https://github.com/ipa-cob4-2/cob_robots into indigo_dev_cob4-2
* remap diagnostics for cob_head_axis
* add aggregating robot_state_publisher instead of one per component
* move script_server to t1 pc, add machine timeouts
* add 2dof torso to cob4-2 including all configuration files
* merge
* added cob4-4
* robot test
* remove side argument
* no default value in image_flip_nodelet launch file
* robot_state_publisher moved to base_controller launch file
* robot_state_publisher moved to base_controller
* fix namespace
* proper remap for joint_states
* add robot_state_publisher and joint_state relay
* updates from raw3-1 robot user
* some consistency renaming
* harmonize launch files and resolve node name conflicts
* merge conflict after cherry-picking image_flip updates
* rename yaml file
* remove duplicate robot_state_publisher - it is in controller
* remove deprecation warning again so that tests pass
* moved cob sound launch file
* use updated and adjusted driver and controller launch files for all available robots
* adjust to new namespaces
* remove controller aspects from driver launch file
* adjust old driver launch file to namespaces
* adjust cob_trajectory_controller launch file to namespaces
* unify xml order and beautify
* unify xml order and beautify
* beautify
* cleanup and add dependencies from cob_controller_configuration_gazebo
* remove unused files
* restructure robot_state_publisher
* fix syntax error
* tabs vs. spaces and cleanup
* restructure generic controller launch files
* restructure base_controller_plugin launch file
* tabs vs. spaces
* restructure laser_scan_filter
* adjust image_flip launch and config files
* beautify CMakeLists
* fix missing mode adapter
* add end-of-comment
* remove old non-functional launch files
* added deprecation warning for cob_trajectory_controller
* enable sound for cob4-2 and emergency monitor
* make cob3-6 work in indigo simulation using new namespace structure and fjt controllers only
* make cob3-6 work in indigo simulation using new namespace structure and fjt controllers only
* cob4-6 setup
* add dependency to topic_tools
* update cob4-2 config on real robot
* Adds the joint limits for the base
* Introduces the mode_adapter argument to optionally load the cob_mode_adapter
* resolve conflicts
* setup cob4-6
* setup cob46
* use relay instead of remap for joint_states topic
* setup cob3-9
* setup cob3-9
* set ROBOT variable
* addapted diagnostics new ns and create a separated image_flip launch file
* Contributors: Florian Weisshardt, ipa-cob3-2, ipa-cob3-9, ipa-cob4-2, ipa-cob4-4, ipa-cob4-6, ipa-fmw, ipa-fxm, ipa-nhg, thiagodefreitas

0.6.2 (2015-01-07)
------------------

0.6.1 (2014-12-15)
------------------
* merge
* rename canopen launch files and fix roslaunch test errors
* delete cob3-3
* cob3-9
* Update cob3-9.xml
* setup cob3-9
* comment mimic
* cob3-9
* add recover for grippers
* add light and sdhx to cob4-2
* add namespace for light launch file. needed for cob4-2
* default config for gripper_left
* config for gripper right
* add cob4 to tests
* Delete phidgets_monitor.launch
* Update base_solo.launch
* Update base_solo.launch
* Update teleop_v2.xml
* Update teleop_v1.xml
* Merge pull request `#23 <https://github.com/ipa320/cob_robots/issues/23>`_ from ipa-cob4-2/indigo_dev
  actual version cob4-2
* actual version cob4-2
* test raw3-3
* Update env.sh
* merge
* add robot arg to imageflip
* use teleop v1 and add light to bringup
* remove launch prefix
* Merge pull request `#3 <https://github.com/ipa320/cob_robots/issues/3>`_ from ipa-fmw/indigo_new_structure
  Indigo new structure
* update cob4-2 launch file
* updates on cob4-2
* add lookat components to cob4-2
* added temporary topic_relays for base - v1.5
* indigo_new_structure
* indigo_new_structure
* launch and yaml file base according to new structure
* adapt teleop to v2
* delete desire
* delete cob3-8
* delete cob3-7
* delete cob3-5
* delete cob3-4
* delete cob3-2
* delete cob3-1
* switch parameter namespaces due to BRIDE private nodehandle
* new ros_canopen driver version, adapted bringup configuration
* add parameter for max_X_velocity to launch file
* new parameter files
* Merge pull request `#226 <https://github.com/ipa320/cob_robots/issues/226>`_ from ipa-nhg/indigo_test
  bringup tests
* bringup tests
* moved msgs
* set locahost as default parameter
* set locahost as default parameter
* add monitor scripts to replace pr2_computer_monitor
* Contributors: Florian Weisshardt, Nadia Hammoudeh García, ipa-cob3-9, ipa-cob4-2, ipa-fmw, ipa-fxm, ipa-nhg

0.6.0 (2014-09-18)
------------------
* moved frame_tracker to separate package
* moved frame_tracker to separate package
* Contributors: ipa-fxm

0.5.4 (2014-08-28)
------------------
* remove obsolete cob_hwboard
* remove obsolete dependency
* changes due to introduction of cob_msgs
* merge with hydro_dev
* separated ports for tray and torso
* Last update cob3-8
* setup cob3-8
* cob3-8 setup
* do not use twist_controller on real hardware yet
* added cob_image_flip dependency
* renamed pg70
* setup cob3-8
* tabified file
* start lightcontroller on raw3-3 bringup
* use twist controller for cob4-1 torso
* add twist controller launch file
* moved lookat_controller yaml and launch files
* cleaning up debs
* separate controller and driver yaml file
* cob3-8 with new structure
* merge conflict
* update cob4.xml
* moved base_controller to controllers folder
* Merge branch 'hydro_dev' of https://github.com/ipa320/cob_robots into feature/raw3-4-configs
* Added cob3-8
* cleaning up debs
* added missing launch file argument for image_flip
* add lookat launch file
* Merge pull request `#188 <https://github.com/ipa320/cob_robots/issues/188>`_ from ipa-cob4-1/hydro_dev
  Adapt cob_image_flip and new tag for openni2 driver
* another retab
* Retabbing raw3-4.xml
* Retabbing base.launch
* multiple config changes for raw3-4
* adapted image_flip
* adapted image_flip
* needed machine tag for openni2
* component_solo for canopen components
* component_solo for canopen components
* bring latest raw3-3 changes to new structure
* Added cob_image_flip driver
* start grippers in simulation
* Merge branch 'enhancement/separation_driver_control' into merge-aub
* added torso powerball to robot config
* use correct executable
* merge with ipa320
* some renaming as discussed
* separation of driver and controller
* merge with hydro_dev
* add cob4-2
* added voltage ctrl yaml for raw3-3
* beautifying
* added arguments to softkinetic launch file
* remove deprecated launch files in cob_driver and add nodes to cob_robots
* Renamed positions
* changes due to renaming from sdh to gripper and generic gazebo_services
* New maintainer
* added paths to field configs
* tab vs spaces
* tabs vs. spaces
* Merge remote-tracking branch 'origin/groovy_dev' into merge_groovy-dev
  Conflicts:
  CMakeLists.txt
  cob_bringup/robots/cob4-1.xml
  cob_controller_configuration_gazebo/controller/torso_controller_cob4.yaml
  cob_hardware_config/cob4-1/urdf/calibration_default.urdf.xacro
  cob_hardware_config/common/cob4.rviz
  cob_hardware_config/raw3-3/urdf/raw3-3.urdf.xacro
* merged groovy changes into hydro
* Torso  and head working
* Torso working
* integrated advanced led feedback into cob_monitor, old behaviour still working
* remap topic odometry
* flexisofft tested on robot
* Flexisoft launch and config files
* add roslaunch and urdf tests
* merge cob4
* tested on cob3-3
* setup cob4-1 xml
* Defined component_name as generic name (arm)
* merge
* merge
* default positions for cob4-1
* specific rviz configuration pro robot
* Contributors: Alexander Bubeck, Benjamin Maidel, Felix Messmer, Florian Weisshardt, Mathias Lüdtke, Nadia Hammoudeh García, abubeck, cob4-1, ipa-bnm, ipa-cob3-3, ipa-cob3-8, ipa-cob4-1, ipa-fmw, ipa-fxm, ipa-nhg, ipa-raw3-3, raw3-1 administrator

0.5.3 (2014-03-28)
------------------
* add dependency to ipa_canopen_ros
* Contributors: Florian Weisshardt

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
