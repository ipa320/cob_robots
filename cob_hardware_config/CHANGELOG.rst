^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_hardware_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2014-03-27)
------------------

0.5.1 (2014-03-20)
------------------
* fix desire dual sdh
* set fixed frame to base_link
* fix rviz soft links
* move rviz config to robot folder
* adjust rviz config
* renamed phidgets.lauch to tray_sensors.launch and added launch and config files for real phidget driver
* base is at pcan0 connected
* fixes while testing in simulation
* update xacro file format
* merge with groovy_dev_cob4 + use hydro configurations for controller
* updates for raw3-1
* addedd missing light parameters
* added missing epsilon parameter
* renamed canopen files
* Tested on simulation
* New cob_controller_configuration_gazebo structure
* Merge pull request `#141 <https://github.com/ipa320/cob_robots/issues/141>`_ from ipa-bnm/fix/raw3-3_bringup
  raw3-3 bringup fixes
* Rename scanners rules
* gazebo controllers for cob4
* New structure cob repositories (cob_controller_configuration_gazebo)
* type error fixed
* New struture for cob repositories
* tested on robot
* cob4 integration
* Merge branch 'groovy_dev' of https://github.com/ipa320/cob_robots into fix/raw3-3_bringup
* removed unused file
* changed encoder counts
* added laserscanners to launch file and added frida to raw3-3 urdf
* added camera holder
* removed a lot of code related to packages not available in hydro anymore
* New cob3-3 calibration
* remove offsets for torso
* removing cob3-5b
* Merge pull request `#9 <https://github.com/ipa320/cob_robots/issues/9>`_ from ipa-fxm/groovy_dev
  bring groovy updates to hydro
* Updated urdf of raw3-1 in cob_hardware_config regarding latest IMU-brick mount on raw3-1
* setup tray configutarion
* Fixed tray powerball
* cob3-6 update
* update cob3-6 config
* adapt calibration
* Fix tray powerball positions
* fix diagnostics and cob3-5b launch
* fixed little number mistake
* added vacuum cleaner launch files
* setup for lwa4d arm on cob3-5b, correction of calibration entries in cob3-5
* copied cob3-5 default config to cob3-5b
* added cob3-5b and adjusted default calibration of cob3-5 to good values
* added teachin handle link
* fix default ref vaues for cob3-5
* update xmlns + beautifying
* bring in groovy updates
* beautifying + slight changes in lookat component
* harmonize with cob structure
* add lookat to all cobs + some fixes in calibration values
* fixing names for cob3-5
* adjust config for cob3-7
* fixed naming error + update structure for all raw's
* 3DOF Tray for cob3-5
* Merge branch 'stable' of github.com:ipa-fmw-ja/cob_robots into lookat
* add lookat component to cob3-3
* cob3-7 new structure with new values
* updated values for cob3-7
* merge with ipa320-groovy_dev
* changes for simulation
* merge 320 with ja
* cam_reference and cam_l differ
* component macro deleted. not supported by xacro
* new better default calibration
* merge
* Renamed ur_connector
* ur_connector launch and yaml files
* canopen launch and yaml files for torso and tray
* Update cob3-7
* merge with uncommited local_robot
* Update cob3-7
* offset of lbr in calibration
* had to flip the laser scans for new udev script
* merge with canopen
* yaml files for canopen components
* merge ipa320/groovy_dev
* Merge branch 'groovy_dev' of https://github.com/ipa-cob3-7/cob_robots into groovy_dev
* Merge branch 'groovy_dev' of https://github.com/ipa-cob3-7/cob_robots into groovy_dev
* update cob3-7
* update cob3-7
* Updated Can configuration for raw3-5.
* Updated lasers configuration for raw3-5.
* move raw calibration
* moved default calibration
* Solved xacro warning in hydro.
* consider left and right arm inside dynamic footprint
* changed homeing switch port for one elmo
* base is connected on pcan0
* attached boxgripper to ee_link
* prosilica config
* added right camera and pc aggregators
* removed wifi monitor and mounted ur10 on robot again, not tested in gazebo yet
* changed prosilica parameters for faster image processing
* Merge branch 'groovy_dev' of github.com:ipa-bnm/cob_robots into groovy_dev
* encoder offsets
* changed homeingdigin port for steer3 because default port on elmo is broken
* fixed yaml file syntax error
* changed urdfs to new base_long and base_short structure, cleaned up all raw's
* change to ur_description
* Merge branch 'review320_catkin' into hydro_dev
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots into review320_catkin
* modifications for new controller stucture, this is not working yet
* add parameters timeout for undercarriage_ctrl and min_input_rate for cob_base_velocity_smoother
* cleanup
* New launch files for PRL+ 80 , torso and tray
* cleaup
* Installation stuff
* extend tests to cob3-7, raw3-5 and raw3-6
* Merged with now rostest catkin looping, which Florian put upstream
* fix launch tests
* add roslaunch tests
* separate sim launch files and enable diagnostics for sim
* remove deprecated relayboard parameters
* Initial catkinization.
* update voltage foilters
* update rviz config
* update on cob3-5
* update for cob3-4
* flipped directories
* temporary fix for calibration_data
* moved default calibration to cob_hardware_config for cob3-3
* deleted files
* Parameters and launch files for cob3-7
* New platform dimensions
* New offsets
* disabled failing tests
* New diagnostics analyzers parameters for desire
* fix cob3-5 urdf for head
* fix powerball launch file for tray
* add tray sensors to cob3-5 and rename phidgets.yaml to tray_sensors.yaml
* remove deprecated rviz config
* fix frame_ids for cameras
* adapt sdh config to driver update
* added canopenmaster config file
* Merge branch 'groovy_dev' of github.com:ipa-cob3-5/cob_robots into groovy_dev
* Added powerball tray
* fixes for cob3-3
* add voltage filter to each robot
* Yaml file for the voltage filter
* merge origin320
* laser configs
* platform ctrl offset
* remove tray and dsa from diagnostics
* adjust tray sensors for cob3-6
* Update rviz config
* Groovy- add rviz configuration
* added adapter plate for frida
* Merge branch 'automerge' into electric_dev
* replace all hardcoded mounting values with respective macros in cob_calibration_data
* replace all hardcoded mounting values with respective macros in cob_calibration_data
* mrege
* new tower description
* new tower description
* some fixes in urdf.xacro for raw3-1
* adapted platform dimensions
* removed gripper
* clean up code
* Merge branch 'groovy_dev' of git://github.com/ipa-raw3-1/cob_robots into groovy_dev
* modifications for icra2013
* encoder offsets for raw3-6
* fixed number of pc cores
* added new robot raw3-6
* added pc_monitor yaml for raw3-5
* Merge pull request `#73 <https://github.com/ipa320/cob_robots/issues/73>`_ from ipa-nhg/groovy_dev
  Added ur10 to raw3-1 urdf model
* changes for icra
* adapted raw3-5s platform ctrl ini
* modified footprint dimensions
* use urdf from short base
* modified footprint observer params for raw3-5
* proper laserscanner configuration for lms100
* adapted diagnostics_analyzers config
* torso mount position can now be parameterized within calibration_data
* added raw3-5
* rename dependency to ur_
* fixed gripper position
* Merge branch 'groovy_dev' of https://github.com/ipa-bnm/cob_robots into groovy_dev
* calibration data for arm mount position
* Adjustments to the voltage filter
* ur5_driver -> ur_driver; ur5_description -> ur_description
* fixed raw3-1s teleop config
* fixed raw3-1s teleop config
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots
* Analyzer mods
* merge
* switched from ur5 to ur10
* Added ur10 from univeral_robot package to raw3-1 description
* add parameter publish_frequency to scanner yaml files; remove swp file
* new parameters for light configuration
* Updated .xml files in Groovy
* Merge pull request `#67 <https://github.com/ipa320/cob_robots/issues/67>`_ from ipa-fmw/master
  add diagnostics to sound and rename launch files
* Merge pull request `#69 <https://github.com/ipa320/cob_robots/issues/69>`_ from ipa-fmw/master
  add diagnostics to sound and rename launch files
* add sound to diagnostics
* no arm_ee_link in frida_description
* Merge branch 'master' into merge
* remove --cov
* Added ur10 to raw3-1 urdf model
* parameter updates for all robots after velocity_smoother-rework
* modified raw3-3s light paramas
* increase circumscribed_threshold for collision velocity filter
* add dsa diagnostics
* separate sdh launch
* changed diagnostic analyzers config, so that diagnostics work together with abb frida on raw3-3
* readded boxgripper on raw3-1 description
* changed raw3-3 description and configs for abb frida
* Revert "removed old packages"
  This reverts commit 23901cb1317a8ae8d477d22ad80f8efd986d9eae.
* removed old packages
* Merge branch 'stable'
* new reference for head due to change in cob_common
* merge
* Included Schunk colors in robot descriptions
* LWA in movevel mode
* head mount calibration
* set horizon of tray back to default
* force velocity mode to have a smooth motion
* change port of led board
* add raw3-3 and raw3-4 to brinup tests
* update cob3-1 urdf
* adapt arm configurations for cob3-5
* fixed order of sdh joint names
* fixed shaky tray movement by reducing the horizon parameter
* changed back previous changes
* adapt head parameters for cob3-1
* Merge remote branch 'origin-ipa320/master' into automerge
* fixed direction of translation for head link. due to last commit
* update horizon parameter of the tray
* using powerball tray for cob3-6
* update hardware parameters for cob3-1 and ros fuerte
* add collision marker and interactive teleop
* using movestep for lwa
* remove swap file
* fix raw urdf
* use ttyTact for cob3-6
* changed reference for "head"
  from "torso_upper_neck_tilt_link"
  to "head_cover_link" for cob3-3 and cob3-6 only
* added inversion flag to raw3-1s light hardware configuration
* Revert "added inversion flag to light hardware configuration"
  This reverts commit f65c326ed3e1bcec9a2f310e0d6bfe6de0ee8fda.
* assigned ttyScanX to scanners
* added raw3-3 to urdf tests
* added inversion flag to light hardware configuration
* Added kinect
* prepared DSA config for cob3-6
* added canopenmaster.yaml
* changes to include tray_powerball
* enable tactile sensors for cob3-3-
* add config for emergency and battery monitor
* remove test file
* separate monitoring
* use move_vel for torso
* comment out wifi monitor
* add monitoring to cob3-3
* hwboard updated
* updated hwboard
* raw3-1 base calibrated
* added hwboard
* raw3-4 settings
* Updated urdf file for cob3-6
* Urdf and parameter files for tray_powerball
* modified/corrected raw3-1 urdf description
* added amadeus box gripper to raw3-1 urdf description
* added cob_voltage_control to bringup
* added launch files for battery board
* settings for raw3-4
* add config for raw3-1 pc monitors
* fixes for raw3-1 config
* changed position of manipulator from back to front
* changed LED device
* changed torso naming to raw
* merge with ipa320
* add hokuyo config for scan filter
* support torso names in joystick, add prefix to ur5
* new pc names on raw3-1 and working torso config for new urdf
* robot specific changes for raw3-1
* config for cob3-1 simulation
* change desire arm_left and arm_right
* Deleted tactile sensor port parameter in the configuration cob3-6
* update to corei7 cob3-3-pc1
* warning for no ROBOT or ROBOT_ENV set
* move light to pc1
* light config for cob3-3
* substitute env ROBOT with arg robot
* harmonize schunk configuration
* New calibration data for torso and tray cob3-4
* adapt laser range
* added torso
* fixed name of xacro macro for raw base
* extend error_range
* removed old arm_ur model
* extend error range
* config for torso and tray on cob3-2
* extend error range for tray
* use movevel for lwa
* force using moveVel
* base calibration for cob3-6
* adapted raw_torso files
* final raw-model V2
* add pc monitor config for all robots
* adjust pc_monitor diagnostics for different cores
* base calibration copied from cob3-5
* config update for cob3-6
* changed can slots on cob3-2
* working parameters for powercube_chain on cob3-5
* added dummy phidgets config
* update config
* config for cob3-5
* Added kinect.launch in cob3-2.xml
* removed wrong launch file
* config for torso, head and lwa
* base calibration
* removed tray, head, sdh config for raw3-1
* removed tray, head, sdh config for raw3-3
* updated base_velocity_smoother_params.yaml files for cob3-1 to cob3-6, desire and raw3-1 and raw3-2
* Merge branch 'review-ipa320'
* updated camera parameter files for cob3-4
* updated camera parameter files for cob3-5
* updated camera parameter files for cob3-2 and cob3-6
* remove calibration files
* camera settings for cob3-2, cob3-4, cob3-5 and cob3-6
* decreased the target frame rate of camera pair to reduce warnings caused by dropped frames
* add tests for cob3-5
* add hardware config for cob3-5
* added pkg_hardware_config, pkg_robot_config and pkg_env_config args to launch files in cob_robots
* added pkg_hardware_config, pkg_robot_config and pkg_env_config args to launch files in bringup
* introducing raw3-3 with frida_arm
* introducing raw3-3 with frida_arm
* clean raw3-1 hardware_config
* final raw-model
* ModuleTypes parameter removed, because not used anymore.
* updates for cob3-2
* adjust tests for cob32
* lights for cob3-6
* adjust diagnostics parameters
* fix desire arm joint names
* add tray links to footprint observer
* remove param farthest_frame from footprint_observer
* add tray links
* Merge branch 'master' of github.com:ipa-fmw/cob_robots
* update manifest
* update stack
* move calibration data to new cob_calibration_data stack
* new torso ref position
* add light by default
* urdf test for desire
* New configuration parameters and calibration  for cob3-2
* new calibration for cob3-3
* Fixed merge conflict
* Setup cob3-6 calibration
* Updated desire config files
* Setup xml file for desire
* Desire config files
* add basic config and tests for cob3-1
* sdh hardware configuration parameters
* lwa configuration parameters for cob3-6
* wifi diagnostics monitor
* Desire configuration parameters
* rename torso joints of raw3-1
* merge
* Merge branch 'master' of github.com:ipa-fmw/cob_robots
* cob3-6 calibration parameters
* cob3-6 bringup file update
* cob3-6 cob_hardware_config update
* add default rviz config
* add controllers for cob3-6
* add config for vel smoother for cob3-6
* add config for vel smoother for cob3-6
* add config for vel smoother for cob3-6
* add tests for cob3-6
* MErge conflict
* Light config
* integration of base_velocity_smoother_param.yaml files and update of base.launch
* Hardware config files for cob3-6
* finished raw3-1 model --- V1
* update deps
* apply bringup launch changes to all robots
* urdf test file for raw3-2
* restructure bringup launch files to use args --> better testing possible, needs to be tested on hardware
* changes before shipping raw3-1
* add ur5_description dep
* move camera ip adresses to hardware config
* merged with ipa320
* first version of raw3-2 config
* calibration by richard
* use old arm model
* Merge branch 'master' of git://github.com/abubeck/cob_robots into review-abubeck
* small modifications for raw
* merge with abubeck
* modifications for raw3-1
* changed for cameras on raw3
* almost final raw3-1 hardware setup
* reduced teleop config
* modifications for new universal robot driver
* add cpu diagnostics
* modifications for upstream ur5_description
* add raw3-1 specific collision_velocity_filter_params, footprint_observer_params, local_costmap_params
* add missing dependencies and update stack.xml
* move launch and config files to cob_robots
* new torso calibration
* commit hardware configuration files for cob3-2
* add empty light.yaml for cbo3-4 to fulffill tests
* fix typo
* fix urdf
* small tuning for gazebo
* urdf structure change: tray can be calibrated now
* config files for light in cob_hardware_config
* changed direction of urdf model to new convention
* Merge branch 'master' of github.com:ipa320/cob_robots
* new torso calibration
* modifications on robot with ur5 arm
* configurations from raw3-1 robot
* add some configuration for cob3-1
* add test for cob3-2
* adapt roslaunch checks
* add calibration for base lasers
* fix for raw
* Merge branch 'master' of github.com:ipa-fmw/cob_robots
* new calibration
* Merge branch 'master' of github.com:ipa-fmw/cob_robots
* using calibration for laser scanners
* new calibration
* renamed icob to raw and merged and cleaned up lots of things
* remove swp file
* again new calibration and moved frequency paramter to controller parameters
* Merge branch 'master' of github.com:ipa320/cob_robots
* chancge speed paraemters
* new calibration for torso
* updated tray config for smoother movements
* new calibration for cameras
* teleop with safe base movements
* load new calibration structure for cob3-4
* cob3-2 with schunk lwa
* cob3-2 update, calibration and urdf file
* cob3-2 updates
* merged with upstream version, deleted a lot of unnecessary stuff
* changed robot/name from cob3-3 to cob3_3 due to cob3_3_arm_navigation requirements
* fixed false macro name
* example config for lwa
* fixed laserscanner for icob
* add calibration files to cob3-4, still uncalibrated
* add laser config for icob
* fix icob urdf
* add tests for cob3-4
* new calibration
* missing files
* restructured icob_description
* icob robot config
* calibrated and verified
* moved camera calibration yaml files from config to calibration folder
* moved sdh up by 1.2cm to correct mount position
* default robot calibration added
* new files for icob for new repository structure
* tosro urdf change: moved head axis up (as in cad)
* torso and arm origins are calibratable in calibration.urdf.xacro
* torso calibrated straight with all zero joint angles
* camera handyed/stereo calibration adjusted to zero offset in head_v3 change
* setup cob3-4
* cob_scan filter: using multiple scan_ranges given in RAD
* cob_scan_filter
* changed default trigger freq for left camera again
* calibrated for experimentation days
* stereo calibration of left and right prosilica
* parameters for left and right prosilica camera separeted from intrinsics calibration
* added lbr stuff to diagnostics
* sick_s300: introduced scan_cycle_time
* changed default trigger freq for left camera, added sensor information to dashboard
* Merge remote branch 'origin/master'
* changed lbr config
* sick_s300: changed laser_frequency to scan_duration
* sick_s300: added laser frequency in yaml
* sick_s300 yaml files to be used with new scan-filter
* changed name of cob_dashboard to cob_commmand_gui
* commented out some not working diagnostics and modified the Actuator analyzers
* change to python test
* lbr working on robot again
* add dep
* added launch tests
* updated calibration
* modifications for tray and torso config to support new powercube chain structure
* added lbr launch files
* base and teleop running
* added camera config
* fix rostest
* added teleop and diagnostics
* launch file for cob3-3
* remove deprecated launch file
* update stack
* moved cob_config to cob_hardware_config
* update hardware config
* Contributors: Alexander Bubeck, Denis Štogl, Felipe Garcia Lopez, Florian Weißhardt, Jannik Abbenseth, Joshua Hampp, Lucian Cucu, Nadia Hammoudeh García, Richard Bormann, SimonEbner, Thiago de Freitas, abubeck, calibration, cob3-1-pc1, cob3-2 admin, cob3-5, cpc-pk, ipa-bnm, ipa-cob3-3, ipa-cob3-4, ipa-cob3-5, ipa-cob3-6, ipa-cob3-7, ipa-fmw, ipa-fmw-ms, ipa-fmw-sh, ipa-frm, ipa-fxm, ipa-goa, ipa-mdl, ipa-mig, ipa-nhg, ipa-raw3-3, ipa-tys, ipa-uhr-eh, ipa-uhr-fm, ipa320, ipa320-cob3-6, mig, nhg-ipa, raw3-1 administrator, robot
