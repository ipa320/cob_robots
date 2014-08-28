^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2014-08-28)
------------------
* unique identifier
* fixed yaml
* Merge branch 'hydro_dev' into indigo_dev
* setup cob3-8 simulation
* consequently remove lookat and hybrid stuff from cob3-X robots
* solve non-unique node names
* solve non-unique node names
* adapted gripper controller
* merge with hydro_dev
* Last update cob3-8
* use same PIDs as in ros-industrial repo
* no chance for tuning PID for follow_joint_trajectory controller for lwa4p -> currently do not use arms in urdf
* tune PID values for follow_joint_trajectory controller for torso
* remove obsolete i_clamp_min and i_clamp_max from yaml
* beautify
* increase spawner timeout for slow computers/complex models
* tuning controller parameters for new torso inertias
* adapted gazebo controllers
* Merge branch 'hydro_dev' of github.com:ipa320/cob_robots into indigo_dev
* test_publisher for controller tuning
* added test publisher
* fixes for raw3-3 simulation according torso-head-renaming
* moved lookat_controller yaml and launch files
* merged hydro upstream with simulation adaptions
* fix dependencies
* cleaning up debs
* cob3-8 has pg70 as gripper
* added timestamp to diagnostics msg
* cob3-8 with new structure
* moved base_controller to controllers folder
* call driver before controller
* Fixed reestructuration errors
* Added cob3-8
* fix dependencies
* cleaning up debs
* added missing launch file argument for image_flip
* Added cob_image_flip driver
* remove parameter for gazebo_adapter from cob_hardware_config
* add cob4-2
* Merge pull request `#178 <https://github.com/ipa320/cob_robots/issues/178>`_ from ipa-nhg/hydro_dev
  Inverted scanners position
* tweak ur_controller parameter
* merge with vel_control
* merge with hydro_control for new file structure
* defined ns for tray sensors (simulation)
* test and tweak head and lookat control for raw3-3
* merge with ipa320
* merge with prace updates
* Merge branch 'prace_dev' of github.com:ipa-fxm/cob_robots into prace_changes
* add gazebo_services for lookat for cob4-1
* lookat component for cob4-1
* optimize frida controller parameter
* loading controllers within adapter, no need for launch argument anymore
* changes due to renaming from sdh to gripper and generic gazebo_services
* cob4 fake diagnistics
* use gazebo joint_trajecory controller again for all components
* cleaning up
* vel_control and lookat_control with raw3-3
* changed fridas controller params
* moved file due to new structure
* Merge branch 'hydro_vel_control' into prace_changes
* Merge remote-tracking branch 'origin/groovy_dev' into merge_groovy-dev
  Conflicts:
  CMakeLists.txt
  cob_bringup/robots/cob4-1.xml
  cob_controller_configuration_gazebo/controller/torso_controller_cob4.yaml
  cob_hardware_config/cob4-1/urdf/calibration_default.urdf.xacro
  cob_hardware_config/common/cob4.rviz
  cob_hardware_config/raw3-3/urdf/raw3-3.urdf.xacro
* use hybrid_controller only for torso - all other components need more tuning
* changes on raw3-3 to get the powerball tracking running
* restructuring for hybrid_control
* merged groovy changes into hydro
* twist controller params in yaml + parameter tuning with arms
* back to torso-only
* preliminary vel control for schunk lwa4p
* preliminary velocity_control for head and sensorring
* update velocity control launchfile
* introducing cob_control_topic_mapper
* tune parameter for cob4-1_torso-only vel control
* try vel controller for cob4-1 torso
* use some velocity controller with cob3-3
* generic launch file for starting velocity controller
* new yaml files for velocity controller
* remove velocity controller params
* beautifying
* add dependency to ros_controllers
* add missing dependency
* add roslaunch and urdf tests
* Added sensors to cob4 description
* added gazebo head controller
* added gazebo controller for prace head
* specific rviz configuration pro robot
* define default robot argument
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, ipa-bnm, ipa-cob3-8, ipa-fxm, ipa-fxm-fm, ipa-nhg

0.5.3 (2014-03-28)
------------------

0.5.2 (2014-03-27)
------------------

0.5.1 (2014-03-20)
------------------
* Merge pull request `#155 <https://github.com/ipa320/cob_robots/issues/155>`_ from ipa-nhg/hydro_dev
  install tags
* install tags
* fix for catkin_make_isolated
* merge
* missing dependencies
* merge with groovy_dev
* setup tests
* fix desire dual sdh
* add tray sensors to simulation
* fix simulated cam3d topics
* fix rviz soft links
* fix diagnostics in simulation
* restructuring joint_state_controller and simulated tray_sensors
* Merge pull request `#12 <https://github.com/ipa320/cob_robots/issues/12>`_ from ipa-fxm/groovy_dev
  bring groovy updates to hydro
* added default value for arg robot for ros launch file checks
* fixed typo
* fixes while testing in simulation
* fix inclusion of joint_state_controller
* New structure
* merge with groovy_dev_cob4 + use hydro configurations for controller
* some more fixes due to restructuring
* update CMakeLists
* added fake topics for diagnostics
* removed obsolete file
* create a generic gazebo controller
* Tested on simulation
* New cob_controller_configuration_gazebo structure
* Added arm configuration for cob4
* gazebo controllers for cob4
* New structure cob repositories (cob_controller_configuration_gazebo)
* cob4 integration
* removed a lot of code related to packages not available in hydro anymore
* optimize torso controller
* better values for head_controller
* restructure and optimize gazebo controllers
* updating cob_controller_configuration_gazebo
* cleaning up
* bring in groovy updates
* merge with ipa320-groovy_dev
* changes for simulation
* launch file for lbr_solo
* gazebo controllers for cob3-7
* no more dependency to pr2_controller_manager
* update cob3-7
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots into review320_catkin
* modifications for new controller stucture, this is not working yet
* Installation stuff
* extend tests to cob3-7, raw3-5 and raw3-6
* Merged with now rostest catkin looping, which Florian put upstream
* fix launch tests
* add roslaunch tests
* use default robot arg
* separate sim launch files and enable diagnostics for sim
* Initial catkinization.
* readded prace gripper_controllers to launch file
* fixed faulty launch file argument 'sim'
* added launch for prace gripper controller
* removed gripper controller
* added new robot raw3-6
* Fixed simulation error for raw3-1
* added raw3-5
* added raw3-1 torso_controller configuration and launch files for gazebo simulation
* changed raw3-3 description and configs for abb frida
* Revert "removed old packages"
  This reverts commit 23901cb1317a8ae8d477d22ad80f8efd986d9eae.
* removed old packages
* add scan filter for hokuyo
* merge
* add tests for raw3-3 and raw3-4
* reorderd simulated sdh joints to match order on real robot
* fix gazebo controllers
* fixed trajectory controller for simulated cob3-6
* Urdf and parameter files for tray_powerball
* merge with ipa320
* fix launch arg handling
* substitute env ROBOT with arg robot
* add cob3-5 arm_controller
* add tests for cob3-5
* added pkg_hardware_config, pkg_robot_config and pkg_env_config args to launch files in cob_robots
* introducing raw3-3 with frida_arm
* add light by default
* added simulated tray sensors to simulation
* New sdh contoller parameters for desire gazebo model
* Desire configuration parameters
* move sound and collision observer
* add controllers for cob3-6
* add tests for cob3-6
* changed controller to support new follow joint trajectors action
* add raw3-2 test
* use relayboard_sim from cob_bringup
* cleanup manifest
* modifications for upstream ur5_description
* move launch and config files to cob_robots
* small tuning for gazebo
* modified joint names of controller configuration
* urdf structure change: tray can be calibrated now
* add some configuration for cob3-1
* moved simulated tactile sensors to schunk repository
* renamed icob to raw and merged and cleaned up lots of things
* preserve history for cob_controller_config_gazebot
* Contributors: Alexander Bubeck, Daniel Mäki, Florian Weisshardt, Florian Weißhardt, Jannik Abbenseth, Lucian Cucu, Mathias Lüdtke, abubeck, ipa-bnm, ipa-fmw, ipa-fmw-ms, ipa-fxm, ipa-mig, ipa-nhg
