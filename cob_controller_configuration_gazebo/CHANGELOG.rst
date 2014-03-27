^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
