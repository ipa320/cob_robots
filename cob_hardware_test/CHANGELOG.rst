^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_hardware_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2014-03-20)
------------------
* removed pr2_controller_msgs
* merge with ipa320-groovy_dev
* deactivate tests temporary
* removed xml files from sensor tests. yaml is enough
* added extra teleop_keyboard test
* fixed tab and spaces inconsistency
* removed unnecessary parameter loading
* ur changes applied to related robot tests
* replaced ur5 and ur10 with ur
* simulation now selected by arg sim
* Merge branch 'groovy_dev' of github.com:ipa320/cob_robots into review320_catkin
* reduce auto time to 300.0
* removed calibration test
* Delete cob3-1_manual.test.xml~
  ~
* COMPLETE testorder in launchfile order now
* INCOMPLETE modification for actual robot configuration
* fitted voltage_filter test to yaml structure
* removed params, hardcoded now
* questions hardcoded instead of param. Increased time for user interaction
* added last question to base_test
* dsa test to yaml structure
* moved test for generating diffrent errors to node
* added lwa_sim test
* re-added srv genearation to CMakeLists.txt
* re-added srv genearation to CMakeLists.txt
* modified new tests for new structure
* forgot raw3-6 auto
* merged
* minor changes
* extend tests to cob3-7, raw3-5 and raw3-6
* Merged with now rostest catkin looping, which Florian put upstream
* fix launch tests
* add roslaunch tests
* Initial catkinization.
* update hardware tests for cob3-5
* increased test-time
* remove old comments
* move the dialog_client to src for import
* create rviz config for calibration
* rename tests and cleanup
* rename test-name of hztest
* added audio_common_msgs to manifest.xml
* create branch thread
* create pass test to produce diffent errors
* threaded single test
* suiting/generating multiple tests in vain
* threaded hztests working
* attemp threaded hztest
* attemp threaded hztest
* sound record test
* added new robot raw3-6
* substitute tray_powerball and pike camera in related tests
* Added test for tray_powerball
* Added Pike Camera Test
* renamed launch command from tray_sensor to tray_sensors
* Added ur5 test; re-replaced spaces with tabs
* Added ur5 test; re-replaced spaces with tabs
* solved the four issues
* solved the four issues
* merge
* created teleop_keyboard test, but will not work with current configuration
* copied rospy.wait(.75) from hztest and replaced tabs with spaces in all python files
* Update cob_hardware_test/test/component_test.py
  added from hztest:
  time.sleep(0.75)
* increased time of kinect test
* replaced tab with 4 spaces in components
* replaced tab with 4 spaces in robots
* created robot_auto_sim.test added the time.sleep(0.75) to calibration
* changed machine of rviz to localhost
* fitted calibration test to xml structure
* fitted wifi test to xml structure
* created functional tactile test for all 6 Matrices
* fitted robot tests to new .test.xml structure
* splitted tools to .test and .test.xml
* splitted tests into .test and .test.xml files
* added tactile test
* changed the ($ROBOT)_auto.test.xml to load yaml as test configuration
* changed the ($ROBOT)_auto.test.xml to load yaml as test configuration
* changed the ($ROBOT)_auto.test.xml to load yaml as test configuration
* changed the ($ROBOT)_auto.test.xml to load yaml as test configuration
* replaced wifi and diagnostics xml with yaml
* changed bringup_sim/robot.launch back to bringup/components/componets.launch
* splited yaml to components also for tools
* splited yaml to components
* replaced hztests_list.yaml with cob3-3_hztests.yaml and desire_hztests.yaml
* failure: cannot merge. do not use
* replaced components/hztests_list.yaml with config/ cob3-3_hztests.yaml and desire_hztests.yaml
* Update cob_hardware_test/components/calibration_sim.test
* Update cob_hardware_test/components/calibration.test
* Update cob_hardware_test/components/calibration.test.xml
* Update cob_hardware_test/components/calibration_sim.test
* added yaml configuration of hztest
* added yaml configuration of hztest
* rearranged /robots tests
* removed ~backup again
* removed hztest of relayboard wrench all and changed arm to lwa and lbr
* removed ~backupfiles
* removed ~backupfiles
* removed obsoloete prosilica
* removed obolete all_tests laser and kinect hztest
* added calibration joy teleop and hztests
* fix lbr and lwa test
* rename for lwa
* update manual tests
* separate lbr and lwa test
* update cob3-6 tests
* beautify
* increase test time
* use pregrasp instead of home for hardware_test
* Update cob_hardware_test/test/sound_test.py
* Update cob_hardware_test/test/light_test.py
* Update cob_hardware_test/test/base_test.py
* Update cob_hardware_test/test/base_test.py
* Update cob_hardware_test/test/light_test.py
* Update cob_hardware_test/test/sound_test.py
* Update cob_hardware_test/test/component_test.py
* settings for raw3-4
* merge with ipa320
* testing of hardware_test on cob3-3
* substitute env ROBOT with arg robot
* fix hardware tests
* adapt desire tests
* beautifying
* component test modifications
* modified base test
* evaluate init and recover
* return action state in life test
* enhanced error messages
* life test without base
* add base to life test
* add tests for all robots
* change color for life_test
* working parameters for powercube_chain on cob3-5
* monitor for life test
* update life test
* update life test
* config for torso, head and lwa
* life test
* life test
* remove project files
* updates for cob3-2
* remove unneccesary files
* remove unneccesary files
* adjust tests for cob32
* fix laser test bug
* add tests for cob3-6
* added all_sim.test
* changed files
* added base light and sound tests
* reduzed frequency for wifi_monitor
* add diagnostics_aggregator and wifi_monitor to test
* fix laser and camera tests
* unify laser and prosilica tests
* fix path
* separate tests into components and robots
* renamed tests to cob_hardware_config
* move tests to hardware_test package
* set manual and auto test actively
* manual and autotesting
* test for current ROBOT
* add cob_hardware_test package
* Contributors: Jannik Abbenseth, Maximilian Sieber, abubeck, cob3-5, cob_hardware_test, ipa-bnm, ipa-cob3-5, ipa-fmw, ipa-fmw-ms, ipa-fxm, max, robot
