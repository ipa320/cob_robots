^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
