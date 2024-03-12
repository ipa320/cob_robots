cmake_minimum_required(VERSION 3.5)
project(cob_hardware_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY robots launch test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ur_urdf_xacro test/test_view_cob_launch.py)
endif()

ament_package()