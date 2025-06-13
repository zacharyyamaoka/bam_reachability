cmake_minimum_required(VERSION 3.8)
project(bam_reachability)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)


install(
  DIRECTORY tests
  DESTINATION share/${PROJECT_NAME}
)


ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pythontest tests/test_mock_kin.py)

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  # add_ros_isolated_launch_test(tests/kinematics/test_six_dof_kinematics.launch.py)
  # add_ros_isolated_launch_test(tests/test_service_api.launch.py)
endif()

ament_package()
