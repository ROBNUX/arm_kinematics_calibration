cmake_minimum_required(VERSION 3.16)
project(arm_calib_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(arm_calib_kinematics REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_utilities REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(${PROJECT_NAME} src/intp/module_calib_command.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  arm_calib_kinematics::arm_calib_kinematics
  robnux_kdl_common::robnux_kdl_common
  Eigen3::Eigen
  ${PYTHON3_LIBRARIES}
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
# Register the PYTHONPATH environment hook (see rob_motion_commands for why
# we call the internal macro directly instead of going through
# ament_python_install_package).
_ament_cmake_python_register_environment_hook()

install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

install(PROGRAMS
  script/01_calib_classes.py
  script/02_direct_measurement_calib.py
  script/03_tcp_and_base_calib.py
  script/04_laser_distance_calib.py
  script/05_pose_path_compensation.py
  script/06_jacobian_picking.py
  script/07_ground_truth_scara_deep_dive.py
  script/test_calib_all_types.py
  script/traj_plot.py
  script/traj_segment_example.py
  script/generate_exception_test_cases.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/calib_registry.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_calib_classes test/test_calib_classes.cpp)
  target_link_libraries(test_calib_classes
    arm_calib_kinematics::arm_calib_kinematics
    robnux_kdl_common::robnux_kdl_common
    Eigen3::Eigen
  )

  ament_add_pytest_test(test_bindings test/test_bindings.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
endif()

ament_package()
