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
