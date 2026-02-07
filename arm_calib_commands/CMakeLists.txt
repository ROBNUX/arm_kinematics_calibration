cmake_minimum_required(VERSION 3.16)
project(rob_commands)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsl_intp REQUIRED)
find_package(scurve_lib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON3_INCLUDE_DIRS}
)


pybind11_add_module(${PROJECT_NAME} src/intp/module_motion_command.cpp)

target_link_libraries(${PROJECT_NAME}
  scurve_lib
  dsl_intp
  Eigen3::Eigen
  ${PYTHON3_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_package()