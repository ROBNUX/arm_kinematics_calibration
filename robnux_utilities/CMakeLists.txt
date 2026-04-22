cmake_minimum_required(VERSION 3.16)
project(robnux_utilities)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(simple_motion_logger REQUIRED)
find_package(robnux_kdl_common REQUIRED)
find_package(robnux_kinematics_map REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
  src/utility.cpp
  src/decent_alg.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  simple_motion_logger
  robnux_kdl_common
  robnux_kinematics_map
)

set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL COMMON_EXPORTS)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_dependencies(
  simple_motion_logger
  robnux_kdl_common
  robnux_kinematics_map
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
