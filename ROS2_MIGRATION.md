# ROS1 to ROS2 Migration Guide
## arm_kinematics_calibration Repository

### Overview
This document describes the migration of the `arm_kinematics_calibration` repository from ROS1 to ROS2. The migration includes updates to all configuration files and build system files for all three packages.

### Migration Summary

#### Completed Changes

##### 1. Package Configuration (package.xml)
- **Format**: Changed from format 2 (implicit) to **format 3**
- **Build System**: Changed from `catkin` to **`ament_cmake`**
- **Dependency Tags**: Consolidated dependencies using:
  - `build_depend`: Packages needed at build time
  - `exec_depend`: Packages needed at runtime
  - Removed all `roscpp` dependencies (packages have minimal ROS-specific code)
  - Changed `run_depend` to `exec_depend` (ROS2 convention)

**Packages Updated:**
- `rob_commands` (arm_calib_commands)
- `arm_calib_kinematics`
- `robnux_utilities`

**Key Changes:**
- `<package>` → `<package format="3">`
- `<buildtool_depend>catkin</buildtool_depend>` → `<buildtool_depend>ament_cmake</buildtool_depend>`
- Removed all `<run_depend>` tags and replaced with `<exec_depend>`
- Added `<build_type>ament_cmake</build_type>` to export section

##### 2. Build Configuration (CMakeLists.txt)
All CMakeLists.txt files have been updated with the following changes:

**Key Changes:**
- `cmake_minimum_required(VERSION 3.15...3.19)` → `cmake_minimum_required(VERSION 3.16)`
- `add_compile_options(-std=c++11)` → `set(CMAKE_CXX_STANDARD 17)` with proper guard
- `find_package(catkin REQUIRED COMPONENTS ...)` → `find_package(ament_cmake REQUIRED)` with individual package finds
- Removed `${catkin_INCLUDE_DIRS}` from include_directories
- Removed `catkin_package()` and replaced with `ament_package()`
- `TARGET_LINK_LIBRARIES()` → `target_link_libraries()` (lowercase)
- `INSTALL()` → `install()` (lowercase)
- Installation destinations changed:
  - `${CATKIN_PACKAGE_INCLUDE_DESTINATION}` → `include/${PROJECT_NAME}`
  - `${CATKIN_PACKAGE_LIB_DESTINATION}` → `lib`
  - `${CATKIN_PACKAGE_BIN_DESTINATION}` → `bin`
- Removed platform-specific Windows code paths (can be refactored later if needed)
- Removed `add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})`

**Files Updated:**
- `arm_calib_commands/CMakeLists.txt`
  - Cleaned up Python bindings setup
  - Simplified Eigen3 finding
  - Removed platform-specific paths
  - Consolidated target_link_libraries and ament_target_dependencies

- `arm_calib_kinematics/CMakeLists.txt`
  - Simplified package structure
  - Removed ROS 1 build tool macros
  - Updated installation paths
  - Consolidated dependencies

- `robnux_utilities/CMakeLists.txt`
  - Updated to ROS2 format
  - Simplified Eigen3 handling
  - Removed unnecessary paths

#### Source Code - No Changes Required

The codebase has minimal ROS-specific API usage:
- **robnux_utilities**: Utility functions, no ROS dependencies needed
- **arm_calib_kinematics**: Kinematics calibration library, no ROS API calls
- **rob_commands**: Python bindings via pybind11, no ROS API calls

All C++ source files are compatible with ROS2 as-is. The migration only required configuration changes.

### Building and Testing

#### Prerequisites
- ROS2 Humble or later
- Python 3.10+
- cmake >= 3.16
- Eigen3
- pybind11
- All transitive dependencies built and sourced

#### Build Instructions

```bash
# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/robnux

# Source any locally built packages
source install/setup.bash

# Build all packages in arm_kinematics_calibration
colcon build --packages-select arm_calib_commands arm_calib_kinematics robnux_utilities

# Or build with dependencies
colcon build --packages-up-to arm_calib_commands

# Test the build
source install/setup.bash
colcon test --packages-select arm_calib_commands arm_calib_kinematics robnux_utilities
```

#### Dependency Build Order

If building from scratch, build dependencies in this order:
1. Dependencies from `arm_kinematics_trajectory` (dsl_intp, scurve_lib, robnux_kdl_common, robnux_kinematics_map, simple_motion_logger)
2. `robnux_utilities` (depends on #1)
3. `arm_calib_kinematics` (depends on #1 and robnux_utilities)
4. `rob_commands` (arm_calib_commands) (depends on dsl_intp and scurve_lib)

#### Known Issues & Notes

- Platform-specific Windows code has been removed from CMakeLists.txt files. This can be re-added if needed.
- Python installation path changed from `/opt/robnux/lib/python3/site-packages` to the ROS2 standard location determined by `${PYTHON_INSTALL_DIR}/${PROJECT_NAME}`
- The `kinematics_plugin.xml` file is present but installation is currently commented out in the CMakeLists.txt

### Comparison: ROS1 vs ROS2 Package Structure

| Aspect | ROS1 (catkin) | ROS2 (ament_cmake) |
|--------|---------------|-------------------|
| package.xml format | format 2 (implicit) | format 3 (explicit) |
| buildtool | `<buildtool_depend>catkin</buildtool_depend>` | `<buildtool_depend>ament_cmake</buildtool_depend>` |
| CMake minimum version | 3.15...3.19 | 3.16 |
| Package macro | `catkin_package()` | `ament_package()` |
| Find packages | `find_package(catkin REQUIRED COMPONENTS ...)` | `find_package(ament_cmake REQUIRED)` then individual finds |
| Target functions | `TARGET_LINK_LIBRARIES()` | `target_link_libraries()` |
| Installation macro | `INSTALL()` | `install()` |
| Standard C++ | `add_compile_options(-std=c++XX)` | `set(CMAKE_CXX_STANDARD XX)` |

### Next Steps

1. **Testing**: Run `colcon build` to verify all packages compile without errors
2. **Integration**: Ensure the migrated packages work with the rest of the ROS2 workspace
3. **CI/CD**: Update any build scripts or CI/CD pipelines to use ROS2 commands
4. **Documentation**: Update project documentation to reflect ROS2 usage
5. **Python Scripts**: Review any Python scripts in the `script/` directory for ROS1-specific imports and update as needed

### Migration Checklist

- [x] Update all package.xml files to format 3
- [x] Update all CMakeLists.txt files to use ament_cmake
- [x] Remove roscpp dependencies from packages without ROS code
- [x] Verify installation paths are ROS2 compliant
- [x] Create migration documentation
- [ ] Build and test all packages
- [ ] Update any Python scripts if needed
- [ ] Update project README with ROS2 instructions

### References

- ROS2 Build System Migration Guide: https://docs.ros.org/en/humble/
- ament_cmake Documentation: https://github.com/ament/ament_cmake
- CMake 3.16 Documentation: https://cmake.org/cmake/help/v3.16/
