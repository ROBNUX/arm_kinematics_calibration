# arm_kinematics_calibration
This is a repo. that contains code for robot kinematic model calibration. It
depends on robnux/arm_kinematics_trajectory, Eigen3, pybind11, and ros2. 
It fits into the framework of ros2.

This repo. include 7 robot kinematics models: SCARA, Single-Axis Module, U-Joint, XYZ gantry, XYZ-UR 5-axis robot, 6-axis robot, and general serial arm. There are two types of calibration methods used. (1) Calibration through 
direct 3d-coordinate measurement using e.g. laser tracker (2) Calibration through measuring the distance traveled
using e.g. laser displacement sensors.

This repo. includes 2 types gradient decent algorithm: adam-type and sam-type algorithms. 

