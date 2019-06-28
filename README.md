# Visual_Inertial_Odometry_Simualtion
MATLAB simulation of visual-inertial odometry (VIO) &amp; visual-wheel odometry

This is MATLAB simulation for visual-odometry & visual-inertial odometry & visual-wheel odometry

* You can use this simulation belong your need
* I checked all the demo files whether it can be runned without any amendation, but the MATLAB version can be critical 
* I made this simulations on MATLAB 2019a

## Code Description 
(Each SIMx-files are in a group)
1. **SIM1_w2Dfeat_demo.m** is a simple 2D simulation of camera
2. (*Editing*) **SIM2_w3Dfeat_demo.m** is a 3D simulation of camera with optical flow and feature tracks
3. (*not yet*) **SIM3_IMU_demo.m** is a simulation of inverse INS(path2IMUdata), INS(IMUdata2path) and IMU error model 
4. (*not yet*) **SIM4_WheelOdo_demo.m** is a simulation of inverse WheelOdometry and WheelOdometry with error model
5. (*not yet*) **SIM5_Visual_odometry_demo.m** is a simulation of visual odometry using the camera in SIM2 
6. (*not yet*) **SIM6_Visual_inertial_odometry_demo.m** is a simulation of visual-inertial odometry using the camera in SIM2 and IMU in SIM3
