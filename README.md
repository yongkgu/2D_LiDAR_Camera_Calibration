# 2D_LiDAR_Camera_Calibration

Reference
Qilong Zhang, & Pless, R. (n.d.). Extrinsic calibration of a camera and laser range finder (improves camera calibration). 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems


1. Build a big checkerboard and place it in front of the camera-laser range finder
system in the different orientations.

2. For each checkerboard pose, extract the laser points in the laser reading,
and detect the checkerboard grid points in the image. Estimate the camera
orientation Ri and position ti with respect to the checkerboard, and then
compute the calibration plane parameter Ni.

3. Estimate the parameter Φ and ∆ using the closed-form solution.

4. Refine Φ and ∆ using the techniques.

5. If necessary, refine all parameters by minimizing.

![image](https://user-images.githubusercontent.com/95640788/222658526-d683b499-3b44-4057-a829-345528d30450.png)
