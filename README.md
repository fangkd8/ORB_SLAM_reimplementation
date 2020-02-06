# SLAM_implementation
This project is aimed for implementing a SLAM, with application of OpenCV functions and g2o optimization.

In `VisualOdometry` directory, there`s a non-structural visual odometry, use OpenCV functions to perform ORB keypoints extraction, descriptor computation and matching, PnP solution with RANSAC. 

## Non-Structural BA
In this update, I only considered BA for 2 frames, pose only optimization. 

And performance improvement is limited.

## TODO: 
* Local Mapping Bundle Adjustment, adding map points optimization.
* Global Mapping BA.


