#include "VisualOdometry/camera.h"

namespace VisualOdometry
{
Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d p_w, 
                                     sophus::SE3 T_c_w){
  return T_c_w * p_w;
}

Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d p_c, 
                                     sophus::SE3 T_c_w){
  return T_c_w.inverse() * p_c;
}

Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d p_c){
  return 
    Eigen::Vector2d(
      fx_ * p_c(0, 0)/p_c(2, 0) + cx_, 
      fy_ * p_c(1, 0)/p_c(2, 0) + cy_
    );
}

Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d p_p, 
                                     double depth = 1){
  return depth * Eigen::Vector3d(
      (p_p(0, 0) - cx_)/fx_, 
      (p_p(1, 0) - cy_)/fy_,
      1 
    );
}

Eigen::Vector2d Camera::world2pixel (const Eigen::Vector3d p_w, 
                                     sophus::SE3 T_c_w){
  Eigen::Vector3d p_c = world2camera(p_w, T_c_w);
  return camera2pixel(p_c);
}

Eigen::Vector3d Camera::pixel2world (const Eigen::Vector2d p_p, 
                                     sophus::SE3 T_c_w, double depth = 1)
  Eigen::Vector3d p_c = pixel2camera(p_p, depth);
  return camera2world(p_c, T_c_w);
}