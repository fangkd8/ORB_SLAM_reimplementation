#ifndef CAMERA_H
#define CAMERA_H

#include "VisualOdometry/common_include.h"
#include "VisualOdometry/config.h"

namespace VisualOdometry
{

class Camera
{
public:
  typedef std::shared_ptr<Camera> Ptr;
  float fx_, fy_, cx_, cy_, depth_scale_;

  Camera();
  Camera(float fx, float fy, float cx, float cy, float depth_scale):
         fx_ (fx), fy_ (fy), cx_ (cx), cy_ (cy), depth_scale_(depth_scale) {};
  
  // Coordinates System Transform
  Eigen::Vector3d world2camera(const Eigen::Vector3d p_w, Sophus::SE3 T_c_w);
  Eigen::Vector3d camera2world(const Eigen::Vector3d p_c, Sophus::SE3 T_c_w);
  Eigen::Vector2d camera2pixel(const Eigen::Vector3d p_c);
  Eigen::Vector3d pixel2camera(const Eigen::Vector2d p_p, double depth = 1);
  Eigen::Vector2d world2pixel (const Eigen::Vector3d p_w, Sophus::SE3 T_c_w);
  Eigen::Vector3d pixel2world (const Eigen::Vector2d p_p, Sophus::SE3 T_c_w, double depth = 1);

  // For further use of Intrinsic matrix.
  cv::Mat getIntrinsic();
};

}
#endif