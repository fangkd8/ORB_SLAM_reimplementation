#include "VisualOdometry/g2o_type.h"

namespace VisualOdometry{

EdgeProjectXYZ2UVPose::EdgeProjectXYZ2UVPose(Camera::Ptr cam, 
  cv::Point3f p): camera_(cam){
  point_ = Eigen::Vector3d(0, 0, 0);
  point_(0, 0) = p.x;
  point_(1, 0) = p.y;
  point_(2, 0) = p.z;
}

void EdgeProjectXYZ2UVPose::computeError(){
  const g2o::VertexSE3Expmap* pose = 
  static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);

  // Reprojection error, sum||u - pixel(exp(eps^) * point_)||.
  _error = _measurement - camera_->camera2pixel(pose->estimate().map(point_));
}

void EdgeProjectXYZ2UVPose::linearizeOplus(){
  const g2o::VertexSE3Expmap* pose = 
  static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);

  // get XYZ in camera coordinate system.
  Eigen::Vector3d p = pose->estimate().map(point_);
  double x = p(0, 0);
  double y = p(1, 0);
  double z = p(2, 0);
  double fx = camera_->fx_;
  double fy = camera_->fy_;

  // Jacobian. [de / deps]
  _jacobianOplusXi(0, 0) = fx * x * y / (z*z);
  _jacobianOplusXi(0, 1) = -fx - fx * (x*x) / (z*z);
  _jacobianOplusXi(0, 2) = fx * y / z;
  _jacobianOplusXi(0, 3) = -fx / z;
  _jacobianOplusXi(0, 4) = 0;
  _jacobianOplusXi(0, 5) = fx * x / (z*z);

  _jacobianOplusXi(1, 0) = fy + fy * (y*y) / (z*z);
  _jacobianOplusXi(1, 1) = -fy * x * y / (z*z);
  _jacobianOplusXi(1, 2) = fy * x / z;
  _jacobianOplusXi(1, 3) = 0; 
  _jacobianOplusXi(1, 4) = -fy / z;
  _jacobianOplusXi(1, 5) = fy*y/(z*z);
}

}