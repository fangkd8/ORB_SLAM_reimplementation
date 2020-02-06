#ifndef G2O_TYPE_H
#define G2O_TYPE_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include "VisualOdometry/camera.h"
#include "VisualOdometry/common_include.h"

namespace VisualOdometry{

// Reprojection Error, only for pose.
class EdgeProjectXYZ2UVPose: 
public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjectXYZ2UVPose(Camera::Ptr cam, cv::Point3f p);

  void computeError();

  virtual void linearizeOplus();

  bool read ( std::istream& in ) {}
  bool write ( std::ostream& out ) const {}

protected:
  Eigen::Vector3d point_;
  Camera::Ptr camera_;

};

}
#endif