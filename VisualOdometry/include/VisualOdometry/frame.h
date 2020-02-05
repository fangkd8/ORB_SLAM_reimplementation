#ifndef FRAME_H
#define FRAME_H

#include "VisualOdometry/common_include.h"
#include "VisualOdometry/camera.h"
namespace VisualOdometry
{

class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<Frame> Ptr;
  unsigned long id_; // Frame id.
  double timestamp_; // timestamp
  Sophus::SE3 T_c_w_; // Transformation from world to camera.
  Camera::Ptr camera_;
  cv::Mat color_, depth_; // RGBD data.

public:
  Frame();
  
  Frame(long id, double timestamp=0, Sophus::SE3 T_c_w=Sophus::SE3(), 
        Camera::Ptr cam=nullptr, cv::Mat color = cv::Mat(), cv::Mat depth = cv::Mat());

  ~Frame() = default;

  static Frame::Ptr createFrame();

  double findDepth(const cv::KeyPoint& kp);

  Eigen::Vector3d getCamCenter() const;

  bool isInFrame(const Eigen::Vector3d& pt_w);
};

}
#endif