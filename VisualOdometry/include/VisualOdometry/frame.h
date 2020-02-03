#ifndef FRAME_H
#define FRAME_H

#include "VisualOdometry/common_include.h";
namespace VisualOdometry
{

class Frame
{
public:
  typedef std::shared_ptr<Frame> Ptr;
  unsigned long id_; // Frame id.
  double timestamp_; // timestamp
  Sophus::SE3 T_c_w_; // Transformation from world to camera.
  camera::Ptr camera_;
  cv::Mat color_, depth_; // RGBD data.

public:
  Frame();
  
  Frame(long id, double timestamp=0, Sophus::SE3 T_c_w=Sophus::SE3(), Camera::Ptr cam=nullptr, 
        cv::Mat color, cv::Mat depth);

  ~Frame();

  static Frame::Ptr createFrame();

  double findDepth(const cv::KeyPoint& kp);

  Eigen::Vector3d getCamCenter() const;

  bool isInFrame(const Eigen::Vector3d& pt_w);
};

}
#endif