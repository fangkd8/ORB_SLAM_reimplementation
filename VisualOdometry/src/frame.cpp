#include "VisualOdometry/frame.h"
namespace VisualOdometry
{

Frame::Frame(): id_(-1), timestamp_(-1), camera_(nullptr) { };

Frame::Frame(long id, double timestamp, Sophus::SE3 T_c_w, 
             Camera::Ptr cam, cv::Mat color, cv::Mat depth): 
  id_(id), timestamp_(timestamp), T_c_w_(T_c_w), camera_(cam), color_(color), 
  depth_(depth) { };

// Frame::~Frame() {};

// At present, not clear of how this function works,
Frame::Ptr Frame::createFrame(){
  static long factory_id = 0;
  return Frame::Ptr(new Frame(factory_id++));
  // Frame::Ptr framePtr = Frame::Ptr(new(std::nothrow) Frame());
  // if(framePtr != NULL){
  //   framePtr->id_ = factory_id++;
  // }
  // return framePtr;
}

double Frame::findDepth(const cv::KeyPoint& kp){
  int x = round(kp.pt.x);
  int y = round(kp.pt.y);
  ushort d = depth_.ptr<ushort>(y)[x];

  if (d != 0)
    return double(d / camera_->depth_scale_);
  // if d is 0(no depth), return neighbor.
  int dx[4] = {-1,  0, 1, 0};
  int dy[4] = { 0, -1, 0, 1};
  for (int i = 0; i < 4; i++){
    if (depth_.ptr<ushort>(y + dy[i])[x + dx[i]] != 0)
      return double(depth_.ptr<ushort>(y + dy[i])[x + dx[i]]
                     / camera_->depth_scale_);
  }
  // still no value;
  return -1.0;
}

Eigen::Vector3d Frame::getCamCenter() const{
  return T_c_w_.inverse().translation();
}

bool Frame::isInFrame(const Eigen::Vector3d& pt_w){
  Eigen::Vector3d p_c = camera_->world2camera(pt_w, T_c_w_);
  // check if p_c having negative depth, which is on the behind of 
  // image plane.
  if (p_c(2, 0) < 0)
    return false;
  Eigen::Vector2d p_p = camera_->camera2pixel(p_c);
  int x = int(p_p(0, 0));
  int y = int(p_p(1, 0));
  if ((x >= 0) && (y >= 0) && (x < color_.cols) &&(y < color_.rows))
    return true;
  return false;
}

}

