#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "VisualOdometry/frame.h"
#include "VisualOdometry/common_include.h"
namespace VisualOdometry
{

class MapPoint
{
public:
  typedef std::shared_ptr<MapPoint> Ptr;
  unsigned long id_;
  // position in world frame.
  Eigen::Vector3d pos_;
  // Normal of viewing direction.
  Eigen::Vector3d norm_;
  // ORB descriptor
  cv::Mat descriptor_;
  // times of being observed by feature matching.
  int observed_;
  // times of being an inlier for pose estimation.
  int correct_;

  static long factory_id_;
  bool good_; // whether it is a good point.

  std::list<Frame*> observed_frames_;
  // keyframes that can observe this mappoint
  int matched_times_; // being one of inliers in PnP
  int visible_times_; // being visible in current frames.

public:
  MapPoint();

  MapPoint(long id, Eigen::Vector3d position, Eigen::Vector3d normal);

  MapPoint(unsigned long id, const Eigen::Vector3d& position,
           const Eigen::Vector3d& norm, Frame* frame = nullptr, 
           const cv::Mat& descriptor = cv::Mat());

  inline cv::Point3f getPositionCV() const {
    return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
  }

  // still not clear of this function, same as frame.h
  static MapPoint::Ptr createMapPoint();
  static MapPoint::Ptr createMapPoint(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& norm,
    const cv::Mat& descriptor, 
    Frame* frame = nullptr);
};

}
#endif