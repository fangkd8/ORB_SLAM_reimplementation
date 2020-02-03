#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "VisualOdometry/common_include.h";
namespace VisualOdometry
{

class MapPoint
{
public:
  typedef std::shared_ptr<MapPoint> Ptr;
  unsigned long id_;
  // position in world frame.
  Eigen::Vector3d pos;
  // Normal of viewing direction.
  Eigen::Vector3d norm_;
  // ORB descriptor
  cv::Mat descriptor;
  // times of being observed by feature matching.
  int observed_;
  // times of being an inlier for pose estimation.
  int correct_;

public:
  MapPoint();

  MapPoint(long id, Eigen::Vector3d position, Eigen::Vector3d normal);

  // still not clear of this function, same as frame.h
  static MapPoint::Ptr createMapPoint();
}

}
#endif