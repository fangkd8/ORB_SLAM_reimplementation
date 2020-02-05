#ifndef MAP_H
#define MAP_H

#include "VisualOdometry/common_include.h"
#include "VisualOdometry/mappoint.h"
#include "VisualOdometry/frame.h"
namespace VisualOdometry
{

class Map
{
public:
  typedef std::shared_ptr<Map> Ptr;
  // landmarks.
  std::unordered_map<unsigned long, MapPoint::Ptr> map_points;
  // keyframes.
  std::unordered_map<unsigned long, Frame::Ptr> keyframes;

public:
  Map() {};

  void insertKeyFrame(Frame::Ptr frame);

  void insertMapPoint(MapPoint::Ptr mpt);
};

}
#endif