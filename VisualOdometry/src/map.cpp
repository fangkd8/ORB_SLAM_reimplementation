#include "VisualOdometry/map.h"

namespace VisualOdometry
{

void Map::insertKeyFrame(Frame::Ptr frame){
  std::cout << "Key Frame numbers: " << keyframes.size() << std::endl;
  if (keyframes.find(frame->id_) == keyframes.end())
    keyframes.insert({frame->id_, frame});
  else
    keyframes[frame->id_] = frame;
}

void insertMapPoint(MapPoint::Ptr mpt){
  if (map_points.find(mpt->id_) == map_points.end())
    map_points.insert({mpt->id_, mpt});
  else
    map_points[mpt->id_] = mpt;
}


}