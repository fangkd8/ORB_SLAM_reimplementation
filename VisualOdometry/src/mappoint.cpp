#include "VisualOdometry/mappoint.h";

namespace VisualOdometry
{
MapPoint::MapPoint():
  id_(-1), pos(Eigen::Vector3d(0, 0, 0)), norm_(Eigen::Vector3d(0, 0, 0)), 
  observed_(0), correct_(0) {};

MapPoint::MapPoint(long id, Eigen::Vector3d position, Eigen::Vector3d normal):
  id_(id), pos(position), norm_(normal), observed_(0), correct_(0) {};

static MapPoint::Ptr createMapPoint(){
  static long factory_id = 0;
  return MapPoint::Ptr(new MapPoint(factory_id++, 
                           Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector3d(0, 0, 0)));
}

}