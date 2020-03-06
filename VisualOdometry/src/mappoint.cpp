#include "VisualOdometry/mappoint.h"

namespace VisualOdometry
{
MapPoint::MapPoint():
  id_(-1), pos_(Eigen::Vector3d(0, 0, 0)), norm_(Eigen::Vector3d(0, 0, 0)), 
  observed_(0), correct_(0) {};

MapPoint::MapPoint(long id, Eigen::Vector3d position, Eigen::Vector3d normal):
  id_(id), pos_(position), norm_(normal), observed_(0), correct_(0) {};

MapPoint::MapPoint(unsigned long id, const Eigen::Vector3d& position,
                   const Eigen::Vector3d& norm, Frame* frame, 
                   const cv::Mat& descriptor):
  id_(id), pos_ (position), norm_ (norm), good_(true), visible_times_(1), 
  matched_times_(1), descriptor_(descriptor){
    observed_frames_.push_back(frame);
  }

MapPoint::Ptr MapPoint::createMapPoint(){
  static long factory_id = 0;
  return MapPoint::Ptr(new MapPoint(factory_id++, 
                           Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector3d(0, 0, 0)));
}

MapPoint::Ptr MapPoint::createMapPoint(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& norm,
    const cv::Mat& descriptor, 
    Frame* frame){
  static long factory_id = 0;
  return MapPoint::Ptr(new MapPoint(factory_id++, 
                           position, norm, frame, descriptor));
}

}