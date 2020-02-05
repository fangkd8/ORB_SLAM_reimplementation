#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "VisualOdometry/common_include.h";
#include "VisualOdometry/camera.h";
#include "VisualOdometry/config.h";
#include "VisualOdometry/frame.h";
#include "VisualOdometry/map.h";
#include "VisualOdometry/mappoint.h";

namespace VisualOdometry
{

class VisualOdometry
{
public:
  std::shared_ptr<VisualOdometry> Ptr;
  enum State{
    INITIALIZING = -1;
    OK = 0;
    LOST;
  };

  // current VO state.
  State state_;
  // Map
  Map::Ptr map_;
  // current frame and reference frame.
  Frame::Ptr cur_frame_;
  Frame::Ptr ref_frame_;

  // ORB detector, 3d points in ref, keypoints.
  cv::Ptr<cv::FeatureDetector> ORB_detector;
  std::vector<cv::Point3f> ref_points_3d_;
  std::vector<cv::KeyPoint> kpts_;
  // descriptors in both frames;
  cv::Mat des_ref_;
  cv::Mat des_cur_;
  // matches in both frames.
  std::vector<cv::DMatch> matches;

  // pose of current frame;
  Sophus::SE3 T_c_r_estimated;
  int num_inliers_;
  int num_lost_;

  // parameters of ORB extractor.
  int num_of_features_; // number of features
  double scale_factor_; // scale in image pyramid
  int level_pyramid_; // number of pyramid levels
  float match_ratio_; // ratio for selecting good matches
  
  int max_num_lost_; // max number of continuous lost times
  int min_inliers_; // minimum inliers

  double key_frame_min_rot; // minimal rotation of two key-frames
  double key_frame_min_trans; // minimal translation of two key-frames

public:
  VisualOdometry();

  ~VisualOdometry();

  bool addFrame(Frame::Ptr new_frame);

private:
  void extractKeypoints();
  
  void computeDescriptor();
  
  void findFeatureMatches();
  
  void poseEstimationPnP();
  
  void setPointCloud();

  void addKeyFrame();

  bool checkEstimatedPose();

  bool checkKeyFrame();
};

}
#endif