#include "VisualOdometry/VisualOdometry.h"

namespace VisualOdometry
{

VisualOdometry::VisualOdometry() :
  state_ ( INITIALIZING ), cur_frame_ ( nullptr ), 
  ref_frame_ ( nullptr ), map_ ( new Map ), 
  num_lost_ ( 0 ), num_inliers_ ( 0 )
{
  num_of_features_    = Config::get<int> ( "number_of_features" );
  scale_factor_       = Config::get<double> ( "scale_factor" );
  level_pyramid_      = Config::get<int> ( "level_pyramid" );
  match_ratio_        = Config::get<float> ( "match_ratio" );
  max_num_lost_       = Config::get<float> ( "max_num_lost" );
  min_inliers_        = Config::get<int> ( "min_inliers" );
  key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
  key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
  ORB_detector = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

// VisualOdometry::~VisualOdometry(){}

bool VisualOdometry::addFrame(Frame::Ptr new_frame){
  switch(state_){
    // first case.
    case INITIALIZING:{
      state_ = OK;
      cur_frame_ = ref_frame_ = new_frame;
      // std::cout << Sophus::SE3().inverse() << std::endl;
      map_->insertKeyFrame(new_frame);
      // detect keypoints, compute descriptors for 1st frame.
      extractKeypoints();
      computeDescriptor();
      setPointCloud();
      break;
    }

    case OK:{
      cur_frame_ = new_frame;
      cur_frame_->T_c_w_ = ref_frame_->T_c_w_;
      // detect keypoints, compute descriptors, find matches.
      extractKeypoints();
      computeDescriptor();
      findFeatureMatches();
      poseEstimationPnP();

      if (checkEstimatedPose()){
        cur_frame_->T_c_w_ = T_c_r_estimated * ref_frame_->T_c_w_;
        ref_frame_ = cur_frame_;
        setPointCloud();
        num_lost_ = 0;
        if (checkKeyFrame())
          addKeyFrame();
      }
      else{
        num_lost_ += 1;
        if (num_lost_ > max_num_lost_){
          state_ = LOST;
        }
        return false;
      }
      break;
    }

    case LOST:{
      std::cout << "vo has lost." << std::endl;
      break;
    }
  }
  return true;
}

void VisualOdometry::extractKeypoints(){
  kpts_.clear();
  ORB_detector->detect(cur_frame_->color_, kpts_);
  // std::cout << "keypoints: " << kpts_.size() << std::endl;
}

void VisualOdometry::computeDescriptor(){
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  descriptor->compute(cur_frame_->color_, kpts_, des_cur_);
}

void VisualOdometry::findFeatureMatches(){
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> raw_match;
  matcher->match(des_ref_, des_cur_, raw_match);

  double min_dist = 100000, max_dist = 0;
  for (int i = 0; i < des_ref_.rows; i++){
    if (raw_match[i].distance > max_dist)
      max_dist = raw_match[i].distance;
    if (raw_match[i].distance < min_dist)
      min_dist = raw_match[i].distance;
  }

  matches.clear();
  for (int i = 0; i < des_ref_.rows; i++){
    if (raw_match[i].distance < std::max(30.0, 2*min_dist))
      matches.push_back(raw_match[i]);
  }
  std::cout << "match size: " << matches.size() << std::endl;
}

void VisualOdometry::poseEstimationPnP(){
  std::vector<cv::Point3f> pts3d;
  std::vector<cv::Point2f> pts2d;
  for (auto m : matches){
    pts3d.push_back(ref_points_3d_[m.queryIdx]);
    pts2d.push_back(kpts_[m.trainIdx].pt);
  }
  cv::Mat K = ref_frame_->camera_->getIntrinsic();
  cv::Mat rvec, tvec, inliers;
  cv::solvePnPRansac( pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );

  num_inliers_ = inliers.rows;
  std::cout << "PnP inliers: " << num_inliers_ << std::endl;
  // T_c_r_estimated = Sophus::SE3(
  //   Sophus::SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
  //   Eigen::Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
  // );
  Eigen::Quaterniond q =
        Eigen::AngleAxisd(rvec.at<double>(0, 0), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(rvec.at<double>(1, 0), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rvec.at<double>(2, 0), Eigen::Vector3d::UnitZ());
  T_c_r_estimated = Sophus::SE3(q, Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                                tvec.at<double>(2, 0)));
  // std::cout << T_c_r_estimated * Sophus::SE3().inverse() << std::endl << std::endl;
}

void VisualOdometry::setPointCloud(){
  // After solving pnp.
  // loop througn current points, 
  // create point cloud for next iteration.
  ref_points_3d_.clear();
  des_ref_ = cv::Mat();
  cv::Mat K = ref_frame_->camera_->getIntrinsic();
  // std::cout << "K: " << std::endl << K << std::endl;
  // std::cout << "depth scale: " << ref_frame_->camera_->depth_scale_ << std::endl;
  for (int i = 0; i < kpts_.size(); i++){
    double d = ref_frame_->findDepth(kpts_[i]);
    // std::cout << d << std::endl;
    if (d > 0)
    {
      Eigen::Vector3d p = ref_frame_->camera_->pixel2camera(
                        Eigen::Vector2d(kpts_[i].pt.x, kpts_[i].pt.y), d);
    
      ref_points_3d_.push_back(cv::Point3f(p(0, 0), p(1, 0), p(2, 0)));
      des_ref_.push_back(des_cur_.row(i));
    }
  }
}

void VisualOdometry::addKeyFrame(){
  std::cout << "adding a key frame" << std::endl;
  map_->insertKeyFrame(cur_frame_);
}

bool VisualOdometry::checkEstimatedPose(){
  // 1. check num_inliers.
  if (num_inliers_ < min_inliers_){
    std::cout << "inliers too little." << std::endl;
    return false;
  }

  // 2. check pose norm.
  Sophus::Vector6d pose = T_c_r_estimated.log();
  if (pose.norm() > 5.0){
    std::cout << "motion too large" << std::endl;
    return false;
  }
}

bool VisualOdometry::checkKeyFrame(){
  Sophus::Vector6d d = T_c_r_estimated.log();
  Eigen::Vector3d trans = d.head<3>();
  Eigen::Vector3d rot = d.tail<3>();
  if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
      return true;
  return false;
}

}