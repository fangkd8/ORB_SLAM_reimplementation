// -------------- test the visual odometry -------------
#include <iostream>
#include <fstream>
#include <string>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "VisualOdometry/config.h"
#include "VisualOdometry/VisualOdometry.h"

using namespace std;
using namespace cv;

void writeTUM(vector<Sophus::SE3> poses, vector<long> timestamps, 
              Sophus::SE3 init, string file_name);

int main ( int argc, char** argv )
{
  if ( argc != 2 )
  {
      cout<<"usage: run_vo parameter_file"<<endl;
      return 1;
  }

  VisualOdometry::Config::setParameterFile ( argv[1] );
  VisualOdometry::VisualOdometry::Ptr vo ( new VisualOdometry::VisualOdometry );

  string dataset_dir = VisualOdometry::Config::get<string> ( "dataset_dir" );
  cout<<"dataset: "<<dataset_dir<<endl;
  ifstream fin ( dataset_dir+"/associate.txt" );
  if ( !fin )
  {
    cout<<"please generate the associate file called associate.txt!"<<endl;
    return 1;
  }

  vector<string> rgb_files, depth_files;
  vector<double> rgb_times, depth_times;
  while ( !fin.eof() )
  {
    string rgb_time, rgb_file, depth_time, depth_file;
    fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
    rgb_times.push_back ( atof ( rgb_time.c_str() ) );
    depth_times.push_back ( atof ( depth_time.c_str() ) );
    rgb_files.push_back ( dataset_dir+"/"+rgb_file );
    depth_files.push_back ( dataset_dir+"/"+depth_file );

    if ( fin.good() == false )
      break;
  }

  VisualOdometry::Camera::Ptr camera ( new VisualOdometry::Camera );
    
  // visualization
  cv::viz::Viz3d vis("Visual Odometry");
  cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
  cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
  cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
  vis.setViewerPose( cam_pose );
  
  world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
  camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
  vis.showWidget( "World", world_coor );
  vis.showWidget( "Camera", camera_coor );

  cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
  vector<Sophus::SE3> poses;
  vector<long> timestamps;
  for ( int i=0; i < rgb_files.size(); i++ )
  {
    Mat color = cv::imread ( rgb_files[i] );
    Mat depth = cv::imread ( depth_files[i], -1 );
    if ( color.data==nullptr || depth.data==nullptr )
        break;
    VisualOdometry::Frame::Ptr pFrame = VisualOdometry::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->color_ = color;
    pFrame->depth_ = depth;
    pFrame->timestamp_ = rgb_times[i];

    boost::timer timer;
    vo->addFrame ( pFrame );
    cout<<"VO costs time: "<<timer.elapsed()<<endl;
    
    if ( vo->state_ == VisualOdometry::VisualOdometry::LOST ){
        cout << "track lost" << endl;
        break;
    }
    // cout << i << endl << pFrame->T_c_w_ << pFrame->T_c_w_.inverse() << endl;
    Sophus::SE3 Tcw = pFrame->T_c_w_.inverse();
    poses.push_back(pFrame->T_c_w_);
    timestamps.push_back(pFrame->timestamp_);
    // cout << i << endl;
    // show the map and the camera pose 
    cv::Affine3d M(
        cv::Affine3d::Mat3( 
            Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
            Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
            Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
        ), 
        cv::Affine3d::Vec3(
            Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
        )
    );
    // cout << "M" << endl;
    cv::imshow("image", color );
    cv::waitKey(1);
    vis.setWidgetPose( "Camera", M);
    vis.spinOnce(1, false);
  }
  cout << "out of loop" << endl;

  // save trajectory
  double tx = VisualOdometry::Config::get<double>("tx");
  double ty = VisualOdometry::Config::get<double>("ty");
  double tz = VisualOdometry::Config::get<double>("tz");
  double qx = VisualOdometry::Config::get<double>("qx");
  double qy = VisualOdometry::Config::get<double>("qy");
  double qz = VisualOdometry::Config::get<double>("qz");
  double qw = VisualOdometry::Config::get<double>("qw");
  Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz);
  Eigen::Vector3d t = Eigen::Vector3d(tx, ty, tz);
  Sophus::SE3 init = Sophus::SE3(q, t);
  string file_name = VisualOdometry::Config::get<string>("traj_dir");

  writeTUM(poses, timestamps, init, file_name);
  // Warning: If get rid of waitkey(0), will cause memory leak.
  cv::waitKey(0);

  // cv::destroyAllWindows();
  cout << "exit" << endl;
  return EXIT_SUCCESS;
}

// timestamp tx ty tz qx qy qz qw
void writeTUM(vector<Sophus::SE3> poses, vector<long> timestamps, Sophus::SE3 init, string file_name){
  ofstream myfile;
  myfile.open (file_name);
  if (myfile.is_open()){
    cout << "opened." << endl;
  }
  else{
    cout << "no such file" << endl;
  }
  for (int i = 0; i < poses.size(); i++){
    // poses[i] *= init;
    Eigen::Vector3d tran = poses[i].translation();
    Eigen::Quaterniond q = poses[i].unit_quaternion();

    myfile << to_string(timestamps[i]) << " ";
    myfile << to_string(tran(0, 0)) << " ";
    myfile << to_string(tran(1, 0)) << " ";
    myfile << to_string(tran(2, 0)) << " ";
    myfile << to_string(q.vec()(0, 0)) << " ";
    myfile << to_string(q.vec()(1, 0)) << " ";
    myfile << to_string(q.vec()(2, 0)) << " ";
    myfile << to_string(q.w()) << "\n";
  }
  myfile.close();
}
