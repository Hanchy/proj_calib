#include "camera.h"
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>


Camera::Camera(const std::string &_intrinsic_file,
               const std::string &_2d_pts_file,
               const int _cam_label) : 
    cols_(2592), rows_(1944), cam_label_(_cam_label){

  R_ = cv::Mat::eye(3, 3, CV_64F);
  t_ = cv::Mat::zeros(3, 1, CV_64F);

  intrinsic_ = cv::Mat::eye(3, 3, CV_64F);
  dist_coeff_ = cv::Mat::zeros(5, 1, CV_64F);

  read_intrinsics(_intrinsic_file);
  read_2d_pts(_2d_pts_file);

}


bool Camera::read_intrinsics(const std::string &_intrinsic_file) {
  cv::FileStorage intrinsic_fs(_intrinsic_file, cv::FileStorage::READ);

  if (!intrinsic_fs.isOpened())
    return false;
  
  intrinsic_fs["cam1_K"] >> intrinsic_;
  intrinsic_fs["cam1_kc"] >> dist_coeff_;

  invK_ = intrinsic_.inv();

  return true;
}



bool Camera::read_2d_pts(const std::string &_2d_pts_file) {

  std::ifstream pts_file(_2d_pts_file);
  if (!pts_file.is_open()) {
    std::cerr << "Cannot open file: " << _2d_pts_file << std::endl;
    return false;
  }

  
  // std::vector<cv::Point2f> pts;

  std::string line;
  int i = 0;
  while(std::getline(pts_file, line)) {
    std::istringstream line_to_read(line);
    int order_img;
    cv::Point2d pt;
    
    line_to_read >> order_img >> pt.x >> pt.y;

    img_pts_.push_back(pt);
    l2imgpt_idx_.insert(std::make_pair(order_img, i));
    ++i;
  }
  // std::cout << img_pts_[0].x << ' ' << img_pts_[0].y << std::endl;
  cv::undistortPoints(img_pts_, rectified_pts_, intrinsic_, dist_coeff_);
  // std::cout << "read_2d_pts undist.size() " 
  //           << rectified_pts_.size() << std::endl;
  // std::cout << rectified_pts_[0].x << ' ' << rectified_pts_[0].y << std::endl;

  return true;
}


