#pragma once

#include <ceres/ceres.h>

#include "camera.h"
#include "SpacePoints.hpp"

#include <array>


class BundleAllCameras {
 public:
  BundleAllCameras(std::vector<Camera> *_cams, 
                   SpacePoints<cv::Point3d> *_space_pts) 
      : cams_(_cams), space_pts_(_space_pts) {}

  void Optimize();


  std::vector<Camera> *cams_;
  SpacePoints<cv::Point3d> *space_pts_;
};


class BundleTwoCameras {
 public:
  bool operator()(std::array<cv::Mat, 2> &_Ks,
                  std::array<cv::Mat, 2> &_Rs,
                  std::array<cv::Mat, 2> &_ts,
                  std::array<std::vector<cv::Point2d>, 2> &_observes,
                  std::vector<cv::Point3d> &_space_pts);
  

  //void Optimize();

  
};
