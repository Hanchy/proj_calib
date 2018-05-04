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
  // The _Ks and _dists are fixed
  // The _Rs[0] and _ts[0] are also fixed
  bool operator()(std::array<cv::Mat, 2> &_Ks,
                  std::array<cv::Mat, 2> &_dists,
                  std::array<cv::Mat, 2> &_Rs,
                  std::array<cv::Mat, 2> &_ts,
                  std::array<std::vector<cv::Point2d>, 2> &_observes,
                  std::vector<cv::Point3d> &_space_pts);
  
  // Only _space_pts are optimized
  bool OptPoints(std::array<cv::Mat, 2> &_Ks,
                 std::array<cv::Mat, 2> &_dists,
                 std::array<cv::Mat, 2> &_Rs,
                 std::array<cv::Mat, 2> &_ts,
                 std::array<std::vector<cv::Point2d>, 2> &_observes,
                 std::vector<cv::Point3d> &_space_pts);

  
};


class BundleCameraParameters {
 public:

  // _observes and _space_pts are fixed
  bool operator()(cv::Mat &_K,
                  cv::Mat &_dists,
                  cv::Mat &_R,
                  cv::Mat &_t,
                  std::vector<cv::Point2d> &_observes,
                  std::vector<cv::Point3d> &_space_pts);
};

