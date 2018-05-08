#pragma once

#include <ceres/ceres.h>

#include "camera.h"
#include "SpacePoints.hpp"

#include <array>
#include <initializer_list>

class OPTIMIZE_CONST_CHOICES {
 public:
  OPTIMIZE_CONST_CHOICES();
  OPTIMIZE_CONST_CHOICES(std::initializer_list<bool> _l);


  void set_all(bool state = false);

  bool &operator[](int i) {return fixes_[i];}
  bool operator[](int i) const {return fixes_[i];}
  
  bool &fix_K_() {return fixes_[0];}
  bool &fix_D_() {return fixes_[1];}
  bool &fix_R_() {return fixes_[2];}
  bool &fix_t_() {return fixes_[3];}
  bool &fix_P_() {return fixes_[4];} // 3d space points


  bool fix_K_() const {return fixes_[0];}
  bool fix_D_() const {return fixes_[1];}
  bool fix_R_() const {return fixes_[2];}
  bool fix_t_() const {return fixes_[3];}
  bool fix_P_() const {return fixes_[4];} // 3d space points

  friend std::ostream &operator<<(std::ostream &_out, 
                                  OPTIMIZE_CONST_CHOICES _choices);


  bool fixes_[5];
};





class BundleAllCameras {
 public:
  BundleAllCameras(std::vector<Camera> *_cams, 
                   SpacePoints<cv::Point3d> *_space_pts) 
      : cams_(_cams), space_pts_(_space_pts) {}

  void Optimize(bool _fix_pts = false);


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
                  std::vector<cv::Point3d> &_space_pts,
                  const OPTIMIZE_CONST_CHOICES _fix_choices);
};

