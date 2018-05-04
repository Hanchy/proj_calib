#pragma once

#include <string>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>




class Camera {
 public:
  
  explicit Camera(const std::string &_intrinsic_file,
                  const std::string &_2d_pts_file, 
                  const int _cam_labels);

  explicit Camera(const std::string &_2d_pts_file,
                  const int _cols, const int _rows, 
                  const int _cam_label);
  
  void ReadObserves(const std::string &_2d_pts_file);


  cv::Mat intrinsic_;
  cv::Mat dist_coeff_;

  cv::Mat invK_;

  cv::Mat R_;
  cv::Mat t_;

  std::vector<cv::Point2d> img_pts_;
  std::vector<cv::Point2d> rectified_pts_;
  std::map<int, int> l2imgpt_idx_; // from the image order to the 2d_pts order
  std::set<int> space_pt_labels_; // point labels


  const int cols_;
  const int rows_;

  const int cam_label_;
  
 private:
  bool read_intrinsics(const std::string &_intrinsic_file);
  bool read_2d_pts(const std::string &_2d_pts_file);

};


typedef Camera Projector;


