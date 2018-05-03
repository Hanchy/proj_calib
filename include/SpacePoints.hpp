#pragma once

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <iterator>

template <typename T>
class SpacePoints {
 public:
  explicit SpacePoints();

  // bool insert(int _label, cv::Point3f &_pt);
  bool insert(const int _label, const T &_pt, const std::vector<int> &_cams);
  bool remove(const int _label);

  bool get_labeled_pt(const int _label, T &_pt);

  bool get_labeled_pt(const int _label, T &_pt, std::vector<int> &_cams);

  T *get_labeled_pt_pointer(const int _label);

  void clear();
  int size();
  
  

  std::vector<T> points_;
  std::map<int, int> l2pt_idx_;

  std::vector<std::vector<int>> pt2cams_; // map point to cams
  

 private:
  int tail_idx_;
};


template <typename T>
SpacePoints<T>::SpacePoints() : tail_idx_(0) {
}


template <typename T>
bool SpacePoints<T>::insert(const int _label, const T &_pt,
                            const std::vector<int> &_cams) {
  auto search = l2pt_idx_.find(_label);
  if (search != l2pt_idx_.end()) { // already exists, rewrite it
    points_[search->second] = _pt;

    std::vector<int> cam_union;
    std::set_union(pt2cams_[search->second].begin(), 
                   pt2cams_[search->second].end(),
                   _cams.begin(), _cams.end(),
                   std::back_inserter(cam_union));

    pt2cams_[search->second] = cam_union;

    return false;

  } else {
    points_.push_back(_pt);
    pt2cams_.push_back(_cams);

    l2pt_idx_.insert(std::make_pair(_label, tail_idx_));
    tail_idx_++;
  }
  return true;
}


template <typename T>
bool SpacePoints<T>::remove(const int _label) {
  auto search = l2pt_idx_.find(_label);
  if (search == l2pt_idx_.end())
    return false;

  auto pIt = points_.begin() + search->second;
  points_.erase(pIt);

  pt2cams_.erase(pt2cams_.begin() + search->second);

  l2pt_idx_.erase(search);
  tail_idx_--;

  for (auto & e :l2pt_idx_) {
    if (e.second > search->second)
      e.second--;
  }

  return true;
}


template <typename T>
bool SpacePoints<T>::get_labeled_pt(const int _label, T &_pt) {
  auto search = l2pt_idx_.find(_label);
  if (search == l2pt_idx_.end()) // not found
    return false;

  _pt = points_[search->second];
  return true;
}


template <typename T>
bool SpacePoints<T>::get_labeled_pt(const int _label, 
                                    T &_pt, std::vector<int> &_cams) {
  auto search = l2pt_idx_.find(_label);
  if (search == l2pt_idx_.end()) // not found
    return false;

  _pt = points_[search->second];
  _cams = pt2cams_[search->second];
  return true;
}


template <typename T>
T *SpacePoints<T>::get_labeled_pt_pointer(const int _label) {
  auto search = l2pt_idx_.find(_label);
  if (search == l2pt_idx_.end()) // not found
    return nullptr;

  return &(points_[search->second]);
}


template <typename T>
void SpacePoints<T>::clear() {
  points_.clear();
  pt2cams_.clear();
  l2pt_idx_.clear();
  tail_idx_ = 0;
}

template <typename T>
int SpacePoints<T>::size() {
  return tail_idx_;
}
