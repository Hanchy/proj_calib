#include <algorithm>
#include <random>
#include "camera.h"
#include "read_data.hpp"
#include "SpacePoints.hpp"
#include "bundle_adjuster.hpp"

void draw_points(const Camera &_cam, cv::Mat &_plane);
void draw_r_points(const Camera &_cam, cv::Mat &_plane);
void common_points_idx(const Camera &_cam1, const Camera &_cam2,
                       std::vector<int> &_order_img);

void retrieve_points(const Camera &_cam, const std::vector<int> &_pt_idx,
                     std::vector<cv::Point2d> &_pts);

void construct_3d_pts(std::vector<Camera> &_cams,
                      SpacePoints<cv::Point3d> &_space_pts);

int main(int argc, char **argv) {
  
  if (argc != 3)
    return -1;
  std::vector<Camera> cams;
  read_cams(argv[1], argv[2], cams);
  
#if 0

  cv::namedWindow("pts0", cv::WINDOW_NORMAL);
  cv::namedWindow("pts1", cv::WINDOW_NORMAL);
  cv::namedWindow("pts2", cv::WINDOW_NORMAL);
  cv::namedWindow("pts3", cv::WINDOW_NORMAL);

  cv::Mat img0(cams[0].rows_, cams[0].cols_, 
               CV_32FC3, cv::Scalar(255,255,255));
  draw_points(cams[0], img0);
  draw_r_points(cams[0], img0);

  cv::Mat img1(cams[0].rows_, cams[0].cols_, 
               CV_32FC3, cv::Scalar(255,255,255));
  draw_points(cams[1], img1);
  draw_r_points(cams[1], img1);


  cv::Mat img2(cams[0].rows_, cams[0].cols_, 
               CV_32FC3, cv::Scalar(255,255,255));
  draw_points(cams[2], img2);

  cv::Mat img3(cams[0].rows_, cams[0].cols_, 
               CV_32FC3, cv::Scalar(255,255,255));
  draw_points(cams[3], img3);


  std::vector<int> common_labels;
  common_points_idx(cams[0], cams[1], common_labels);

  std::vector<int> common_pt_idx;
  for(int i = 0; i < 5; ++i)
    common_pt_idx.push_back(common_labels[i]);

  std::vector<cv::Point2d> common_pt1;
  retrieve_points(cams[0], common_pt_idx, common_pt1);
  std::cout << "common_pt1 " << common_pt1.size() << std::endl;
  for(auto &pt : common_pt1) {
    cv::circle(img0, pt, 13, cv::Scalar(255, 0, 0), 2);
  }

  std::vector<cv::Point2d> common_pt2;
  retrieve_points(cams[1], common_pt_idx, common_pt2);
  std::cout << "common_pt2 " << common_pt2.size() << std::endl;
  for(auto &pt : common_pt2) {
    cv::circle(img1, pt, 13, cv::Scalar(255, 0, 0), 2);
  }

  cv::Mat hcont;
  cv::hconcat(img0, img1, hcont);
  for (std::size_t i = 0; i < common_pt_idx.size(); ++i) {
    const auto &pt0 = common_pt1[i];
    auto pt1 = common_pt2[i] + cv::Point2d(cams[1].cols_, 0);
    cv::line(hcont, pt0, pt1, cv::Scalar(0, 255, 0), 2);
  }
  cv::imshow("pts0", img0);
  cv::imshow("pts1", img1);
  cv::imshow("pts2", img2);
  cv::imshow("pts3", img3);
  cv::waitKey(0);
#endif

  SpacePoints<cv::Point3d> space_points;
  construct_3d_pts(cams, space_points);






  return 0;
}


void draw_points(const Camera &_cam, cv::Mat &_plane) {
  // _plane = cv::Mat(_cam.rows_, _cam.cols_, 
  //                  CV_32FC3, cv::Scalar(255,255,255));
  
  for(auto &pt : _cam.img_pts_) {
    cv::circle(_plane, pt, 10, cv::Scalar(0, 0, 255), 2);
  }
}

void draw_r_points(const Camera &_cam, cv::Mat &_plane) {
  // _plane = cv::Mat(_cam.rows_, _cam.cols_, 
  //                  CV_32FC3, cv::Scalar(255,255,255));
  const auto &K = _cam.intrinsic_;
  for(const auto &pt : _cam.rectified_pts_) {
    double data[3] = {pt.x, pt.y, 1};
    cv::Mat h_pt(3, 1, CV_64F, data);
    h_pt = K * h_pt;
    cv::Point2d npt(data[0]/data[2], data[1]/data[2]);
    cv::circle(_plane, npt, 5, cv::Scalar(0, 0, 0), 2);
  }
}


template<typename Tk, typename Tv>
void retrieve_keys(const std::map<Tk, Tv> &_map,
                   std::vector<Tk> &_vk) {
  _vk.clear();

  std::for_each(_map.begin(), _map.end(), 
                [&_vk](const std::pair<Tk, Tv> &p) {
                  _vk.push_back(p.first);
                });
}

void common_points_idx(const Camera &_cam1, 
                       const Camera &_cam2,
                       std::vector<int> &_common_labels) {
  const auto &map_i_p1 = _cam1.l2imgpt_idx_;
  const auto &map_i_p2 = _cam2.l2imgpt_idx_;


  std::vector<int> key1;
  retrieve_keys(map_i_p1, key1);
  std::sort(key1.begin(), key1.end());

  
  std::vector<int> key2;
  retrieve_keys(map_i_p2, key2);
  std::sort(key2.begin(), key2.end());

  std::set_intersection(key1.begin(), key1.end(),
                        key2.begin(), key2.end(),
                        std::back_inserter(_common_labels));
}



void retrieve_points(const Camera &_cam, 
                     const std::vector<int> &_pt_idx,
                     std::vector<cv::Point2d> &_pts) {
  const auto &i_p_idx = _cam.l2imgpt_idx_;
  const auto &pts_buf = _cam.img_pts_;

  for (const auto i : _pt_idx) {
    auto search = i_p_idx.find(i);
    if (search != i_p_idx.end()) {
      auto buf_idx = search->second;
      _pts.push_back(pts_buf[buf_idx]);
    }
  }
}

void rectify_pts(const cv::Mat &_invK, 
                 const std::vector<cv::Point2d> &_input_pts,
                 std::vector<cv::Point2d> &_output_pts) {
  
  std::for_each(_input_pts.begin(), _input_pts.end(), 
                [&](const cv::Point2d &pt) {
                  cv::Mat homo = cv::Mat::ones(3, 1, CV_32F);
                  homo.at<float>(0, 0) = pt.x;
                  homo.at<float>(1, 0) = pt.y;
                  cv::Mat rectified = _invK * homo;
                  auto x = rectified.at<float>(0, 0)/rectified.at<float>(2, 0);
                  auto y = rectified.at<float>(1, 0)/rectified.at<float>(2, 0);
                  _output_pts.push_back(cv::Point2d(x, y));
                });
}

void retrieve_rectified_pts(const Camera &_cam, 
                            const std::vector<int> &_idx,
                            std::vector<cv::Point2d> &_r_pts) {

  const auto &i_p_idx = _cam.l2imgpt_idx_;
  const auto &pts_buf = _cam.rectified_pts_;

  for (const auto i : _idx) {
    auto search = i_p_idx.find(i);
    if (search != i_p_idx.end()) {
      auto buf_idx = search->second;
      _r_pts.push_back(pts_buf[buf_idx]);
    }
  }
}


bool triangulate_pts(Camera &_cam1,
                     Camera &_cam2,
                     SpacePoints<cv::Point3d> &_space_pts) {
  if (_cam1.cam_label_ == _cam2.cam_label_)
    return false;

  std::vector<int> cam_labels {_cam1.cam_label_, _cam2.cam_label_};


  BundleTwoCameras btc;

  std::vector<int> common_labels;
  common_points_idx(_cam1, _cam2, common_labels);

  cv::Mat homo_space_pts(4, common_labels.size(), CV_64F);

  std::vector<cv::Point3d> space_pts;
  space_pts.reserve(common_labels.size());

  cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat o = cv::Mat::zeros(3, 1, CV_64F);

  std::random_device rd;
  std::mt19937 g(rd());
  
  int num_it = 0;
  do {

    std::shuffle(common_labels.begin(), common_labels.end(), g);

    std::vector<cv::Point2d> rectified_pts1;
    retrieve_rectified_pts(_cam1, common_labels, rectified_pts1);
    std::vector<cv::Point2d> rectified_pts2;
    retrieve_rectified_pts(_cam2, common_labels, rectified_pts2);

    cv::Mat essentialMat = 
        cv::findEssentialMat(rectified_pts1, rectified_pts2,
                             1.0, cv::Point2d(0, 0), cv::RANSAC, 0.999, 0.1);

    cv::Mat R(3, 3, CV_64F);
    cv::Mat t(3, 1, CV_64F);
    cv::recoverPose(essentialMat, rectified_pts1, 
                    rectified_pts2, R, t);


    cv::Mat projMat1(3, 4, CV_64F);//  = cv::Mat::zeros(3, 4, CV_64F);
    cv::hconcat(I, o, projMat1);
    projMat1 = _cam1.intrinsic_ * projMat1;

    cv::Mat projMat2(3, 4, CV_64F);
    cv::hconcat(R, t, projMat2);
    projMat2 = _cam2.intrinsic_ * projMat2;

    std::vector<cv::Point2d> observes1;
    retrieve_points(_cam1, common_labels, observes1);
    std::vector<cv::Point2d> observes2;
    retrieve_points(_cam2, common_labels, observes2);


    cv::triangulatePoints(projMat1, projMat2, 
                          observes1, observes2, homo_space_pts);

    for (int i = 0; i < homo_space_pts.cols; ++i) {
      auto x = homo_space_pts.at<double>(0, i);
      auto y = homo_space_pts.at<double>(1, i);
      auto z = homo_space_pts.at<double>(2, i);
      auto u = homo_space_pts.at<double>(3, i);
      cv::Point3d current_pt(x/u, y/u, z/u);
      space_pts.push_back(current_pt);
    }

    std::array<cv::Mat, 2> Ks {{_cam1.intrinsic_, _cam2.intrinsic_}};
    std::array<cv::Mat, 2> Rs {{I, R}};
    std::array<cv::Mat, 2> ts {{o, t}};
    std::array<std::vector<cv::Point2d>, 2> observes{{observes1, observes2}};

    if (btc(Ks, Rs, ts, observes, space_pts)) {
      cv::Mat R12 = R * I.t(); // from I to R
      _cam2.R_ = R12 * _cam1.R_; 
      cv::Mat C2 = -R.t() * t; 
      cv::Mat Cc2 = _cam1.R_.t() * (I * C2 + o - _cam1.t_);
      _cam2.t_ = -_cam2.R_ * Cc2;
      break;
    } else {
      I = cv::Mat::eye(3, 3, CV_64F);
      o = cv::Mat::zeros(3, 1, CV_64F);
      space_pts.clear();
    }
    std::cout << "num of optimization iterated: " << num_it << std::endl;
  } while(++num_it < 30);

  if (num_it == 29)
    return false;

  for (std::size_t i = 0; i < space_pts.size(); ++i) {
    auto pt = space_pts[i];
    cv::Mat mpt(3, 1, CV_64F, &(pt.x));
    cv::Mat cpt = I * mpt + o; // to [I | o]
    
    const cv::Mat cam1_pt = _cam1.R_.t() * (cpt - _cam1.t_);

    const auto label = common_labels[i];
    cv::Point3d spt(cam1_pt.at<double>(0, 0), 
                    cam1_pt.at<double>(1, 0), 
                    cam1_pt.at<double>(2, 0));

    _space_pts.get_labeled_pt(label, spt);
    
    spt.x += cam1_pt.at<double>(0, 0);
    spt.y += cam1_pt.at<double>(1, 0);
    spt.z += cam1_pt.at<double>(2, 0);

    spt /= 2;
    _space_pts.insert(label, spt, cam_labels);
                      

    _cam1.space_pt_labels_.insert(label);
    _cam2.space_pt_labels_.insert(label);
  }
  return true;
}



void construct_3d_pts(std::vector<Camera> &_cams,
                      SpacePoints<cv::Point3d> &_space_pts) {
  triangulate_pts(_cams[2], _cams[0], _space_pts);

  // cv::viz::writeCloud("bsolved.ply", _space_pts.points_);


  std::vector<int> space_pts_labels;
  retrieve_keys(_space_pts.l2pt_idx_, space_pts_labels);
  std::vector<int> cam0_pts_labels;
  retrieve_keys(_cams[1].l2imgpt_idx_, cam0_pts_labels);
  
  std::vector<int> common_labels;
  std::set_intersection(cam0_pts_labels.begin(), cam0_pts_labels.end(),
                        space_pts_labels.begin(), space_pts_labels.end(),
                        std::back_inserter(common_labels));

  std::vector<cv::Point3d> objectPoints;
  objectPoints.reserve(common_labels.size());
  std::vector<cv::Point2d> imagePoints;
  imagePoints.reserve(imagePoints.size());

  for (const auto label : common_labels) {
    auto obs_i = _cams[1].l2imgpt_idx_[label];
    auto pts_i = _space_pts.l2pt_idx_[label];

    imagePoints.push_back(_cams[1].img_pts_[obs_i]);
    objectPoints.push_back(_space_pts.points_[pts_i]);
  }

  
  

}
                      
