#include <algorithm>
#include <random>
#include "camera.h"
#include "read_data.hpp"
#include "SpacePoints.hpp"
#include "bundle_adjuster.hpp"
#include "saveply.hpp"
#include "show_cam.hpp"

#include "reconstruction.hpp"

void draw_points(const Camera &_cam, cv::Mat &_plane);
void draw_r_points(const Camera &_cam, cv::Mat &_plane);

int main(int argc, char **argv) {
  
  if (argc != 4)
    return -1;
  std::vector<Camera> cams;
  read_cams(argv[1], argv[2], cams);

  Projector proj = read_proj(argv[3], 1024, 768, 0);
  
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
  SavePLY("final.ply", space_points.points_);

  recover_projector_matrix(proj, space_points);
  
  cams.push_back(proj);
  show_cam(cams);

  return 0;
}


void draw_points(const Camera &_cam, cv::Mat &_plane) {
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

