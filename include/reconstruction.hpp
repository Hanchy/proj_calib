#pragma once

#include <algorithm>
#include <random>
#include "camera.h"
#include "read_data.hpp"
#include "SpacePoints.hpp"
#include "bundle_adjuster.hpp"
#include "saveply.hpp"
#include "show_cam.hpp"



void common_points_idx(const Camera &_cam1, const Camera &_cam2,
                       std::vector<int> &_order_img);


void retrieve_points(const Camera &_cam, const std::vector<int> &_pt_idx,
                     std::vector<cv::Point2d> &_pts);


void construct_3d_pts(std::vector<Camera> &_cams,
                      SpacePoints<cv::Point3d> &_space_pts);


void recover_projector_matrix(Projector &_proj, Camera &_cam,
                              SpacePoints<cv::Point3d> &_space_pts);

void optimize_K_dist(Projector &_proj, SpacePoints<cv::Point3d> &_space_pts);

void bundle_cameras_projectors(std::vector<Camera> &_cams,
                              Projector &_proj,
                              SpacePoints<cv::Point3d> &_space_pts);


template<typename Tk, typename Tv>
void retrieve_keys(const std::map<Tk, Tv> &_map,
                   std::vector<Tk> &_vk) {
  _vk.clear();

  std::for_each(_map.begin(), _map.end(), 
                [&_vk](const std::pair<Tk, Tv> &p) {
                  _vk.push_back(p.first);
                });
}


