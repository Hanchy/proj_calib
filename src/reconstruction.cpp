#include "reconstruction.hpp"

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
      cv::Mat R12 = R; // from I to R
      _cam2.R_ = R12 * _cam1.R_; 
      cv::Mat C2 = -R.t() * t; 
      cv::Mat Cc2 = _cam1.R_.t() * (I * C2 + o - _cam1.t_);
      _cam2.t_ = -_cam2.R_ * Cc2;

      _cam1.intrinsic_ = Ks[0];
      _cam2.intrinsic_ = Ks[1];
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



void get_init_R_t(std::vector<Camera> &_cams,
                  Camera &_new_cam,
                  SpacePoints<cv::Point3d> &_space_pts) {
  std::vector<int> space_pts_labels;
  retrieve_keys(_space_pts.l2pt_idx_, space_pts_labels);

  std::vector<int> new_cam_pts_labels;
  retrieve_keys(_new_cam.l2imgpt_idx_, new_cam_pts_labels);

  std::vector<int> common_labels;
  std::set_intersection(space_pts_labels.begin(), space_pts_labels.end(),
                        new_cam_pts_labels.begin(), new_cam_pts_labels.end(),
                        std::back_inserter(common_labels));

  for(auto l : common_labels)
    _new_cam.space_pt_labels_.insert(l);

  std::vector<cv::Point3d> objectPoints;
  objectPoints.reserve(common_labels.size());
  std::vector<cv::Point2d> imagePoints;
  imagePoints.reserve(common_labels.size());

  for (const auto label : common_labels) {
    auto obs_i = _new_cam.l2imgpt_idx_[label];
    auto pts_i = _space_pts.l2pt_idx_[label];

    imagePoints.push_back(_new_cam.img_pts_[obs_i]);
    objectPoints.push_back(_space_pts.points_[pts_i]);
  }

  cv::Mat rvec;
  cv::Rodrigues(_cams.back().R_, rvec);
  cv::Mat tvec = _cams.back().t_.clone();
  cv::solvePnPRansac(objectPoints, imagePoints, 
                     _new_cam.intrinsic_, _new_cam.dist_coeff_, 
                     rvec, tvec, true, 300, 4);

  cv::Mat rot_mat;
  cv::Rodrigues(rvec, rot_mat);

  _new_cam.R_ = rot_mat.clone();
  _new_cam.t_ = tvec.clone();

}


void incremental_triangulate(std::vector<Camera> &_cams,
                             Camera &_new_cam,
                             SpacePoints<cv::Point3d> &_space_pts) {
  cv::Mat new_projMat(3, 4, CV_64F);
  cv::hconcat(_new_cam.R_, _new_cam.t_, new_projMat);
  new_projMat = _new_cam.intrinsic_ * new_projMat;


  std::vector<int> space_pts_labels;
  retrieve_keys(_space_pts.l2pt_idx_, space_pts_labels);

  std::vector<int> new_cam_pts_labels;
  retrieve_keys(_new_cam.l2imgpt_idx_, new_cam_pts_labels);

  std::vector<int> left_labels;
  std::set_difference(new_cam_pts_labels.begin(),
                      new_cam_pts_labels.end(),
                      space_pts_labels.begin(),
                      space_pts_labels.end(),
                      std::back_inserter(left_labels));

  for (auto &another_cam : _cams) {

    std::vector<int> another_labels;
    retrieve_keys(another_cam.l2imgpt_idx_, another_labels);
    
    std::vector<int> common_labels;
    std::set_intersection(left_labels.begin(), left_labels.end(),
                          another_labels.begin(), another_labels.end(),
                          std::back_inserter(common_labels));
    if (common_labels.empty())
      continue;

    decltype(common_labels) diff_labels;
    std::set_difference(left_labels.begin(), left_labels.end(),
                        common_labels.begin(), common_labels.end(),
                        std::back_inserter(diff_labels));
    left_labels = diff_labels;
    

    cv::Mat another_projMat(3, 4, CV_64F);
    cv::hconcat(another_cam.R_, another_cam.t_, another_projMat);
    another_projMat = another_cam.intrinsic_ * another_projMat;

    std::vector<cv::Point2d> new_obs;
    new_obs.reserve(common_labels.size());
    retrieve_points(_new_cam, common_labels, new_obs);
    std::vector<cv::Point2d> another_obs;
    another_obs.reserve(common_labels.size());
    retrieve_points(another_cam, common_labels, another_obs);

    cv::Mat homo_space_pts(4, common_labels.size(), CV_64F);

    cv::triangulatePoints(new_projMat, another_projMat, 
                          new_obs, another_obs, homo_space_pts);

    std::vector<cv::Point3d> space_pts;
    space_pts.reserve(common_labels.size());
    
    std::vector<int> cam_labels {{_new_cam.cam_label_, another_cam.cam_label_}};

    for (int i = 0; i < homo_space_pts.cols; ++i) {
      auto x = homo_space_pts.at<double>(0, i);
      auto y = homo_space_pts.at<double>(1, i);
      auto z = homo_space_pts.at<double>(2, i);
      auto u = homo_space_pts.at<double>(3, i);
      cv::Point3d current_pt(x/u, y/u, z/u);
      space_pts.push_back(current_pt);
      _new_cam.space_pt_labels_.insert(common_labels[i]);
    }

    std::cout << _new_cam.cam_label_ << '^' << another_cam.cam_label_ << '\t';
    std::cout << "points: " << space_pts.size() << std::endl;
    
    std::array<cv::Mat, 2> Ks{{_new_cam.intrinsic_, another_cam.intrinsic_}};
    std::array<cv::Mat, 2> Rs{{_new_cam.R_, another_cam.R_}};
    std::array<cv::Mat, 2> ts{{_new_cam.t_, another_cam.t_}};
    std::array<std::vector<cv::Point2d>, 2> observes{{new_obs, another_obs}};
    BundleTwoCameras btw_points;
    
    int num_it {0};
    while (num_it < 100) {
      auto s_pts = space_pts;
      if (btw_points.OptPoints(Ks, Rs, ts, observes, s_pts)) {
        space_pts = s_pts;
        break;
      }
      num_it++;
    }
    std::cout << "num_it " << num_it << std::endl;

    for(std::size_t i = 0; i < space_pts.size(); ++i) {
      auto &pt = space_pts[i];
      if (!_space_pts.insert(common_labels[i], pt, cam_labels))
        std::cerr << "coincidence points!\n";
      _new_cam.space_pt_labels_.insert(common_labels[i]);
    }    
  }

}



void construct_3d_pts(std::vector<Camera> &_cams,
                      SpacePoints<cv::Point3d> &_space_pts) {
  std::vector<int> cam_index {{0, 3, 2, 1}};
  auto it = cam_index.begin();
  
  triangulate_pts(_cams[*it], _cams[*(it+1)], _space_pts);
  // SavePLY("first.ply", _space_pts.points_);
  std::vector<Camera> cams{{_cams[*it], _cams[*(it+1)]}};
  BundleAllCameras bac(&cams, &_space_pts);

  for (it += 2; it != cam_index.end(); ++it) {
    get_init_R_t(cams, _cams[*it], _space_pts);
    cams.push_back(_cams[*it]);
    bac.Optimize();
    cams.pop_back();
    incremental_triangulate(cams, _cams[*it], _space_pts);
    cams.push_back(_cams[*it]);
    bac.Optimize();
  }

  // SavePLY("bac0321.ply", _space_pts.points_);

}
                      
