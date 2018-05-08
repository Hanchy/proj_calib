#include "read_data.hpp"
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

bool read_cams(const std::string &_intrinsic_dir,
               const std::string &_img_pts_dir,
               std::vector<Camera> &_cam_list) {
  fs::path p_intrinsic_dir(_intrinsic_dir);
  if (!fs::exists(p_intrinsic_dir) || !fs::is_directory(p_intrinsic_dir)) 
    return false;

  fs::path p_img_pts_dir(_img_pts_dir);
  if (!fs::exists(p_img_pts_dir) || !fs::is_directory(p_img_pts_dir)) 
    return false;


  
  fs::directory_iterator intrinsic_it(p_intrinsic_dir);
  fs::directory_iterator img_pts_it(p_img_pts_dir);
  fs::directory_iterator end;
  int cnt_files = 
      std::count_if(intrinsic_it, end, 
                    static_cast<bool(*)(const fs::path &)>
                    (fs::is_regular_file));
  for (int i = 0; i < cnt_files; ++i) {
    std::string intrinsic_file = _intrinsic_dir;
    if (intrinsic_file.back() != '/')
      intrinsic_file += "/";
    intrinsic_file += "Camera" + std::to_string(i+1) + ".yml";
    
    std::string img_pts_file = _img_pts_dir;
    if (img_pts_file.back() != '/')
      img_pts_file += "/";
    img_pts_file += "Camera" + std::to_string(i+1) + ".txt";


    std::cout << intrinsic_file << '\t' << img_pts_file << std::endl;

    Camera cam(intrinsic_file, img_pts_file, i);
    _cam_list.push_back(cam);
  }

  return true;
}



Projector read_proj(const std::string &_proj_file,
                    const int _cols,
                    const int _rows,
                    const int _cam_label) {

  Projector proj(_proj_file, _cols, _rows, _cam_label);

  proj.intrinsic_ = cv::Mat::eye(3, 3, CV_64F);
  // 1.6181099752898197e+003, 0., 5.0007429573261237e+002, 0.,
  //     1.7160920068224825e+003, 5.0126179627357050e+002, 0., 0., 1.
  // proj.intrinsic_.at<double>(0, 0) = 1.6181099752898197e+003;
  // proj.intrinsic_.at<double>(0, 2) = 5.0007429573261237e+002;
  // proj.intrinsic_.at<double>(1, 1) = 1.7160920068224825e+003;
  // proj.intrinsic_.at<double>(1, 2) = 5.0126179627357050e+002;

  proj.intrinsic_.at<double>(0, 0) = 512;
  proj.intrinsic_.at<double>(0, 2) = 512;
  proj.intrinsic_.at<double>(1, 1) = 384;
  proj.intrinsic_.at<double>(1, 2) = 384;


  //-1.8229298965172336e-002, -7.3555538054537947e-003, 
  //-4.1185635516133516e-002, -1.3764460456398904e-002, 0.
  proj.dist_coeff_ = cv::Mat::zeros(5, 1, CV_64F);
  proj.dist_coeff_.at<double>(0, 0) = -1.8229298965172336e-002;
  proj.dist_coeff_.at<double>(1, 0) = -7.3555538054537947e-003;
  proj.dist_coeff_.at<double>(2, 0) = -4.1185635516133516e-002;
  proj.dist_coeff_.at<double>(3, 0) = -1.3764460456398904e-002;


  proj.R_ = cv::Mat::eye(3, 3, CV_64F);
  proj.t_ = cv::Mat::zeros(3, 1, CV_64F);
  
  return proj;
}
