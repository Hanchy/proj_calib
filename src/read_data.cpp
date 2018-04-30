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
