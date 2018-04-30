#pragma once

#include "camera.h"

bool read_cams(const std::string &_intrinsic_dir,
               const std::string &_img_pts_dir,
               std::vector<Camera> &_cam_list);
