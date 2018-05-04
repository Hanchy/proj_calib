#pragma once

#include "camera.h"

bool read_cams(const std::string &_intrinsic_dir,
               const std::string &_img_pts_dir,
               std::vector<Camera> &_cam_list);

typedef Camera Projector;
Projector read_proj(const std::string &_proj_file, 
                    const int _cols = 1024,
                    const int _rows = 768,
                    const int _cam_label = 0);
