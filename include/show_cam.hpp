#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "camera.h"


// static cv::Mat cvcloud_load(const std::string &_cloud)
// {
//   cv::Mat cloud(1, 616, CV_64FC3);
//   std::ifstream ifs(_cloud);
//   std::string str;
//   for(std::size_t i = 0; i < 12; ++i)
//     std::getline(ifs, str);
//   cv::Point3d* data = cloud.ptr<cv::Point3d>();
//   double dummy1, dummy2;
//   for(std::size_t i = 0; i < 616; ++i)
//     ifs >> data[i].x >> data[i].y >> data[i].z >> dummy1 >> dummy2;
//   // cloud *= 5.0f;
//   return cloud;
// }

void show_cam(std::vector<Camera> &_cams);
