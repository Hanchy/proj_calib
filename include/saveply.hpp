#pragma once

#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

template <typename T>
void SavePLY(const std::string &_file, 
             const std::vector<cv::Point3_<T>> &_points) {

  std::ofstream outfile(_file);
  outfile << "ply\n" << "format ascii 1.0\n" 
          << "comment VTK generated PLY File\n";
  outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" 
          << "element vertex " << _points.size() << "\n";
  outfile << "property float x\n" << "property float y\n" 
          << "property float z\n" << "element face 0\n";
  outfile << "property list uchar int vertex_indices\n" << "end_header\n";

  for (std::size_t i = 0; i < _points.size(); i++) {
    const cv::Point3_<T> &point = _points.at(i);
    outfile << point.x << " ";
    outfile << point.y << " ";
    outfile << point.z << " ";
    outfile << "\n";
  }
  outfile.close();
}
