#include "saveply.hpp"

void SavePLY(const std::string &_file) {
  ofstream outfile("pointcloud.ply");
  outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
  outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << pointCloud.size() << "\n";
  outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
  outfile << "property list uchar int vertex_indices\n" << "end_header\n";
  for (int i = 0; i < pointCloud.size(); i++)
  {
    Point3d point = pointCloud.at(i).point;
    outfile << point.x << " ";
    outfile << point.y << " ";
    outfile << point.z << " ";
    outfile << "\n";
  }
  outfile.close();
}
