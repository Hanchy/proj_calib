#include "show_cam.hpp"


void show_cam(std::vector<Camera> &_cams) {
  cv::viz::Viz3d myWindow("Frame");
  myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

  int i = 0;
  
  std::vector<cv::viz::Color> colors {
    cv::viz::Color::white(),
        cv::viz::Color::cyan(),
        cv::viz::Color::green(),
        cv::viz::Color::cherry(),
        cv::viz::Color::gold()};

  for (auto & cam : _cams) {
    cv::Mat &R = cam.R_;
    cv::Mat &t = cam.t_;
    cv::Mat &K = cam.intrinsic_;

    cv::Mat c = -R.t() * t;
  
    cv::Vec3d position(c.at<double>(0, 0), 
                       c.at<double>(1, 0),
                       c.at<double>(2, 0));
  
    cv::Mat Rt = R.t();
    cv::Vec3d focal_point(Rt.at<double>(0, 2), 
                          Rt.at<double>(1, 2),
                          Rt.at<double>(2, 2));

    cv::Vec3d y_dir(Rt.at<double>(0, 1), 
                    Rt.at<double>(1, 1),
                    Rt.at<double>(2, 1));


    cv::Vec3d x_dir(Rt.at<double>(0, 0), 
                    Rt.at<double>(1, 0),
                    Rt.at<double>(2, 0));

    cv::Matx33d k(K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
                  K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
                  K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2));
    

    cv::Affine3d cam_pose = 
        cv::viz::makeCameraPose(position, focal_point, y_dir);

    // cv::Affine3d transform = 
    //     cv::viz::makeTransformToGlobal(x_dir, y_dir, focal_point, position);
    cv::viz::WCameraPosition cpw(0.05);
    cv::viz::WCameraPosition cpw_frustum(k, 0.1, colors[i]);
    myWindow.showWidget("cpw"+std::to_string(i), cpw, cam_pose);
    myWindow.showWidget("cpw_frus"+std::to_string(i), cpw_frustum, cam_pose);
    i++;
  }
  
  // cv::Mat cloud = cvcloud_load(_cloud);
  // cv::viz::WCloud c_w(cloud, cv::viz::Color::green());
  // myWindow.showWidget("cloud", c_w, cv::Vec3d(0, 0, 0));

  myWindow.spin();
}
