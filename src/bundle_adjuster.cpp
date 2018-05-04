#include "bundle_adjuster.hpp"

#include <ceres/rotation.h>


void AngleAxisRotatePoint(const double angle_axis[3], 
                          const double pt[3], double result[3]) {
  const double theta2 = ceres::DotProduct(angle_axis, angle_axis);
  if (theta2 > std::numeric_limits<double>::epsilon()) {

    const double theta = sqrt(theta2);
    const double costheta = cos(theta);
    const double sintheta = sin(theta);
    const double theta_inverse = 1.0 / theta;

    const double w[3] = { angle_axis[0] * theta_inverse,
                          angle_axis[1] * theta_inverse,
                          angle_axis[2] * theta_inverse };

    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    const double w_cross_pt[3] = { w[1] * pt[2] - w[2] * pt[1],
                                   w[2] * pt[0] - w[0] * pt[2],
                                   w[0] * pt[1] - w[1] * pt[0] };
    const double tmp =
        (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (1.0 - costheta);

    result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
    result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
    result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
  } else {

    const double w_cross_pt[3] = 
        { angle_axis[1] * pt[2] - angle_axis[2] * pt[1],
          angle_axis[2] * pt[0] - angle_axis[0] * pt[2],
          angle_axis[0] * pt[1] - angle_axis[1] * pt[0] };

    result[0] = pt[0] + w_cross_pt[0];
    result[1] = pt[1] + w_cross_pt[1];
    result[2] = pt[2] + w_cross_pt[2];
  }
}


struct StableCameraReprojectError{
  
  StableCameraReprojectError(const double _observed_x, 
                             const double _observed_y,
                             const double *_R, 
                             const double *_t)
      : observed_x_(_observed_x), 
        observed_y_(_observed_y),
        R_(_R), t_(_t) {}

  template <typename T>
  bool operator()(const T * const _point, // 3d space pt projected to the image
                  T *_residuals) const {

    T p[3];
    AngleAxisRotatePoint(R_, (double *)_point, (double *)p);
    p[0] += t_[0];
    p[1] += t_[1];
    p[2] += t_[2];

    p[0] /= p[2];
    p[1] /= p[2];

    _residuals[0] = observed_x_ - p[0];
    _residuals[1] = observed_y_ - p[1];
    
    return true;
  }


  static ceres::CostFunction *Create(const double _observed_x,
                                     const double _observed_y,
                                     const double * const _R,
                                     const double * const _t) {
    return (new ceres::AutoDiffCostFunction<StableCameraReprojectError, 2, 3> 
            (new StableCameraReprojectError(_observed_x, _observed_y, _R, _t)));
  }

  const double observed_x_;
  const double observed_y_;

  const double *R_; // AngleAxis form
  const double *t_;
};



struct CameraReprojectError{
  
  CameraReprojectError(double _observed_x, double _observed_y)
      : observed_x_(_observed_x), 
        observed_y_(_observed_y) {}

  template <typename T>
  bool operator()(const T *const _R, // AngleAxis form
                  const T *const _t, // translation
                  const T *const _point, // 3d space pt projected to the image
                  T *_residuals) const {

    T p[3];
    ceres::AngleAxisRotatePoint(_R, _point, p);
    p[0] += _t[0];
    p[1] += _t[1];
    p[2] += _t[2];

    p[0] /= p[2];
    p[1] /= p[2];

    _residuals[0] = observed_x_ - p[0];
    _residuals[1] = observed_y_ - p[1];
    
    return true;
  }


  static ceres::CostFunction *Create(const double _observed_x,
                                     const double _observed_y) {
    return (
        new ceres::AutoDiffCostFunction<CameraReprojectError, 2, 3, 3, 3> 
        (new CameraReprojectError(_observed_x, _observed_y)));
  }

  double observed_x_;
  double observed_y_;
};



struct KDistReprojectError{
  
  KDistReprojectError(double _observed_x, double _observed_y)
      : observed_x_(_observed_x), 
        observed_y_(_observed_y) {}

  template <typename T>
  bool operator()(const T *const _K,
                  const T *const _dist,
                  const T *const _R, // AngleAxis form
                  const T *const _t, // translation
                  const T *const _point, // 3d space pt projected to the image
                  T *_residuals) const {

    T p[3];
    ceres::AngleAxisRotatePoint(_R, _point, p);
    p[0] += _t[0];
    p[1] += _t[1];
    p[2] += _t[2];

    p[0] /= p[2];
    p[1] /= p[2];

    T r2 = p[0] * p[0] + p[1] * p[1];
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    T k1 = _dist[0];
    T k2 = _dist[1];
    T k3 = _dist[4];
    T p1 = _dist[2];
    T p2 = _dist[3];

    T radial_len = T(1) + k1 * r2 + k2 * r4 + k3 * r6;

    T x = p[0];
    T y = p[1];
    
    T xq = radial_len * x + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
    T yq = radial_len * y + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;
    
    T h[3] = {xq, yq, T(1)};

    T pp[3] = {T(0), T(0), T(0)};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        pp[i]  += _K[i * 3 + j] * h[j];
      }
    }

    pp[0] /= pp[2];
    pp[1] /= pp[2];

    _residuals[0] = observed_x_ - pp[0];
    _residuals[1] = observed_y_ - pp[1];
    
    return true;
  }


  static ceres::CostFunction *Create(const double _observed_x,
                                     const double _observed_y) {
    return (
        new 
        ceres::AutoDiffCostFunction<KDistReprojectError, 2, 9, 5, 3, 3, 3> 
        (new KDistReprojectError(_observed_x, _observed_y)));
  }

  double observed_x_;
  double observed_y_;
};





struct CameraKReprojectError{
  
  CameraKReprojectError(double _observed_x, double _observed_y)
      : observed_x_(_observed_x), 
        observed_y_(_observed_y) {}

  template <typename T>
  bool operator()(const T *const _K,
                  const T *const _R, // AngleAxis form
                  const T *const _t, // translation
                  const T *const _point, // 3d space pt projected to the image
                  T *_residuals) const {

    T p[3];
    ceres::AngleAxisRotatePoint(_R, _point, p);
    p[0] += _t[0];
    p[1] += _t[1];
    p[2] += _t[2];


    T pp[3] = {T(0), T(0), T(0)};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        pp[i]  += _K[i * 3 + j] * p[j];
      }
    }

    pp[0] /= pp[2];
    pp[1] /= pp[2];

    _residuals[0] = observed_x_ - pp[0];
    _residuals[1] = observed_y_ - pp[1];
    
    return true;
  }


  static ceres::CostFunction *Create(const double _observed_x,
                                     const double _observed_y) {
    return (
        new 
        ceres::AutoDiffCostFunction<CameraKReprojectError, 2, 9, 3, 3, 3> 
        (new CameraKReprojectError(_observed_x, _observed_y)));
  }

  double observed_x_;
  double observed_y_;
};



struct PointsKReprojectError{
  
  PointsKReprojectError(
      double *_K,
      double *_R,
      double *_t,
      double _observed_x, double _observed_y) 
      : K_(_K), R_(_R), t_(_t),
        observed_x_(_observed_x), 
        observed_y_(_observed_y) {}

  template <typename T>
  bool operator()(const T *const _point, // 3d space pt projected to the image
                  T *_residuals) const {

    T p[3];
    ceres::AngleAxisRotatePoint((T*)R_, _point, p);
    p[0] += t_[0];
    p[1] += t_[1];
    p[2] += t_[2];


    T pp[3] = {T(0), T(0), T(0)};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        pp[i]  += K_[i * 3 + j] * p[j];
      }
    }

    pp[0] /= pp[2];
    pp[1] /= pp[2];

    _residuals[0] = observed_x_ - pp[0];
    _residuals[1] = observed_y_ - pp[1];
    
    return true;
  }


  static ceres::CostFunction *Create(double *_K,
                                     double *_R,
                                     double *_t,
                                     const double _observed_x,
                                     const double _observed_y) {
    return (
        new 
        ceres::AutoDiffCostFunction<PointsKReprojectError, 2, 3> 
        (new PointsKReprojectError(_K, _R, _t,
                                   _observed_x, _observed_y)));
  }

  double *K_;
  double *R_;
  double *t_;

  double observed_x_;
  double observed_y_;
};



void BundleAllCameras::Optimize() {

  std::vector<double[3]> angle_axes(cams_->size());
  for (std::size_t i = 0; i < cams_->size(); ++i) {
    const auto &cam = cams_->at(i);
    const double *rot_mat = cam.R_.ptr<double>(0);
    ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(rot_mat),
                                     angle_axes[i]); // This is error prone
  }

  ceres::Problem problem;

  for (std::size_t i = 0; i < cams_->size(); ++i) {
    auto &cam = cams_->at(i);
    // std::cout << cam.cam_label_ << std::endl;
    const auto &labels = cam.space_pt_labels_; //////!!!!!
    
    for (const auto label : labels) {
      auto l2observ = cam.l2imgpt_idx_.find(label);
      if (l2observ == cam.l2imgpt_idx_.end()) {//
        std::cerr << "cannot find corresponding observe "
                  << cam.cam_label_ << '\t' << label << '\n';
        return;
      }

      const auto ob_idx = l2observ->second; // corresponding observe
      const auto &r_pt = cam.img_pts_[ob_idx];
    
      cv::Point3d *space_pt = space_pts_->get_labeled_pt_pointer(label);
      if (space_pt == nullptr) {// ignore observe without 3d points
        std::cerr << "cannot find a space point\n";
        return;
      }
    
      ceres::CostFunction * cost_function = 
          //CameraKReprojectError::Create(r_pt.x, r_pt.y);
          KDistReprojectError::Create(r_pt.x, r_pt.y);

      problem.AddResidualBlock(cost_function,
                               NULL, //new ceres::CauchyLoss(0.5),
                               cam.intrinsic_.ptr<double>(0),
                               cam.dist_coeff_.ptr<double>(0),
                               angle_axes[i],
                               cam.t_.ptr<double>(0),
                               &(space_pt->x));
      problem.SetParameterBlockConstant(cam.intrinsic_.ptr<double>(0));
      problem.SetParameterBlockConstant(cam.dist_coeff_.ptr<double>(0));
    }
  }
  problem.SetParameterBlockConstant(angle_axes[0]);
  problem.SetParameterBlockConstant((*cams_)[0].t_.ptr<double>(0));

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::JACOBI;
  options.max_num_iterations = 50;
  // options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  
  for (std::size_t i = 0; i < cams_->size(); ++i) {
    auto &cam = cams_->at(i);
    double *rot_mat = cam.R_.ptr<double>(0);
    ceres::AngleAxisToRotationMatrix(angle_axes[i],
                                     ceres::RowMajorAdapter3x3(rot_mat)); 
  }
}


bool BundleTwoCameras::operator()(
    std::array<cv::Mat, 2> &_Ks,
    std::array<cv::Mat, 2> &_dists,
    std::array<cv::Mat, 2> &_Rs,
    std::array<cv::Mat, 2> &_ts,
    std::array<std::vector<cv::Point2d>, 2> &_observes,
    std::vector<cv::Point3d> &_space_pts) {
  
  if (_space_pts.size() != _observes[0].size() &&
      _space_pts.size() != _observes[1].size())
    return false;


  std::array<double[3], 2> angle_axes = {{{0, 0, 0}, {0, 0, 0}}};
  for (int i = 0; i < 2; ++i) {
    const double *rot_mat = _Rs[i].ptr<double>(0);
    ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(rot_mat),
                                     angle_axes[i]);
  }

  ceres::Problem problem;
  for (int i = 0; i < 2; ++i) {
    for (std::size_t idx = 0; idx < _space_pts.size(); ++idx) {
      const auto &observe_pt = _observes[i][idx];
      ceres::CostFunction * cost_function = 
          //CameraKReprojectError::Create(observe_pt.x, observe_pt.y);
          KDistReprojectError::Create(observe_pt.x, observe_pt.y);

      auto &space_pt = _space_pts[idx];
      problem.AddResidualBlock(cost_function,
                               NULL,// new ceres::CauchyLoss(1),
                               _Ks[i].ptr<double>(0),
                               _dists[i].ptr<double>(0),
                               angle_axes[i],
                               _ts[i].ptr<double>(0),
                               &(space_pt.x));
    }
    problem.SetParameterBlockConstant(_Ks[i].ptr<double>(0));
    problem.SetParameterBlockConstant(_dists[i].ptr<double>(0));
  }
  problem.SetParameterBlockConstant(angle_axes[0]);
  problem.SetParameterBlockConstant(_ts[0].ptr<double>(0));

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::JACOBI;
  options.max_num_iterations = 15;
  // options.use_nonmonotonic_steps = true;
  
  // options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::CONVERGENCE)
    return true;
  else
    return false;
  
  for (int i = 0; i < 2; ++i) {
    double *rot_mat = _Rs[i].ptr<double>(0);
    ceres::AngleAxisToRotationMatrix(angle_axes[i],
                                     ceres::RowMajorAdapter3x3(rot_mat));
  }

  
  return true;
}



bool BundleTwoCameras::OptPoints(
    std::array<cv::Mat, 2> &_Ks,
    std::array<cv::Mat, 2> &_dists,
    std::array<cv::Mat, 2> &_Rs,
    std::array<cv::Mat, 2> &_ts,
    std::array<std::vector<cv::Point2d>, 2> &_observes,
    std::vector<cv::Point3d> &_space_pts) {
  
  if (_space_pts.size() != _observes[0].size() &&
      _space_pts.size() != _observes[1].size())
    return false;


  std::array<double[3], 2> angle_axes = {{{0, 0, 0}, {0, 0, 0}}};
  for (int i = 0; i < 2; ++i) {
    const double *rot_mat = _Rs[i].ptr<double>(0);
    ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(rot_mat),
                                     angle_axes[i]);
  }

  ceres::Problem problem;
  for (int i = 0; i < 2; ++i) {
    for (std::size_t idx = 0; idx < _space_pts.size(); ++idx) {
      const auto &observe_pt = _observes[i][idx];
      ceres::CostFunction * cost_function = 
          //    CameraKReprojectError::Create(observe_pt.x, observe_pt.y);
          KDistReprojectError::Create(observe_pt.x, observe_pt.y);

      auto &space_pt = _space_pts[idx];
      problem.AddResidualBlock(cost_function,
                               NULL, //new ceres::CauchyLoss(0.5),
                               _Ks[i].ptr<double>(0),
                               _dists[i].ptr<double>(0),
                               angle_axes[i],
                               _ts[i].ptr<double>(0),
                               &(space_pt.x));
    }
    problem.SetParameterBlockConstant(_Ks[i].ptr<double>(0));
    problem.SetParameterBlockConstant(_dists[i].ptr<double>(0));
    problem.SetParameterBlockConstant(angle_axes[i]);
    problem.SetParameterBlockConstant(_ts[i].ptr<double>(0));
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::JACOBI;
  options.max_num_iterations = 10;
  // options.use_nonmonotonic_steps = true;
  
  // options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::CONVERGENCE)
    return true;
  else
    return false;
  
  //  return true;

}



bool BundleCameraParameters::operator()(cv::Mat &_K,
                                        cv::Mat &_dists,
                                        cv::Mat &_R,
                                        cv::Mat &_t,
                                        std::vector<cv::Point2d> &_observes,
                                        std::vector<cv::Point3d> &_space_pts) {
  
  if (_space_pts.size() != _observes.size())
    return false;


  double angle_axis[3] = {0, 0, 0};
  const double *rot_mat = _R.ptr<double>(0);
  ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(rot_mat),
                                   angle_axis);
  
  ceres::Problem problem;
  for (std::size_t idx = 0; idx < _space_pts.size(); ++idx) {
    const auto &observe_pt = _observes[idx];
    ceres::CostFunction * cost_function = 
        KDistReprojectError::Create(observe_pt.x, observe_pt.y);

    auto &space_pt = _space_pts[idx];
    problem.AddResidualBlock(cost_function,
                             NULL, //new ceres::CauchyLoss(0.5),
                             _K.ptr<double>(0),
                             _dists.ptr<double>(0),
                             angle_axis,
                             _t.ptr<double>(0),
                             &(space_pt.x));
    problem.SetParameterBlockConstant(&(space_pt.x));
  }
  problem.SetParameterBlockConstant(_K.ptr<double>(0));
  problem.SetParameterBlockConstant(_dists.ptr<double>(0));

  
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.preconditioner_type = ceres::JACOBI;
  options.max_num_iterations = 1000;
  // options.use_nonmonotonic_steps = true;
  
  options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  if (summary.termination_type == ceres::CONVERGENCE)
    return true;
  else
    return false;
  
  double *rot = _R.ptr<double>(0);
  ceres::AngleAxisToRotationMatrix(angle_axis, 
                                   ceres::RowMajorAdapter3x3(rot));

  return true;
}
