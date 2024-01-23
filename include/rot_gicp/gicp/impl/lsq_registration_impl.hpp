#include <rot_gicp/gicp/lsq_registration.hpp>

#include <boost/format.hpp>
#include <rot_gicp/so3/so3.hpp>

namespace fast_gicp {

template <typename PointTarget, typename PointSource>
LsqRegistration<PointTarget, PointSource>::LsqRegistration() {
  this->reg_name_ = "LsqRegistration";
  max_iterations_ = 64;
  rotation_epsilon_ = 2e-3;
  transformation_epsilon_ = 5e-4;

  // lsq_optimizer_type_ = LSQ_OPTIMIZER_TYPE::LevenbergMarquardt;
  lsq_optimizer_type_ = LSQ_OPTIMIZER_TYPE::SO3_LevenbergMarquardt;
  lm_debug_print_ = false;
  lm_max_iterations_ = 10;
  lm_init_lambda_factor_ = 1e-9;
  lm_lambda_ = -1.0;

  final_hessian_.setIdentity();
  so3_final_hessian_.setIdentity();
}

template <typename PointTarget, typename PointSource>
LsqRegistration<PointTarget, PointSource>::~LsqRegistration() {}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setRotationEpsilon(double eps) {
  rotation_epsilon_ = eps;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setInitialLambdaFactor(double init_lambda_factor) {
  lm_init_lambda_factor_ = init_lambda_factor;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setDebugPrint(bool lm_debug_print) {
  lm_debug_print_ = lm_debug_print;
}

template <typename PointTarget, typename PointSource>
const Eigen::Matrix<double, 6, 6>& LsqRegistration<PointTarget, PointSource>::getFinalHessian() const {
  return final_hessian_;
}

template <typename PointTarget, typename PointSource>
double LsqRegistration<PointTarget, PointSource>::evaluateCost(const Eigen::Matrix4f& relative_pose, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  return this->linearize(Eigen::Isometry3f(relative_pose).cast<double>(), H, b);
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::computeTranslation(PointCloudSource& output, Eigen::Vector3d& trans,
                                                                   const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0, 
                                                                   const double interval_tn, const double interval_tn_1) {
  // Eigen::Vector3d t0 = Eigen::Vector3d(trans.template cast<double>());
  Eigen::Vector3d t0 = trans;
  lm_lambda_ = -1.0;
  converged_ = false;

  for (int i = 0; i < max_iterations_ && !converged_; i++) {
    nr_iterations_ = i;
    Eigen::Vector3d delta_t;  // delta是每次迭代的变化量（更新量），x0为最终要输出的变换，x0=Sigma(delta)
    if (!step_t_optimize(t0, delta_t, init_guess, last_t0, interval_tn, interval_tn_1)) {
      std::cerr << "lm not converged!!" << std::endl;
      break;
    }

    // converged_ = is_converged(delta_t);
  }
  // std::cout << "Optimized over!!" << std::endl;

  Eigen::Affine3f final_translation;
  final_translation.matrix() = Eigen::Matrix4f::Identity();
  final_translation.matrix().col(3).head<3>() = t0.cast<float>();
  pcl::transformPointCloud(*input_, output, final_translation);
  trans = t0;
}

//! 用LM算法求解CT平移优化
template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_t_optimize(Eigen::Vector3d& x0, Eigen::Vector3d& delta,
                                                                const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0, 
                                                                const double& interval_tn, const double& interval_tn_1) {
  Eigen::Matrix<double, 6, 6> H;  // 海森矩阵
  Eigen::Matrix<double, 6, 1> b;  // 偏置
  
  // std::cout << "x0: " << x0.matrix() << std::endl;
  double y0 = t3_linearize(x0, init_guess, last_t0, interval_tn, interval_tn_1, &H, &b); // y0：总误差，x0：变换矩阵
  // std::cout << "H: " << std::endl << H << std::endl;
  // std::cout << "b: " << std::endl << b.transpose() << std::endl;

  if (lm_lambda_ < 0.0) {
    lm_lambda_ = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
  }

  double nu = 2.0;
  for (int i = 0; i < lm_max_iterations_; i++) {
    // 利用Cholesky分解来求解线性方程组，Ax = b
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H + lm_lambda_ * Eigen::Matrix<double, 6, 6>::Identity());
    Eigen::Matrix<double, 6, 1> d = solver.solve(-b);
    // Eigen::Vector3d d_v(d(0), d(1), d(2));
    Eigen::Isometry3d delt_t = se3_exp(d); // SE3转换，得到deltT
    delta = delt_t.translation();

    // delta = d;
    // std::cout << "t_delta: " << delta.transpose() << std::endl;
    Eigen::Vector3d xi = delta + x0;
    double yi = compute_t_error(xi, init_guess, last_t0, interval_tn, interval_tn_1);  // 计算新的总误差
    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b)); // 分母为判断deltT的方向与误差变换的方向是否为同向

    if (lm_debug_print_) {
      if (i == 0) {
        std::cout << boost::format("--- LM optimization ---\n%5s %15s %15s %15s %15s %15s %5s\n") % "i" % "y0" % "yi" % "rho" % "lambda" % "|delta|" % "dec";
      }
      char dec = rho > 0.0 ? 'x' : ' ';
      std::cout << boost::format("%5d %15g %15g %15g %15g %15g %5c") % i % y0 % yi % rho % lm_lambda_ % d.norm() % dec << std::endl;
    }

    if (rho < 0) {
      if (is_t_converged(delta)) {
        return true;  // 判断是否收敛
      }

      lm_lambda_ = nu * lm_lambda_;
      nu = 2 * nu;  // 优化方向对，则增大步长，类似于梯度下降法
      continue;
    }
    // 方向不对，则减小步长，类似于GN
    x0 = xi;
    lm_lambda_ = lm_lambda_ * std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
    // so3_final_hessian_ = H; // 存储最新的海森矩阵
    return true;
  }

  return false;
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::is_t_converged(const Eigen::Vector3d& delta) const {

  Eigen::Vector3d t_delta = 1.0 / transformation_epsilon_ * delta.array().abs();

  return t_delta.maxCoeff() < 1;

}


template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  Eigen::Isometry3d x0 = Eigen::Isometry3d(guess.template cast<double>());
  // std::cout << "x00: " << x0.matrix() << std::endl;
  lm_lambda_ = -1.0;
  converged_ = false;
  
  if (lm_debug_print_) {
    std::cout << "********************************************" << std::endl;
    std::cout << "***************** optimize *****************" << std::endl;
    std::cout << "********************************************" << std::endl;
  }

  for (int i = 0; i < max_iterations_ && !converged_; i++) {
    nr_iterations_ = i;
    // Isometry3d是指的一次欧式变换，[R | t]_{4*4}
    Eigen::Isometry3d delta;  // delta是每次迭代的变化量（更新量），x0为最终要输出的变换，x0=Sigma(delta)
    if (!step_optimize(x0, delta)) {
      std::cerr << "lm not converged!!" << std::endl;
      break;
    }

    converged_ = is_converged(delta);
  }
  // std::cout << "Optimized over!!" << std::endl;

  final_transformation_ = x0.cast<float>().matrix();
  pcl::transformPointCloud(*input_, output, final_transformation_);
}
//! 判断收敛，依据是，求出的变换矩阵变化幅度很小-->收敛
template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::is_converged(const Eigen::Isometry3d& delta) const {
  double accum = 0.0;
  Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = delta.translation();

  Eigen::Matrix3d r_delta = 1.0 / rotation_epsilon_ * R.array().abs();
  Eigen::Vector3d t_delta = 1.0 / transformation_epsilon_ * t.array().abs();

  return std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) < 1;
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_optimize(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  switch (lsq_optimizer_type_) {
    case LSQ_OPTIMIZER_TYPE::LevenbergMarquardt:
      return step_lm(x0, delta);
    case LSQ_OPTIMIZER_TYPE::GaussNewton:
      return step_gn(x0, delta);
    case LSQ_OPTIMIZER_TYPE::SO3_LevenbergMarquardt:
      return rot_step_lm(x0, delta);
  }

  return step_lm(x0, delta);
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_gn(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;
  double y0 = linearize(x0, &H, &b);

  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H);
  Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

  delta = se3_exp(d);

  x0 = delta * x0;
  final_hessian_ = H;

  return true;
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_lm(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  Eigen::Matrix<double, 6, 6> H;  // 海森矩阵
  Eigen::Matrix<double, 6, 1> b;  // 偏置
  double y0 = linearize(x0, &H, &b); // y0：总误差，x0：变换矩阵
  if (lm_lambda_ < 0.0) {
    lm_lambda_ = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
  }

  double nu = 2.0;
  for (int i = 0; i < lm_max_iterations_; i++) {
    // 利用Cholesky分解来求解线性方程组，Ax = b
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H + lm_lambda_ * Eigen::Matrix<double, 6, 6>::Identity());
    Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

    delta = se3_exp(d); // SE3转换，得到deltT

    Eigen::Isometry3d xi = delta * x0;
    double yi = compute_error(xi);  // 计算新的总误差
    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b)); // 分母为判断deltT的方向与误差变换的方向是否为同向

    if (lm_debug_print_) {
      if (i == 0) {
        std::cout << boost::format("--- LM optimization ---\n%5s %15s %15s %15s %15s %15s %5s\n") % "i" % "y0" % "yi" % "rho" % "lambda" % "|delta|" % "dec";
      }
      char dec = rho > 0.0 ? 'x' : ' ';
      std::cout << boost::format("%5d %15g %15g %15g %15g %15g %5c") % i % y0 % yi % rho % lm_lambda_ % d.norm() % dec << std::endl;
    }

    if (rho < 0) {
      if (is_converged(delta)) {
        return true;  // 判断是否收敛
      }

      lm_lambda_ = nu * lm_lambda_;
      nu = 2 * nu;  // 优化方向对，则增大步长，类似于梯度下降法
      continue;
    }
    // 方向不对，则减小步长，类似于GN
    x0 = xi;
    lm_lambda_ = lm_lambda_ * std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
    final_hessian_ = H; // 存储最新的海森矩阵
    return true;
  }

  return false;
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::rot_step_lm(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  Eigen::Matrix<double, 3, 3> H;  // 海森矩阵
  Eigen::Matrix<double, 3, 1> b;  // 偏置
  
  // std::cout << "x0: " << x0.matrix() << std::endl;
  double y0 = so3_linearize(x0, &H, &b); // y0：总误差，x0：变换矩阵
  // std::cout << "H: " << std::endl << H << std::endl;

  if (lm_lambda_ < 0.0) {
    lm_lambda_ = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
  }

  double nu = 2.0;
  for (int i = 0; i < lm_max_iterations_; i++) {
    // 利用Cholesky分解来求解线性方程组，Ax = b
    Eigen::LDLT<Eigen::Matrix<double, 3, 3>> solver(H + lm_lambda_ * Eigen::Matrix<double, 3, 3>::Identity());
    Eigen::Matrix<double, 3, 1> d = solver.solve(-b);
    // Eigen::Vector3d d_v(d(0), d(1), d(2));
    Eigen::Quaterniond q_delta = so3_exp(d); // 转换到SO3
    delta.setIdentity();
    delta.matrix().block<3,3>(0,0) = q_delta.toRotationMatrix();  // 旋转赋值
    // std::cout << "rot_delta: " << q_delta.toRotationMatrix() << std::endl;
    Eigen::Isometry3d xi = delta * x0;
    double yi = compute_error(xi);  // 计算新的总误差
    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b)); // 分母为判断deltT的方向与误差变换的方向是否为同向

    if (lm_debug_print_) {
      if (i == 0) {
        std::cout << boost::format("--- LM optimization ---\n%5s %15s %15s %15s %15s %15s %5s\n") % "i" % "y0" % "yi" % "rho" % "lambda" % "|delta|" % "dec";
      }
      char dec = rho > 0.0 ? 'x' : ' ';
      std::cout << boost::format("%5d %15g %15g %15g %15g %15g %5c") % i % y0 % yi % rho % lm_lambda_ % d.norm() % dec << std::endl;
    }

    if (rho < 0) {
      if (is_rot_converged(delta)) {
        return true;  // 判断是否收敛
      }

      lm_lambda_ = nu * lm_lambda_;
      nu = 2 * nu;  // 优化方向对，则增大步长，类似于梯度下降法
      continue;
    }
    // 方向不对，则减小步长，类似于GN
    x0 = xi;
    lm_lambda_ = lm_lambda_ * std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
    so3_final_hessian_ = H; // 存储最新的海森矩阵
    return true;
  }

  return false;
}

//! 判断收敛，依据是，求出的变换矩阵变化幅度很小-->收敛
template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::is_rot_converged(const Eigen::Isometry3d& delta) const {
  double accum = 0.0;
  Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();

  Eigen::Matrix3d r_delta = 1.0 / rotation_epsilon_ * R.array().abs();

  return r_delta.maxCoeff() < 1;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setOptimizerType(LSQ_OPTIMIZER_TYPE optimizier_type_){
  lsq_optimizer_type_ = optimizier_type_;
}
}  // namespace fast_gicp