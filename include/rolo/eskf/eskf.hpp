#ifndef ROLO_ESKF_ESKF_HPP
#define ROLO_ESKF_ESKF_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "rolo/eskf/IKFoM_toolkit/esekfom/esekfom.hpp"
#include "rolo/utility.h"

namespace rolo {
namespace eskf {

using Vec3 = MTK::vect<3, double>;
using SO3 = MTK::SO3<double>;

MTK_BUILD_MANIFOLD(PoseState,
  ((Vec3, pos))
  ((SO3, rot))
  ((Vec3, vel))
  ((Vec3, omega))
  ((Vec3, acc))
  ((Vec3, alpha))
);

MTK_BUILD_MANIFOLD(PoseMeasurement,
  ((Vec3, pos))
  ((SO3, rot))
);

struct PoseInput {
  double dt = 0.0;
};

class PoseESEKF {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kStateDof = PoseState::DOF;
  static constexpr int kStateDim = PoseState::DIM;
  static constexpr int kMeasDof = PoseMeasurement::DOF;
  static constexpr int kProcessNoiseDof = 6;

  using Filter = esekfom::esekf<PoseState, kProcessNoiseDof, PoseInput, PoseMeasurement, kMeasDof>;
  using StateCov = typename Filter::cov;
  using ProcessNoiseCov = typename Filter::processnoisecovariance;
  using MeasurementNoiseCov = typename Filter::measurementnoisecovariance;
  using PoseVector = Eigen::Matrix<double, 7, 1>;
  using PoseVectorList = std::vector<PoseVector, Eigen::aligned_allocator<PoseVector>>;

  struct Options {
    double max_dt = 1.0;
    double q_linear_jerk_std = 0.5;
    double q_angular_jerk_std = 0.5;
    double r_position_std = 0.20;
    double r_rotation_std = 0.10;
    double init_position_std = 0.05;
    double init_rotation_std = 0.05;
    double init_velocity_std = 5.0;
    double init_angular_velocity_std = 2.0;
    double init_acceleration_std = 5.0;
    double init_angular_acceleration_std = 2.0;
    int maximum_iteration = 3;
    double convergence_limit = 1e-4;
  };

  PoseESEKF() : PoseESEKF(Options()) {}

  explicit PoseESEKF(Options options) : options_(options) {
    configure();
  }

  void setOptions(Options options) {
    options_ = options;
    configure();
  }

  bool initialized() const {
    return initialized_;
  }

  double lastTime() const {
    return last_time_;
  }

  void reset() {
    initialized_ = false;
    last_time_ = 0.0;
    StateCov P = initialCovariance();
    kf_.change_P(P);
  }

  void initialize(double stamp, Eigen::Vector3d position, Eigen::Quaterniond orientation) {
    PoseState state;
    state.pos = Vec3(position);
    state.rot = SO3(normalizedQuaternion(orientation));
    state.vel = Vec3(Eigen::Vector3d::Zero());
    state.omega = Vec3(Eigen::Vector3d::Zero());
    state.acc = Vec3(Eigen::Vector3d::Zero());
    state.alpha = Vec3(Eigen::Vector3d::Zero());

    StateCov P = initialCovariance();
    kf_.change_x(state);
    kf_.change_P(P);
    initialized_ = true;
    last_time_ = stamp;
  }

  bool processMeasurement(double stamp, Eigen::Vector3d position, Eigen::Quaterniond orientation) {
    MeasurementNoiseCov R = defaultMeasurementNoise();
    return processMeasurement(stamp, position, orientation, R);
  }

  bool processMeasurement(double stamp, Eigen::Vector3d position, Eigen::Quaterniond orientation, MeasurementNoiseCov R) {
    if(!initialized_) {
      initialize(stamp, position, orientation);
      return true;
    }

    double dt = stamp - last_time_;
    if(dt <= 0.0 || !std::isfinite(dt)) {
      return false;
    }

    if(dt > options_.max_dt) {
      initialize(stamp, position, orientation);
      return true;
    }

    PoseInput input;
    input.dt = dt;
    double predict_dt = dt;
    kf_.predict(predict_dt, Q_, input);

    PoseMeasurement z;
    z.pos = Vec3(position);
    z.rot = SO3(normalizedQuaternion(orientation));
    sanitizeMeasurementNoise(R);
    kf_.update_iterated(z, R);

    last_time_ = stamp;
    return true;
  }

  bool statePredict(double stamp) {
    if(!initialized_) {
      return false;
    }

    double dt = stamp - last_time_;
    if(dt <= 0.0 || !std::isfinite(dt)) {
      return false;
    }

    if(dt > options_.max_dt) {
      return false;
    }

    PoseInput input;
    input.dt = dt;
    double predict_dt = dt;
    kf_.predict(predict_dt, Q_, input);

    last_time_ = stamp;
    return true;
  }

  Eigen::Vector3d position() const {
    return Eigen::Vector3d(kf_.get_x().pos);
  }

  Eigen::Quaterniond orientation() const {
    Eigen::Quaterniond q(kf_.get_x().rot);
    q.normalize();
    return q;
  }

  Eigen::Vector3d velocity() const {
    return Eigen::Vector3d(kf_.get_x().vel);
  }

  Eigen::Vector3d angularVelocity() const {
    return Eigen::Vector3d(kf_.get_x().omega);
  }

  StateCov covariance() const {
    return kf_.get_P();
  }

  MeasurementNoiseCov defaultMeasurementNoise() const {
    MeasurementNoiseCov R = MeasurementNoiseCov::Zero();
    R.template block<3, 3>(0, 0).setIdentity();
    R.template block<3, 3>(3, 3).setIdentity();
    R.template block<3, 3>(0, 0) *= options_.r_position_std * options_.r_position_std;
    R.template block<3, 3>(3, 3) *= options_.r_rotation_std * options_.r_rotation_std;
    return R;
  }

  Eigen::Matrix<double, 6, 6> poseCovariance() const {
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
    StateCov P = kf_.get_P();
    cov.template block<3, 3>(0, 0) = P.template block<3, 3>(0, 0);
    cov.template block<3, 3>(0, 3) = P.template block<3, 3>(0, 3);
    cov.template block<3, 3>(3, 0) = P.template block<3, 3>(3, 0);
    cov.template block<3, 3>(3, 3) = P.template block<3, 3>(3, 3);
    return cov;
  }

  PoseVectorList statePropagate(double dt, double dis) const {
    PoseVectorList poses;
    if(!initialized_ || dt <= 0.0 || dis <= 0.0 || !std::isfinite(dt) || !std::isfinite(dis)) {
      return poses;
    }

    PoseState state = kf_.get_x();
    PoseInput input;
    input.dt = dt;
    Eigen::Vector3d last_pos(state.pos);
    double propagated_dis = 0.0;

    while(propagated_dis < dis) {
      Eigen::Matrix<double, kStateDim, 1> dx = processModel(state, input);
      state.oplus(dx, dt);

      Eigen::Vector3d pos(state.pos);
      double step_dis = (pos - last_pos).norm();
      if(!std::isfinite(step_dis) || step_dis < 1e-12) {
        break;
      }

      propagated_dis += step_dis;
      last_pos = pos;

      Eigen::Quaterniond q(state.rot);
      q.normalize();
      PoseVector pose;
      pose << pos.x(), pos.y(), pos.z(), q.x(), q.y(), q.z(), q.w();
      poses.push_back(pose);
    }

    return poses;
  }

private:
  void configure() {
    double limit[kStateDof];
    std::fill(limit, limit + kStateDof, options_.convergence_limit);
    kf_.init(processModel, processJacobian, processNoiseJacobian, measurementModel, measurementJacobian, measurementNoiseJacobian, options_.maximum_iteration, limit);
    Q_ = processNoise();
    reset();
  }

  StateCov initialCovariance() const {
    StateCov P = StateCov::Zero();
    setDiagBlock(P, 0, options_.init_position_std);
    setDiagBlock(P, 3, options_.init_rotation_std);
    setDiagBlock(P, 6, options_.init_velocity_std);
    setDiagBlock(P, 9, options_.init_angular_velocity_std);
    setDiagBlock(P, 12, options_.init_acceleration_std);
    setDiagBlock(P, 15, options_.init_angular_acceleration_std);
    return P;
  }

  ProcessNoiseCov processNoise() const {
    ProcessNoiseCov Q = ProcessNoiseCov::Zero();
    Q.template block<3, 3>(0, 0).setIdentity();
    Q.template block<3, 3>(3, 3).setIdentity();
    Q.template block<3, 3>(0, 0) *= options_.q_linear_jerk_std * options_.q_linear_jerk_std;
    Q.template block<3, 3>(3, 3) *= options_.q_angular_jerk_std * options_.q_angular_jerk_std;
    return Q;
  }

  static void setDiagBlock(StateCov& P, int idx, double stddev) {
    P.template block<3, 3>(idx, idx).setIdentity();
    P.template block<3, 3>(idx, idx) *= stddev * stddev;
  }

  static Eigen::Quaterniond normalizedQuaternion(Eigen::Quaterniond q) {
    if(!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || q.norm() < 1e-12) {
      return Eigen::Quaterniond::Identity();
    }
    q.normalize();
    return q;
  }

  static void sanitizeMeasurementNoise(MeasurementNoiseCov& R) {
    double min_var = 1e-12;
    for(int i = 0; i < kMeasDof; i++) {
      if(!std::isfinite(R(i, i)) || R(i, i) < min_var) {
        R(i, i) = min_var;
      }
    }
  }

  static Eigen::Matrix<double, kStateDim, 1> processModel(PoseState& s, PoseInput const& input) {
    Eigen::Matrix<double, kStateDim, 1> res = Eigen::Matrix<double, kStateDim, 1>::Zero();
    res.template block<3, 1>(0, 0) = Eigen::Vector3d(s.vel) + 0.5 * input.dt * Eigen::Vector3d(s.acc);
    res.template block<3, 1>(3, 0) = Eigen::Vector3d(s.omega) + 0.5 * input.dt * Eigen::Vector3d(s.alpha);
    res.template block<3, 1>(6, 0) = Eigen::Vector3d(s.acc);
    res.template block<3, 1>(9, 0) = Eigen::Vector3d(s.alpha);
    return res;
  }

  static Eigen::Matrix<double, kStateDim, kStateDof> processJacobian(PoseState& s, PoseInput const& input) {
    (void)s;
    Eigen::Matrix<double, kStateDim, kStateDof> F = Eigen::Matrix<double, kStateDim, kStateDof>::Zero();
    F.template block<3, 3>(0, 6).setIdentity();
    F.template block<3, 3>(0, 12) = 0.5 * input.dt * Eigen::Matrix3d::Identity();
    F.template block<3, 3>(3, 9).setIdentity();
    F.template block<3, 3>(3, 15) = 0.5 * input.dt * Eigen::Matrix3d::Identity();
    F.template block<3, 3>(6, 12).setIdentity();
    F.template block<3, 3>(9, 15).setIdentity();
    return F;
  }

  static Eigen::Matrix<double, kStateDim, kProcessNoiseDof> processNoiseJacobian(PoseState& s, PoseInput const& input) {
    (void)s;
    (void)input;
    Eigen::Matrix<double, kStateDim, kProcessNoiseDof> G = Eigen::Matrix<double, kStateDim, kProcessNoiseDof>::Zero();
    G.template block<3, 3>(12, 0).setIdentity();
    G.template block<3, 3>(15, 3).setIdentity();
    return G;
  }

  static PoseMeasurement measurementModel(PoseState& s, bool& valid) {
    valid = true;
    PoseMeasurement h;
    h.pos = s.pos;
    h.rot = s.rot;
    return h;
  }

  static Eigen::Matrix<double, kMeasDof, kStateDof> measurementJacobian(PoseState& s, bool& valid) {
    (void)s;
    valid = true;
    Eigen::Matrix<double, kMeasDof, kStateDof> H = Eigen::Matrix<double, kMeasDof, kStateDof>::Zero();
    H.template block<3, 3>(0, 0).setIdentity();
    H.template block<3, 3>(3, 3).setIdentity();
    return H;
  }

  static Eigen::Matrix<double, kMeasDof, kMeasDof> measurementNoiseJacobian(PoseState& s, bool& valid) {
    (void)s;
    valid = true;
    return Eigen::Matrix<double, kMeasDof, kMeasDof>::Identity();
  }

private:
  Options options_;
  Filter kf_;
  ProcessNoiseCov Q_;
  bool initialized_ = false;
  double last_time_ = 0.0;
};

}  // namespace eskf
}  // namespace rolo

#endif  // ROLO_ESKF_ESKF_HPP
