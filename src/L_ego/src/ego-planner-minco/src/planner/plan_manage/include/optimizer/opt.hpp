
#include "poly_traj_utils.hpp"
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include <traj_utils/plan_container.hpp>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner {

// 需要提前声明
class PolyTrajOptimizerCeres;

class ConstraintPoints {
public:
  int cp_size;
  Eigen::MatrixXd points;
  std::vector<std::vector<Eigen::Vector3d>> base_point;
  std::vector<std::vector<Eigen::Vector3d>> direction;
  std::vector<bool> flag_temp;
  std::vector<double> times; // 每个约束点对应的时间（绝对时间）
  std::vector<int> segment_idx;     //所属段索引
  std::vector<double> normalized_t; //段内归一化时间

  void resize_cp(const int size_set) {
    cp_size = size_set;
    base_point.clear();
    direction.clear();
    flag_temp.clear();

    points.resize(3, size_set);
    base_point.resize(cp_size);
    direction.resize(cp_size);
    flag_temp.resize(cp_size);
    times.resize(size_set);
    segment_idx.resize(size_set);
    normalized_t.resize(size_set);
  }

  static inline int two_thirds_id(Eigen::MatrixXd &points,
                                  const bool touch_goal) {
    // return touch_goal ? points.cols() - 1
    //                   : points.cols() - 1 - (points.cols() - 2) / 3;
    return points.cols() - 1;
  }

  void segment(ConstraintPoints &buf, const int start, const int end) {
    if (start < 0 || end >= cp_size || points.rows() != 3) {
      ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
      return;
    }
    buf.resize_cp(end - start + 1);
    buf.points = points.block(0, start, 3, end - start + 1);
    buf.cp_size = end - start + 1;
    for (int i = start; i <= end; i++) {
      buf.base_point[i - start] = base_point[i];
      buf.direction[i - start] = direction[i];
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class PolyTrajOptimizerCeres {

private:
  template <typename EIGENVEC>
  void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT) {
    for (int i = 0; i < RT.size(); i++) {
      VT[i] = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void VirtualT2RealT(const EIGENVEC &Vt, Eigen::VectorXd &RT) {
    for (int i = 0; i < Vt.size(); i++) {
      RT(i) = Vt(i) > 0.0 ? ((0.5 * Vt(i) + 1.0) * Vt(i) + 1.0)
                          : 1.0 / ((0.5 * Vt(i) - 1.0) * Vt(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                        const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                        double &costT) {
    for (int i = 0; i < VT.size(); i++) {
      double gdVT2Rt;
      if (VT(i) > 0) {
        gdVT2Rt = VT(i) + 1.0; // dRT/dVT for VT > 0
      } else {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt); // dRT/dVT for VT < 0
      }
      gdVT(i) = (gdRT(i)) * gdVT2Rt;
    }
    costT = RT.sum();
  }

public:
  enum FORCE_STOP_OPTIMIZE_TYPE {
    DONT_STOP,
    STOP_FOR_REBOUND,
    STOP_FOR_ERROR
  } force_stop_type_;

  enum CHK_RET { OBS_FREE, ERR, FINISH };

private:
  GridMap::Ptr grid_map_;
  AStar::Ptr a_star_;
  poly_traj::MinJerkOpt jerkOpt_;
  SwarmTrajData *swarm_trajs_;
  ConstraintPoints cps_;

  int drone_id_;
  int cps_num_prePiece_;
  int piece_num_;
  bool touch_goal_;
  int iter_num_;

  std::vector<double> min_ellip_dist2_;

  // 多拓扑数据
  struct MultitopologyData_t {
    bool use_multitopology_trajs{false};
    bool initial_obstacles_avoided{false};
  } multitopology_data_;

  // 优化参数
  double wei_obs_, wei_obs_soft_;
  double wei_swarm_, wei_swarm_mod_;
  double wei_feas_;      // 通用可行性权重（保留，用于默认值）
  double wei_vel_feas_;  // 速度可行性独立权重
  double wei_acc_feas_;  // 加速度可行性独立权重
  double wei_jerk_feas_; // 加加速度可行性独立权重
  double wei_sqrvar_;
  double wei_time_;
  double obs_clearance_, obs_clearance_soft_, swarm_clearance_;
  double max_vel_, max_acc_, max_jer_;
  double max_inner_point_dev_;
  double t_now_;
  std::vector<std::vector<Eigen::Vector3d>> last_a_star_pathes_;

  // 残差统计变量（用于输出总残差）
  mutable double total_vel_cost_;
  mutable double total_acc_cost_;
  mutable double total_jerk_cost_;
  mutable double total_obs_cost_;
  mutable double total_smooth_cost_;
  mutable double total_sqrvar_cost_;
  mutable double total_time_cost_;

public:
  ros::Publisher direction_pub_;
  const std::vector<std::vector<Eigen::Vector3d>> &getLastAStarPaths() const {
    return last_a_star_pathes_;
  }
  void visualizeDirections();
  PolyTrajOptimizerCeres()
      : swarm_trajs_(nullptr), force_stop_type_(DONT_STOP),
        total_vel_cost_(0.0), total_acc_cost_(0.0), total_jerk_cost_(0.0),
        total_obs_cost_(0.0)
  {
    ros::NodeHandle nh;
    direction_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>("direction_markers", 1);
  }
  ~PolyTrajOptimizerCeres() {}

  using Ptr = std::shared_ptr<PolyTrajOptimizerCeres>;

  void setParam(ros::NodeHandle &nh);
  void setEnvironment(const GridMap::Ptr &map);
  void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
  void setDroneId(const int drone_id);

  void setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }
  void setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }
  void setUseMultitopologyTrajs(bool use_multitopology_trajs) {
    multitopology_data_.use_multitopology_trajs = use_multitopology_trajs;
  }

  inline const ConstraintPoints &getControlPoints(void) { return cps_; }
  inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
  inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
  inline double get_swarm_clearance_(void) { return swarm_clearance_; }

  // 主优化函数
  bool optimizeTrajectory(const Eigen::MatrixXd &iniState,
                          const Eigen::MatrixXd &finState,
                          const Eigen::MatrixXd &initInnerPts,
                          const Eigen::VectorXd &initT, double &final_cost);

  bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end,
                            PtsChk_t &points_check);

  CHK_RET
  finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                    const poly_traj::MinJerkOpt &pt_data,
                                    const bool flag_first_init = true);

  bool roughlyCheckConstraintPoints(void);
  bool allowRebound(void);

  std::vector<ConstraintPoints>
  distinctiveTrajs(std::vector<std::pair<int, int>> segments);

private:
  class JerkSmoothnessCostAnalytic : public ceres::CostFunction {
  public:
    PolyTrajOptimizerCeres *optimizer;
    JerkSmoothnessCostAnalytic(PolyTrajOptimizerCeres *opt,
                               const Eigen::Matrix3d &head_state,
                               const Eigen::Matrix3d &tail_state, int piece_num)
        : piece_num_(piece_num), optimizer(opt) {
      mutable_parameter_block_sizes()->push_back(3 *
                                                 (piece_num - 1)); // 内部控制点
      mutable_parameter_block_sizes()->push_back(piece_num); // 时间参数
      set_num_residuals(1);                                  // 单个残差
    }

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override {
      // 1. 解析参数
      // Eigen::Map<const Eigen::MatrixXd> P(parameters[0], 3, piece_num_ - 1);
      Eigen::Map<const Eigen::VectorXd> t(parameters[1], piece_num_);

      const double *p_params = parameters[0];
      const double *t_params = parameters[1];
      int piece_num = optimizer->piece_num_;
      int inner_pts_num = piece_num - 1;

      Eigen::MatrixXd P(3, inner_pts_num);

      for (int i = 0; i < inner_pts_num; i++) {
        P(0, i) = p_params[3 * i];
        P(1, i) = p_params[3 * i + 1];
        P(2, i) = p_params[3 * i + 2];
      }

      // 2. 虚拟时间 -> 实际时间
      Eigen::VectorXd T(piece_num_);
      optimizer->VirtualT2RealT(t, T);

      // 3. 生成轨迹并计算未加权的代价和梯度
      optimizer->jerkOpt_.generate(P, T);
      double smooth_cost; // 未加权
      Eigen::VectorXd grad_T(piece_num_);
      Eigen::MatrixXd grad_P(3, piece_num_ - 1);
      optimizer->jerkOpt_.initGradCost(grad_T, smooth_cost);
      optimizer->jerkOpt_.getGrad2TP(grad_T, grad_P);

      // 4. 获取权重
      double w = optimizer->wei_feas_; // 平滑性代价仍使用 wei_feas_

      // 5. 计算加权残差
      double residual = sqrt(2.0 * w * std::max(smooth_cost, 1e-12));
      residuals[0] = residual;
      optimizer->total_smooth_cost_ += residual;
      if (jacobians == nullptr)
        return true;

      // 6. 控制点的雅可比（加权）
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::MatrixXd> Jp(jacobians[0], 1, 3 * (piece_num_ - 1));
        double scale = (residual > 1e-12) ? w / residual : 0.0;
        for (int i = 0; i < piece_num_ - 1; i++) {
          for (int j = 0; j < 3; j++) {
            int idx = i * 3 + j;
            Jp(0, idx) = scale * grad_P(j, i);
          }
        }
      }

      // 7. 时间参数的雅可比（加权）
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::VectorXd> Jt(jacobians[1], piece_num_);
        double scale_t = (residual > 1e-12) ? w / residual : 0.0;
        for (int i = 0; i < piece_num_; i++) {
          double virtual_t = t(i);
          double dT_dt;
          if (virtual_t > 0.0) {
            dT_dt = virtual_t + 1.0;
          } else {
            double den = 0.5 * virtual_t * virtual_t - virtual_t + 1.0;
            dT_dt = (1.0 - virtual_t) / (den * den);
          }
          Jt(i) = scale_t * grad_T(i) * dT_dt;
        }
      }
      return true;
    }

  private:
    int piece_num_;
  };

  class ObstacleCostAnalytic : public ceres::CostFunction {
  private:
    PolyTrajOptimizerCeres *optimizer_;
    int point_idx_;
    int seg_idx_;
    double norm_t_;

    // 辅助函数：计算给定位置的障碍物目标值（非残差）
    double computeCost(const Eigen::Vector3d &pos) const {
      double cost = 0.0;
      const double d_safe = optimizer_->obs_clearance_;
      const double d_soft = optimizer_->obs_clearance_soft_;
      const double w_hard = optimizer_->wei_obs_;
      const double w_soft = optimizer_->wei_obs_soft_;
      const double r = 0.05;
      const double rsqr = r * r;

      for (size_t j = 0; j < optimizer_->cps_.direction[point_idx_].size(); ++j) {
        const auto &base = optimizer_->cps_.base_point[point_idx_][j];
        const auto &dir = optimizer_->cps_.direction[point_idx_][j];
        const double signed_dist = (pos - base).dot(dir);
        const double d_err = d_safe - signed_dist;
        const double d_err_soft = d_soft - signed_dist;

        if (d_err > 0.0) {
          cost += w_hard * std::pow(d_err, 3.0);
        }
        if (d_err_soft > 0.0) {
          const double term = std::sqrt(1.0 + d_err_soft * d_err_soft / rsqr);
          cost += w_soft * rsqr * (term - 1.0);
        }
      }
      return cost;
    }

    double computeResidual(const Eigen::Vector3d &pos) const {
      const double cost = computeCost(pos);
      return std::sqrt(std::max(0.0, 2.0 * cost));
    }

  public:
    ObstacleCostAnalytic(PolyTrajOptimizerCeres *optimizer, int point_idx,
                         int seg_idx, double norm_t)
        : optimizer_(optimizer), point_idx_(point_idx), seg_idx_(seg_idx),
          norm_t_(norm_t) {
      mutable_parameter_block_sizes()->push_back(
          3 * (optimizer->piece_num_ - 1)); // 控制点
      mutable_parameter_block_sizes()->push_back(optimizer->piece_num_); // 时间
      set_num_residuals(1);
    }

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override {
      const double *p_params = parameters[0];
      const double *t_params = parameters[1];
      int piece_num = optimizer_->piece_num_;
      int inner_pts_num = piece_num - 1;

    

      // 1. 重建控制点矩阵 P (3 x piece_num+1)
      Eigen::MatrixXd P(3, inner_pts_num);

      for (int i = 0; i < inner_pts_num; i++) {
        P(0, i) = p_params[3 * i];
        P(1, i) = p_params[3 * i + 1];
        P(2, i) = p_params[3 * i + 2];
      }

      // 2. 虚拟时间 -> 实际时间
      Eigen::Map<const Eigen::VectorXd> virtual_t(t_params, piece_num);
      Eigen::VectorXd T(piece_num);
      optimizer_->VirtualT2RealT(virtual_t, T);

      // 3. 更新轨迹（计算多项式系数）
      optimizer_->jerkOpt_.generate(P, T);

      // 4. 计算绝对时间
      double t_abs = 0.0;
      for (int i = 0; i < seg_idx_; i++) {
        t_abs += T[i];
      }
      t_abs += norm_t_ * T[seg_idx_];

      // 5. 获取该点的位置
      Eigen::Vector3d pos = optimizer_->jerkOpt_.getTraj().getPos(t_abs);

      // 6. 计算障碍物代价
      const double cost = computeCost(pos);
      residuals[0] = computeResidual(pos);
      optimizer_->total_obs_cost_ += cost;

      // 7. 数值微分雅可比
      if (jacobians) {
        double eps = 1e-6;
        double r0 = residuals[0];

        // 控制点雅可比
        if (jacobians[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jp(
              jacobians[0], 1, 3 * inner_pts_num);
          Jp.setZero();
          if (r0 > 0.0) {
            for (int i = 0; i < inner_pts_num; i++) {
              for (int k = 0; k < 3; k++) {
                int idx = 3 * i + k;
                // 正向扰动
                Eigen::MatrixXd P_plus = P;
                P_plus(k, i) += eps;
                optimizer_->jerkOpt_.generate(P_plus, T);
                double t_abs_plus = 0.0;
                for (int s = 0; s < seg_idx_; s++)
                  t_abs_plus += T[s];
                t_abs_plus += norm_t_ * T[seg_idx_];
                Eigen::Vector3d pos_plus =
                    optimizer_->jerkOpt_.getTraj().getPos(t_abs_plus);
                double residual_plus = computeResidual(pos_plus);

                // 反向扰动
                Eigen::MatrixXd P_minus = P;
                P_minus(k, i) -= eps;
                optimizer_->jerkOpt_.generate(P_minus, T);
                double t_abs_minus = 0.0;
                for (int s = 0; s < seg_idx_; s++)
                  t_abs_minus += T[s];
                t_abs_minus += norm_t_ * T[seg_idx_];
                Eigen::Vector3d pos_minus =
                    optimizer_->jerkOpt_.getTraj().getPos(t_abs_minus);
                double residual_minus = computeResidual(pos_minus);

                Jp(0, idx) = (residual_plus - residual_minus) / (2 * eps);
              }
            }
            optimizer_->jerkOpt_.generate(P, T); // 恢复
          }
        }

        // 时间参数雅可比
        if (jacobians[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jt(jacobians[1],
                                                                  1, piece_num);
          Jt.setZero();
          if (r0 > 0.0) {
            for (int i = 0; i < piece_num; i++) {
              // 正向扰动虚拟时间 i
              Eigen::VectorXd vt_plus = virtual_t;
              vt_plus(i) += eps;
              Eigen::VectorXd T_plus(piece_num);
              optimizer_->VirtualT2RealT(vt_plus, T_plus);
              optimizer_->jerkOpt_.generate(P, T_plus);
              double t_abs_plus = 0.0;
              for (int s = 0; s < seg_idx_; s++)
                t_abs_plus += T_plus[s];
              t_abs_plus += norm_t_ * T_plus[seg_idx_];
              Eigen::Vector3d pos_plus =
                  optimizer_->jerkOpt_.getTraj().getPos(t_abs_plus);
              double residual_plus = computeResidual(pos_plus);

              // 反向扰动
              Eigen::VectorXd vt_minus = virtual_t;
              vt_minus(i) -= eps;
              Eigen::VectorXd T_minus(piece_num);
              optimizer_->VirtualT2RealT(vt_minus, T_minus);
              optimizer_->jerkOpt_.generate(P, T_minus);
              double t_abs_minus = 0.0;
              for (int s = 0; s < seg_idx_; s++)
                t_abs_minus += T_minus[s];
              t_abs_minus += norm_t_ * T_minus[seg_idx_];
              Eigen::Vector3d pos_minus =
                  optimizer_->jerkOpt_.getTraj().getPos(t_abs_minus);
              double residual_minus = computeResidual(pos_minus);

              Jt(0, i) = (residual_plus - residual_minus) / (2 * eps);
            }
            optimizer_->jerkOpt_.generate(P, T); // 恢复
          }
        }
      }

      return true;
    }
  };

  class TimeCostAnalytic : public ceres::CostFunction {
  private:
    PolyTrajOptimizerCeres *optimizer_;
    int piece_num_;

  public:
    TimeCostAnalytic(PolyTrajOptimizerCeres *optimizer)
        : optimizer_(optimizer), piece_num_(optimizer->piece_num_) {
      mutable_parameter_block_sizes()->push_back(piece_num_);
      set_num_residuals(piece_num_);
    }

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override {
      Eigen::Map<const Eigen::VectorXd> virtual_t(parameters[0], piece_num_);
      Eigen::VectorXd real_T(piece_num_);
      optimizer_->VirtualT2RealT(virtual_t, real_T);
      double w = optimizer_->wei_time_;
      double sqrt_w = sqrt(w);
      double total_time_cost = 0.0;
      for (int i = 0; i < piece_num_; i++) {
        residuals[i] = sqrt_w * real_T[i];
        total_time_cost += residuals[i];
        optimizer_->total_time_cost_ += residuals[i];
      }
      if (jacobians != nullptr && jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
            J(jacobians[0], piece_num_, piece_num_);
        J.setZero();
        for (int i = 0; i < piece_num_; i++) {
          double t_i = virtual_t[i];
          double dT_dt;
          if (t_i > 0.0) {
            dT_dt = t_i + 1.0;
          } else {
            double den = 0.5 * t_i * t_i - t_i + 1.0;
            dT_dt = (1.0 - t_i) / (den * den);
          }
          J(i, i) = sqrt_w * dT_dt;
        }
      }
      return true;
    }
  };

  class DistanceSqrVarianceCostAnalytic : public ceres::CostFunction {
  private:
    PolyTrajOptimizerCeres *optimizer_;
    int piece_num_;
    int inner_pts_num_;
    double weight_;

  public:
    DistanceSqrVarianceCostAnalytic(PolyTrajOptimizerCeres *optimizer)
        : optimizer_(optimizer), piece_num_(optimizer->piece_num_),
          inner_pts_num_(piece_num_ - 1), weight_(optimizer->wei_sqrvar_) {
      mutable_parameter_block_sizes()->clear();
      mutable_parameter_block_sizes()->push_back(3 * inner_pts_num_);
      set_num_residuals(1);
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const override {
      const double *inner_pts_ptr = parameters[0];
      if (inner_pts_num_ <= 0) {
        residuals[0] = 0.0;
        if (jacobians != nullptr && jacobians[0] != nullptr) {
          memset(jacobians[0], 0, 3 * inner_pts_num_ * sizeof(double));
        }
        return true;
      }

      Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> inner_points(
          inner_pts_ptr, 3, inner_pts_num_);

      Eigen::MatrixXd control_points(3, piece_num_ + 1);
      const auto &initial_state = optimizer_->jerkOpt_.get_iniState();
      const auto &final_state = optimizer_->jerkOpt_.get_finState();

      control_points.col(0) = initial_state.col(0);
      control_points.middleCols(1, inner_pts_num_) = inner_points;
      control_points.col(piece_num_) = final_state.col(0);

      int segment_count = piece_num_;
      Eigen::MatrixXd displacement_vectors(3, segment_count);
      Eigen::VectorXd squared_distances(segment_count);

      for (int i = 0; i < segment_count; i++) {
        displacement_vectors.col(i) =
            control_points.col(i + 1) - control_points.col(i);
        squared_distances(i) = displacement_vectors.col(i).squaredNorm();
      }

      double mean_squared_dist = squared_distances.sum() / segment_count;
      double variance = 0.0;
      for (int i = 0; i < segment_count; i++) {
        double diff = squared_distances(i) - mean_squared_dist;
        variance += diff * diff / segment_count;
      }

      residuals[0] = sqrt(weight_ * variance);
      optimizer_->total_sqrvar_cost_ += residuals[0];

      if (jacobians != nullptr && jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>>
            jacobian_inner_points(jacobians[0], 1, 3 * inner_pts_num_);
        jacobian_inner_points.setZero();

        if (fabs(residuals[0]) < 1e-12) {
          return true;
        }

        Eigen::VectorXd dvariance_dsquared_dist(segment_count);
        for (int i = 0; i < segment_count; i++) {
          dvariance_dsquared_dist(i) =
              2.0 * (squared_distances(i) - mean_squared_dist) / segment_count;
        }

        Eigen::MatrixXd dsquared_dist_dpoints =
            Eigen::MatrixXd::Zero(3, piece_num_ + 1);
        for (int segment_idx = 0; segment_idx < segment_count; segment_idx++) {
          const Eigen::Vector3d &dp = displacement_vectors.col(segment_idx);
          int start_point_idx = segment_idx;
          int end_point_idx = segment_idx + 1;
          if (start_point_idx > 0) {
            dsquared_dist_dpoints.col(start_point_idx) += -2.0 * dp;
          }
          if (end_point_idx < piece_num_) {
            dsquared_dist_dpoints.col(end_point_idx) += 2.0 * dp;
          }
        }

        Eigen::MatrixXd dvariance_dpoints =
            Eigen::MatrixXd::Zero(3, piece_num_ + 1);
        for (int segment_idx = 0; segment_idx < segment_count; segment_idx++) {
          double factor = dvariance_dsquared_dist(segment_idx);
          int start_point_idx = segment_idx;
          int end_point_idx = segment_idx + 1;
          const Eigen::Vector3d &dp = displacement_vectors.col(segment_idx);
          if (start_point_idx > 0) {
            dvariance_dpoints.col(start_point_idx) += factor * (-2.0 * dp);
          }
          if (end_point_idx < piece_num_) {
            dvariance_dpoints.col(end_point_idx) += factor * (2.0 * dp);
          }
        }

        double derivative_factor = weight_ / (2.0 * residuals[0]);
        for (int inner_idx = 0; inner_idx < inner_pts_num_; inner_idx++) {
          int point_idx = inner_idx + 1;
          jacobian_inner_points(0, 3 * inner_idx) =
              derivative_factor * dvariance_dpoints(0, point_idx);
          jacobian_inner_points(0, 3 * inner_idx + 1) =
              derivative_factor * dvariance_dpoints(1, point_idx);
          jacobian_inner_points(0, 3 * inner_idx + 2) =
              derivative_factor * dvariance_dpoints(2, point_idx);
        }
      }
      return true;
    }
  };

  class AccelerationFeasibilityCostAnalytic : public ceres::CostFunction {
  private:
    PolyTrajOptimizerCeres *optimizer_;
    int seg_idx_;
    double t_norm_;

  public:
    AccelerationFeasibilityCostAnalytic(PolyTrajOptimizerCeres *optimizer,
                                        int seg_idx, double t_norm)
        : optimizer_(optimizer), seg_idx_(seg_idx), t_norm_(t_norm) {
      mutable_parameter_block_sizes()->push_back(
          3 * (optimizer->piece_num_ - 1)); // 控制点
      mutable_parameter_block_sizes()->push_back(optimizer->piece_num_); // 时间
      set_num_residuals(1);
    }

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override {
      const double *p_params = parameters[0];
      const double *t_params = parameters[1];

      int piece_num = optimizer_->piece_num_;
      if (seg_idx_ >= piece_num) {
        residuals[0] = 0.0;
        if (jacobians) {
          if (jacobians[0])
            memset(jacobians[0], 0, 3 * (piece_num - 1) * sizeof(double));
          if (jacobians[1])
            memset(jacobians[1], 0, piece_num * sizeof(double));
        }
        return true;
      }

      // 1. 重建控制点矩阵 P (3 x (piece_num+1))
      int inner_pts_num = piece_num - 1;
      Eigen::MatrixXd P(3, inner_pts_num);
      const auto &iniState = optimizer_->jerkOpt_.get_iniState();
      const auto &finState = optimizer_->jerkOpt_.get_finState();
      // P.col(0) = iniState.col(0);
      for (int i = 0; i < inner_pts_num; i++) {
        P(0, i) = p_params[3 * i];
        P(1, i) = p_params[3 * i + 1];
        P(2, i) = p_params[3 * i + 2];
      }
      // P.col(piece_num) = finState.col(0);

      // 2. 虚拟时间 -> 实际时间，并计算累积时间
      Eigen::Map<const Eigen::VectorXd> virtual_t(t_params, piece_num);
      Eigen::VectorXd T(piece_num);
      optimizer_->VirtualT2RealT(virtual_t, T);

      std::vector<double> cum_t(piece_num + 1, 0.0);
      for (int i = 0; i < piece_num; i++) {
        cum_t[i + 1] = cum_t[i] + T[i];
      }

      // 3. 更新轨迹（确保多项式系数最新）
      optimizer_->jerkOpt_.generate(P, T);

      // 4. 计算采样时间，获取加速度
      double t_abs = cum_t[seg_idx_] + t_norm_ * T[seg_idx_];
      Eigen::Vector3d acc = optimizer_->jerkOpt_.getTraj().getAcc(t_abs);
      double acc_norm = acc.norm();
      double max_acc = optimizer_->max_acc_;
      double w = optimizer_->wei_acc_feas_; // 使用独立的加速度权重

      // 5. 残差
      if (acc_norm > max_acc) {
        double exceed = acc_norm - max_acc;
        residuals[0] = sqrt(w) * exceed;
      } else {
        residuals[0] = 0.0;
      }

      // 累加到总加速度残差
      optimizer_->total_acc_cost_ += residuals[0];

      // 6. 雅可比（数值微分）
      if (jacobians != nullptr) {
        double eps = 1e-6;
        double r0 = residuals[0];

        // 控制点雅可比
        if (jacobians[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jp(
              jacobians[0], 1, 3 * inner_pts_num);
          Jp.setZero();

          if (r0 > 0.0) {
            for (int i = 0; i < inner_pts_num; i++) {
              for (int j = 0; j < 3; j++) {
                int idx = 3 * i + j;
                // 正向扰动
                Eigen::MatrixXd P_plus = P;
                P_plus(j, i) += eps;
                optimizer_->jerkOpt_.generate(P_plus, T);
                Eigen::Vector3d acc_plus =
                    optimizer_->jerkOpt_.getTraj().getAcc(t_abs);
                double r_plus = (acc_plus.norm() > max_acc)
                                    ? sqrt(w) * (acc_plus.norm() - max_acc)
                                    : 0.0;

                // 反向扰动
                Eigen::MatrixXd P_minus = P;
                P_minus(j, i) -= eps;
                optimizer_->jerkOpt_.generate(P_minus, T);
                Eigen::Vector3d acc_minus =
                    optimizer_->jerkOpt_.getTraj().getAcc(t_abs);
                double r_minus = (acc_minus.norm() > max_acc)
                                     ? sqrt(w) * (acc_minus.norm() - max_acc)
                                     : 0.0;

                Jp(0, idx) = (r_plus - r_minus) / (2 * eps);
              }
            }
            optimizer_->jerkOpt_.generate(P, T); // 恢复
          }
        }

        // 时间雅可比
        if (jacobians[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jt(jacobians[1],
                                                                  1, piece_num);
          Jt.setZero();

          if (r0 > 0.0) {
            for (int i = 0; i < piece_num; i++) {
              // 正向扰动虚拟时间
              Eigen::VectorXd vt_plus = virtual_t;
              vt_plus(i) += eps;
              Eigen::VectorXd T_plus(piece_num);
              optimizer_->VirtualT2RealT(vt_plus, T_plus);
              std::vector<double> cum_t_plus(piece_num + 1, 0.0);
              for (int k = 0; k < piece_num; k++)
                cum_t_plus[k + 1] = cum_t_plus[k] + T_plus[k];
              double t_abs_plus =
                  cum_t_plus[seg_idx_] + t_norm_ * T_plus[seg_idx_];
              optimizer_->jerkOpt_.generate(P, T_plus);
              Eigen::Vector3d acc_plus =
                  optimizer_->jerkOpt_.getTraj().getAcc(t_abs_plus);
              double r_plus = (acc_plus.norm() > max_acc)
                                  ? sqrt(w) * (acc_plus.norm() - max_acc)
                                  : 0.0;

              // 反向扰动
              Eigen::VectorXd vt_minus = virtual_t;
              vt_minus(i) -= eps;
              Eigen::VectorXd T_minus(piece_num);
              optimizer_->VirtualT2RealT(vt_minus, T_minus);
              std::vector<double> cum_t_minus(piece_num + 1, 0.0);
              for (int k = 0; k < piece_num; k++)
                cum_t_minus[k + 1] = cum_t_minus[k] + T_minus[k];
              double t_abs_minus =
                  cum_t_minus[seg_idx_] + t_norm_ * T_minus[seg_idx_];
              optimizer_->jerkOpt_.generate(P, T_minus);
              Eigen::Vector3d acc_minus =
                  optimizer_->jerkOpt_.getTraj().getAcc(t_abs_minus);
              double r_minus = (acc_minus.norm() > max_acc)
                                   ? sqrt(w) * (acc_minus.norm() - max_acc)
                                   : 0.0;

              Jt(0, i) = (r_plus - r_minus) / (2 * eps);
            }
            optimizer_->jerkOpt_.generate(P, T); // 恢复
          }
        }
      }

      return true;
    }
  };

  class VelocityFeasibilityCostAnalytic : public ceres::CostFunction {
  private:
    PolyTrajOptimizerCeres *optimizer_;
    int seg_idx_;
    double t_norm_;

  public:
    VelocityFeasibilityCostAnalytic(PolyTrajOptimizerCeres *optimizer,
                                    int seg_idx, double t_norm)
        : optimizer_(optimizer), seg_idx_(seg_idx), t_norm_(t_norm) {
      mutable_parameter_block_sizes()->clear();
      mutable_parameter_block_sizes()->push_back(
          3 * (optimizer->piece_num_ - 1)); // 控制点
      mutable_parameter_block_sizes()->push_back(optimizer->piece_num_); // 时间
      set_num_residuals(1);
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const override {
      const double *p_params = parameters[0];
      const double *t_params = parameters[1];

      int piece_num = optimizer_->piece_num_;
      if (seg_idx_ >= piece_num) {
        residuals[0] = 0.0;
        if (jacobians != nullptr) {
          if (jacobians[0] != nullptr) {
            memset(jacobians[0], 0, 3 * (piece_num - 1) * sizeof(double));
          }
          if (jacobians[1] != nullptr) {
            memset(jacobians[1], 0, piece_num * sizeof(double));
          }
        }
        return true;
      }

      // 1. 重建控制点矩阵 (仅内部点)
      int inner_pts_num = piece_num - 1;
      Eigen::MatrixXd P(3, inner_pts_num);
      for (int i = 0; i < inner_pts_num; i++) {
        P(0, i) = p_params[3 * i];
        P(1, i) = p_params[3 * i + 1];
        P(2, i) = p_params[3 * i + 2];
      }

      // 2. 虚拟时间 -> 实际时间，并计算采样绝对时间
      Eigen::Map<const Eigen::VectorXd> virtual_t(t_params, piece_num);
      Eigen::VectorXd T(piece_num);
      optimizer_->VirtualT2RealT(virtual_t, T);

      std::vector<double> cum_t(piece_num + 1, 0.0);
      for (int i = 0; i < piece_num; i++) {
        cum_t[i + 1] = cum_t[i] + T[i];
      }
      double t_abs = cum_t[seg_idx_] + t_norm_ * T[seg_idx_];

      // 3. 基于真实轨迹计算速度约束（与加速度/jerk约束一致）
      optimizer_->jerkOpt_.generate(P, T);
      Eigen::Vector3d vel = optimizer_->jerkOpt_.getTraj().getVel(t_abs);
      double vel_norm = vel.norm();
      double w = optimizer_->wei_vel_feas_;
      double max_vel = optimizer_->max_vel_;

      if (vel_norm > max_vel) {
        residuals[0] = sqrt(w) * (vel_norm - max_vel);
      } else {
        residuals[0] = 0.0;
      }
      optimizer_->total_vel_cost_ += residuals[0];

      // 4. 数值微分雅可比
      if (jacobians != nullptr) {
        const double eps = 1e-6;
        const double r0 = residuals[0];

        if (jacobians[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jp(
              jacobians[0], 1, 3 * inner_pts_num);
          Jp.setZero();

          if (r0 > 0.0) {
            for (int i = 0; i < inner_pts_num; i++) {
              for (int j = 0; j < 3; j++) {
                int idx = 3 * i + j;

                Eigen::MatrixXd P_plus = P;
                P_plus(j, i) += eps;
                optimizer_->jerkOpt_.generate(P_plus, T);
                Eigen::Vector3d vel_plus =
                    optimizer_->jerkOpt_.getTraj().getVel(t_abs);
                double r_plus = (vel_plus.norm() > max_vel)
                                    ? sqrt(w) * (vel_plus.norm() - max_vel)
                                    : 0.0;

                Eigen::MatrixXd P_minus = P;
                P_minus(j, i) -= eps;
                optimizer_->jerkOpt_.generate(P_minus, T);
                Eigen::Vector3d vel_minus =
                    optimizer_->jerkOpt_.getTraj().getVel(t_abs);
                double r_minus = (vel_minus.norm() > max_vel)
                                     ? sqrt(w) * (vel_minus.norm() - max_vel)
                                     : 0.0;

                Jp(0, idx) = (r_plus - r_minus) / (2 * eps);
              }
            }
            optimizer_->jerkOpt_.generate(P, T);
          }
        }

        if (jacobians[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jt(jacobians[1],
                                                                  1, piece_num);
          Jt.setZero();

          if (r0 > 0.0) {
            for (int i = 0; i < piece_num; i++) {
              Eigen::VectorXd vt_plus = virtual_t;
              vt_plus(i) += eps;
              Eigen::VectorXd T_plus(piece_num);
              optimizer_->VirtualT2RealT(vt_plus, T_plus);
              std::vector<double> cum_t_plus(piece_num + 1, 0.0);
              for (int k = 0; k < piece_num; k++)
                cum_t_plus[k + 1] = cum_t_plus[k] + T_plus[k];
              double t_abs_plus =
                  cum_t_plus[seg_idx_] + t_norm_ * T_plus[seg_idx_];
              optimizer_->jerkOpt_.generate(P, T_plus);
              Eigen::Vector3d vel_plus =
                  optimizer_->jerkOpt_.getTraj().getVel(t_abs_plus);
              double r_plus = (vel_plus.norm() > max_vel)
                                  ? sqrt(w) * (vel_plus.norm() - max_vel)
                                  : 0.0;

              Eigen::VectorXd vt_minus = virtual_t;
              vt_minus(i) -= eps;
              Eigen::VectorXd T_minus(piece_num);
              optimizer_->VirtualT2RealT(vt_minus, T_minus);
              std::vector<double> cum_t_minus(piece_num + 1, 0.0);
              for (int k = 0; k < piece_num; k++)
                cum_t_minus[k + 1] = cum_t_minus[k] + T_minus[k];
              double t_abs_minus =
                  cum_t_minus[seg_idx_] + t_norm_ * T_minus[seg_idx_];
              optimizer_->jerkOpt_.generate(P, T_minus);
              Eigen::Vector3d vel_minus =
                  optimizer_->jerkOpt_.getTraj().getVel(t_abs_minus);
              double r_minus = (vel_minus.norm() > max_vel)
                                   ? sqrt(w) * (vel_minus.norm() - max_vel)
                                   : 0.0;

              Jt(0, i) = (r_plus - r_minus) / (2 * eps);
            }
            optimizer_->jerkOpt_.generate(P, T);
          }
        }
      }

      return true;
    }
  };

  class JerkFeasibilityCostAnalytic : public ceres::CostFunction {
  private:
    PolyTrajOptimizerCeres *optimizer_;
    int seg_idx_;
    double t_norm_;

  public:
    JerkFeasibilityCostAnalytic(PolyTrajOptimizerCeres *optimizer, int seg_idx,
                                double t_norm)
        : optimizer_(optimizer), seg_idx_(seg_idx), t_norm_(t_norm) {
      mutable_parameter_block_sizes()->push_back(3 *
                                                 (optimizer->piece_num_ - 1));
      mutable_parameter_block_sizes()->push_back(optimizer->piece_num_);
      set_num_residuals(1);
    }

    bool Evaluate(double const *const *parameters, double *residuals,
                  double **jacobians) const override {
      const double *p_params = parameters[0];
      const double *t_params = parameters[1];

      int piece_num = optimizer_->piece_num_;
      if (seg_idx_ >= piece_num) {
        residuals[0] = 0.0;
        if (jacobians) {
          if (jacobians[0])
            memset(jacobians[0], 0, 3 * (piece_num - 1) * sizeof(double));
          if (jacobians[1])
            memset(jacobians[1], 0, piece_num * sizeof(double));
        }
        return true;
      }

      // 1. 重建控制点矩阵
      int inner_pts_num = piece_num - 1;
      Eigen::MatrixXd P(3, inner_pts_num);
      const auto &iniState = optimizer_->jerkOpt_.get_iniState();
      const auto &finState = optimizer_->jerkOpt_.get_finState();
      // P.col(0) = iniState.col(0);
      for (int i = 0; i < inner_pts_num; i++) {
        P(0, i) = p_params[3 * i];
        P(1, i ) = p_params[3 * i + 1];
        P(2, i ) = p_params[3 * i + 2];
      }
      // P.col(piece_num) = finState.col(0);

      // 2. 虚拟时间 -> 实际时间，累积时间
      Eigen::Map<const Eigen::VectorXd> virtual_t(t_params, piece_num);
      Eigen::VectorXd T(piece_num);
      optimizer_->VirtualT2RealT(virtual_t, T);

      std::vector<double> cum_t(piece_num + 1, 0.0);
      for (int i = 0; i < piece_num; i++) {
        cum_t[i + 1] = cum_t[i] + T[i];
      }

      // 3. 更新轨迹
      optimizer_->jerkOpt_.generate(P, T);

      // 4. 计算采样时间，获取jerk
      double t_abs = cum_t[seg_idx_] + t_norm_ * T[seg_idx_];
      Eigen::Vector3d jerk = optimizer_->jerkOpt_.getTraj().getJer(t_abs);
      double jerk_norm = jerk.norm();
      double max_jerk = optimizer_->max_jer_;
      double w = optimizer_->wei_jerk_feas_; // 使用独立的加加速度权重

      // 5. 残差
      if (jerk_norm > max_jerk) {
        double exceed = jerk_norm - max_jerk;
        residuals[0] = sqrt(w) * exceed;
      } else {
        residuals[0] = 0.0;
      }

      optimizer_->total_jerk_cost_ += residuals[0];

      // 6. 雅可比（数值微分）
      if (jacobians != nullptr) {
        double eps = 1e-6;
        double r0 = residuals[0];

        if (jacobians[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jp(
              jacobians[0], 1, 3 * inner_pts_num);
          Jp.setZero();

          if (r0 > 0.0) {
            for (int i = 0; i < inner_pts_num; i++) {
              for (int j = 0; j < 3; j++) {
                int idx = 3 * i + j;
                // 正向
                Eigen::MatrixXd P_plus = P;
                P_plus(j, i) += eps;
                optimizer_->jerkOpt_.generate(P_plus, T);
                Eigen::Vector3d jerk_plus =
                    optimizer_->jerkOpt_.getTraj().getJer(t_abs);
                double r_plus = (jerk_plus.norm() > max_jerk)
                                    ? sqrt(w) * (jerk_plus.norm() - max_jerk)
                                    : 0.0;

                // 反向
                Eigen::MatrixXd P_minus = P;
                P_minus(j, i) -= eps;
                optimizer_->jerkOpt_.generate(P_minus, T);
                Eigen::Vector3d jerk_minus =
                    optimizer_->jerkOpt_.getTraj().getJer(t_abs);
                double r_minus = (jerk_minus.norm() > max_jerk)
                                     ? sqrt(w) * (jerk_minus.norm() - max_jerk)
                                     : 0.0;

                Jp(0, idx) = (r_plus - r_minus) / (2 * eps);
              }
            }
            optimizer_->jerkOpt_.generate(P, T);
          }
        }

        if (jacobians[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, Eigen::Dynamic>> Jt(jacobians[1],
                                                                  1, piece_num);
          Jt.setZero();

          if (r0 > 0.0) {
            for (int i = 0; i < piece_num; i++) {
              // 正向扰动虚拟时间
              Eigen::VectorXd vt_plus = virtual_t;
              vt_plus(i) += eps;
              Eigen::VectorXd T_plus(piece_num);
              optimizer_->VirtualT2RealT(vt_plus, T_plus);
              std::vector<double> cum_t_plus(piece_num + 1, 0.0);
              for (int k = 0; k < piece_num; k++)
                cum_t_plus[k + 1] = cum_t_plus[k] + T_plus[k];
              double t_abs_plus =
                  cum_t_plus[seg_idx_] + t_norm_ * T_plus[seg_idx_];
              optimizer_->jerkOpt_.generate(P, T_plus);
              Eigen::Vector3d jerk_plus =
                  optimizer_->jerkOpt_.getTraj().getJer(t_abs_plus);
              double r_plus = (jerk_plus.norm() > max_jerk)
                                  ? sqrt(w) * (jerk_plus.norm() - max_jerk)
                                  : 0.0;

              // 反向
              Eigen::VectorXd vt_minus = virtual_t;
              vt_minus(i) -= eps;
              Eigen::VectorXd T_minus(piece_num);
              optimizer_->VirtualT2RealT(vt_minus, T_minus);
              std::vector<double> cum_t_minus(piece_num + 1, 0.0);
              for (int k = 0; k < piece_num; k++)
                cum_t_minus[k + 1] = cum_t_minus[k] + T_minus[k];
              double t_abs_minus =
                  cum_t_minus[seg_idx_] + t_norm_ * T_minus[seg_idx_];
              optimizer_->jerkOpt_.generate(P, T_minus);
              Eigen::Vector3d jerk_minus =
                  optimizer_->jerkOpt_.getTraj().getJer(t_abs_minus);
              double r_minus = (jerk_minus.norm() > max_jerk)
                                   ? sqrt(w) * (jerk_minus.norm() - max_jerk)
                                   : 0.0;

              Jt(0, i) = (r_plus - r_minus) / (2 * eps);
            }
            optimizer_->jerkOpt_.generate(P, T);
          }
        }
      }

      return true;
    }
  };
};

} // namespace ego_planner
