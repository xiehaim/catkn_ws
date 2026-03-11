
#include "optimizer/opt.hpp"
#include <iostream>
namespace ego_planner {
void PolyTrajOptimizerCeres::visualizeDirections() {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  // 确保轨迹已生成
  if (jerkOpt_.getTraj().getPieceNum() == 0) {
    ROS_WARN("No trajectory available for visualization.");
    return;
  }

  const poly_traj::Trajectory &traj = jerkOpt_.getTraj();
  const Eigen::VectorXd &durations = traj.getDurations();

  for (int i = 0; i < cps_.cp_size; i++) {
    if (cps_.direction[i].empty())
      continue;

    for (size_t j = 0; j < cps_.direction[i].size(); j++) {
      const Eigen::Vector3d &base = cps_.base_point[i][j]; // 基点（障碍物表面）
      const Eigen::Vector3d &dir =
          cps_.direction[i][j]; // 方向（从基点指向控制点，安全方向）
      int seg_idx = cps_.segment_idx[i];    // 所属段索引
      double norm_t = cps_.normalized_t[i]; // 段内归一化时间

      // 计算绝对时间 t_abs
      double t_abs = 0.0;
      for (int k = 0; k < seg_idx; k++)
      {
        t_abs += durations(k);
        std::cout << "k: " << k << " Tdurations(k): " << durations(k)
                  << "t_abs:" << t_abs << std::endl;
      }
      t_abs += norm_t * durations(seg_idx);
      std::cout << "Visualizing constraint point " << i << ", direction " << j
                << ": base=" << base.transpose() << ", dir=" << dir.transpose()
                << ", t_abs=" << t_abs << std::endl;
      // 获取轨迹点 pos
      Eigen::Vector3d pos = traj.getPos(t_abs);

      //时间计算有问题  
      // ----- 1. 基点（黄色球体）-----
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = "world";
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "base_points";
      sphere.id = id++;
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = base.x();
      sphere.pose.position.y = base.y();
      sphere.pose.position.z = base.z();
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.1;
      sphere.scale.y = 0.1;
      sphere.scale.z = 0.1;
      sphere.color.a = 1.0;
      sphere.color.r = 1.0;
      sphere.color.g = 1.0;
      sphere.color.b = 0.0;
      sphere.lifetime = ros::Duration(0);
      marker_array.markers.push_back(sphere);

      // ----- 2. 从基点指向轨迹点的箭头（显示有符号距离方向）-----
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "base_to_pos_arrows"; // 名称修改以反映方向变化
      arrow.id = id++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point p_start, p_end;
      p_start.x = base.x();
      p_start.y = base.y();
      p_start.z = base.z();
      p_end.x = pos.x();
      p_end.y = pos.y();
      p_end.z = pos.z();

      arrow.points.push_back(p_start);
      arrow.points.push_back(p_end);

      arrow.scale.x = 0.03; // 杆直径
      arrow.scale.y = 0.06; // 箭头头直径
      arrow.scale.z = 0.06;
      arrow.lifetime = ros::Duration(0);

      // 计算有符号距离： (pos - base) · dir
      double signed_dist = (pos - base).dot(dir);
      double d_safe = obs_clearance_;

      // 颜色指示 signed_dist 是否小于安全距离
      if (signed_dist < d_safe) {
        // 红色：轨迹点太靠近障碍物（被惩罚）
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
      } else {
        // 绿色：轨迹点安全
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
      }
      arrow.color.a = 1.0;

      marker_array.markers.push_back(arrow);

      
      // 在轨迹点位置显示红色球体，表示该点有障碍物影响
      visualization_msgs::Marker traj_point_marker;
      traj_point_marker.header.frame_id = "world";
      traj_point_marker.header.stamp = ros::Time::now();
      traj_point_marker.ns = "constrained_traj_points";
      traj_point_marker.id = id++; 
      traj_point_marker.type = visualization_msgs::Marker::SPHERE;
      traj_point_marker.action = visualization_msgs::Marker::ADD;
      traj_point_marker.pose.position.x = pos.x();
      traj_point_marker.pose.position.y = pos.y();
      traj_point_marker.pose.position.z = pos.z();
      std::cout<< "Visualizing constrained point at: (" << pos.x() << ", " << pos.y()
                << ", " << pos.z() << std::endl;
      traj_point_marker.pose.orientation.w = 1.0;
      traj_point_marker.scale.x = 0.05; // 可根据需要调整大小
      traj_point_marker.scale.y = 0.05;
      traj_point_marker.scale.z = 0.05;
      traj_point_marker.color.a = 1.0;
      traj_point_marker.color.r = 1.0; 
      traj_point_marker.color.g = 0.0;
      traj_point_marker.color.b = 1.0;
      traj_point_marker.lifetime = ros::Duration(0);
      marker_array.markers.push_back(traj_point_marker);
    }
  }

  direction_pub_.publish(marker_array);

}
bool PolyTrajOptimizerCeres::optimizeTrajectory(
    const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
    double &final_cost) {

  if (initInnerPts.cols() != (initT.size() - 1)) {
    ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
    return false;
  }

  ros::Time t0 = ros::Time::now(), t1, t2;
  int restart_nums = 0;
  bool flag_force_return = false;
  bool flag_still_unsafe = false;
  bool flag_success = false;
  bool flag_swarm_too_close = false;

  wei_swarm_mod_ = wei_swarm_;

  piece_num_ = initT.size();
  jerkOpt_.reset(iniState, finState, piece_num_);
  int inner_pts_num = piece_num_ - 1;

  int total_p_params = 3 * inner_pts_num;
  int total_t_params = piece_num_;

  double *p_params = new double[total_p_params];
  double *t_params = new double[total_t_params];

  memcpy(p_params, initInnerPts.data(), total_p_params * sizeof(double));
  RealT2VirtualT(initT, t_params);

  min_ellip_dist2_.resize(swarm_trajs_->size());

  do {
    flag_force_return = false;
    flag_still_unsafe = false;
    flag_success = false;
    flag_swarm_too_close = false;

    total_vel_cost_ = 0.0;
    total_acc_cost_ = 0.0;
    total_jerk_cost_ = 0.0;
    total_obs_cost_ = 0.0;
    total_smooth_cost_ = 0.0;
    total_sqrvar_cost_ = 0.0;
    total_time_cost_ = 0.0;

    std::cout << "=== Starting Ceres Optimization (Analytic) ===" << std::endl;
    std::cout << "Restart attempt: " << restart_nums + 1 << std::endl;

    // 如果cps_还没有初始化，需要先生成初始轨迹
    // if (cps_.cp_size == 0) {
    std::cout << "Generating initial trajectory..." << std::endl;

    Eigen::MatrixXd temp_inner_pts = initInnerPts;
    Eigen::VectorXd temp_times = initT;

    for (int i = 0; i < temp_times.size(); i++) {
      if (temp_times(i) < 0.1) {
        temp_times(i) = 0.1;
      }
    }

    jerkOpt_.generate(temp_inner_pts, temp_times);
    cps_.points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
    cps_.resize_cp(cps_.points.cols());

    std::vector<std::pair<int, int>> segments_nouse;
    CHK_RET check_result =
        finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false);
    // }

    // ============ 创建Ceres问题 ============
    ceres::Problem problem;

    // 1. 平滑性代价
    ceres::CostFunction *smoothness_cost = new JerkSmoothnessCostAnalytic(
        this, iniState, finState, piece_num_);
    problem.AddResidualBlock(smoothness_cost, NULL, p_params, t_params);
  
    // 2. 障碍物代价
    for (int i = 0; i < cps_.cp_size; i++) {
      if (!cps_.direction[i].empty()) {
        int seg_idx = cps_.segment_idx[i];
        double norm_t = cps_.normalized_t[i];
        ceres::CostFunction *obstacle_cost =
        new ObstacleCostAnalytic(this, i, seg_idx, norm_t);
        problem.AddResidualBlock(obstacle_cost, NULL, p_params, t_params);
      }
    }
 

        // 3. 距离平方方差代价
        ceres::CostFunction *variance_cost =
            new DistanceSqrVarianceCostAnalytic(this);
        problem.AddResidualBlock(variance_cost, NULL, p_params);

        // 4. 可行性代价（速度、加速度、加加速度）
        const int FEAS_SAMPLES_PER_SEGMENT = 3;
        for (int seg_idx = 0; seg_idx < piece_num_; seg_idx++) {
          for (int sample = 0; sample <= FEAS_SAMPLES_PER_SEGMENT; sample++) {
            double t_norm = (double)sample / FEAS_SAMPLES_PER_SEGMENT;

            ceres::CostFunction *vel_cost =
                new VelocityFeasibilityCostAnalytic(this, seg_idx, t_norm);
            problem.AddResidualBlock(vel_cost, NULL, p_params, t_params);

            ceres::CostFunction *acc_cost =
                new AccelerationFeasibilityCostAnalytic(this, seg_idx, t_norm);
            problem.AddResidualBlock(acc_cost, NULL, p_params, t_params);

            // ceres::CostFunction *jerk_cost =
            //     new JerkFeasibilityCostAnalytic(this, seg_idx, t_norm);
            // problem.AddResidualBlock(jerk_cost, NULL, p_params, t_params);
          }
        }

        // 5. 时间代价
        ceres::CostFunction *time_cost = new TimeCostAnalytic(this);
        problem.AddResidualBlock(time_cost, NULL, t_params);

        // 参数边界
        for (int i = 0; i < 3; i++) {
          problem.SetParameterLowerBound(p_params, i, p_params[i] - 0.5);
          problem.SetParameterUpperBound(p_params, i, p_params[i] + 0.5);
        }
        if (touch_goal_) {
          for (int i = total_p_params - 3; i < total_p_params; i++) {
            problem.SetParameterLowerBound(p_params, i, p_params[i] - 0.5);
            problem.SetParameterUpperBound(p_params, i, p_params[i] + 0.5);
          }
        }
        for (int i = 0; i < total_t_params; i++) {
          problem.SetParameterLowerBound(t_params, i, -10.0);
          problem.SetParameterUpperBound(t_params, i, 10.0);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.max_num_iterations = 500;
        options.function_tolerance = 1e-4;
        options.gradient_tolerance = 1e-4;
        options.parameter_tolerance = 1e-4;
        options.minimizer_progress_to_stdout = false;
        options.use_nonmonotonic_steps = false;
        options.max_consecutive_nonmonotonic_steps = 0;
        options.min_relative_decrease = 1e-3;
        options.initial_trust_region_radius = 1e-1;
        options.max_trust_region_radius = 1e3;
        options.min_trust_region_radius = 1e-12;
        options.num_threads = 1;
        options.num_linear_solver_threads = 1;
        options.use_inner_iterations = false;
        options.dense_linear_algebra_library_type = ceres::EIGEN;
        options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-3;
        options.update_state_every_iteration = true;

        t1 = ros::Time::now();

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        t2 = ros::Time::now();
        double time_ms = (t2 - t1).toSec() * 1000;
        double total_time_ms = (t2 - t0).toSec() * 1000;

        final_cost = summary.final_cost;

        std::cout << "\n========== 残差统计 ==========" << std::endl;
        std::cout << "平滑性总残差: " << total_smooth_cost_ << std::endl;
        std::cout << "速度可行性总残差: " << total_vel_cost_ << std::endl;
        std::cout << "加速度可行性总残差: " << total_acc_cost_ << std::endl;
        std::cout << "加加速度可行性总残差: " << total_jerk_cost_ << std::endl;
        std::cout << "障碍物总残差: " << total_obs_cost_ << std::endl;
        std::cout << "距离方差总残差: " << total_sqrvar_cost_ << std::endl;
        std::cout << "时间总残差: " << total_time_cost_ << std::endl;
        std::cout << "===============================" << std::endl;

        Eigen::MatrixXd optimized_inner_pts(3, inner_pts_num);
        Eigen::VectorXd optimized_times(piece_num_);

        memcpy(optimized_inner_pts.data(), p_params,
               total_p_params * sizeof(double));

        for (int i = 0; i < piece_num_; i++) {
          double virtual_t = t_params[i];
          if (virtual_t > 0) {
            optimized_times(i) = (0.5 * virtual_t + 1.0) * virtual_t + 1.0;
          } else {
            optimized_times(i) =
                1.0 / ((0.5 * virtual_t - 1.0) * virtual_t + 1.0);
          }
        }

        jerkOpt_.generate(optimized_inner_pts, optimized_times);
        cps_.points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);

        bool ceres_success = summary.IsSolutionUsable();
        visualizeDirections();
        ros::Duration(1.0).sleep();
        if (ceres_success) {
          std::cout << "=== Optimization Summary ===" << std::endl;
          std::cout << summary.BriefReport() << std::endl;

          std::vector<std::pair<int, int>> segments_nouse;
          CHK_RET check_result = finelyCheckAndSetConstraintPoints(
              segments_nouse, jerkOpt_, false);
          if (check_result == CHK_RET::OBS_FREE) {
            flag_success = true;
          } else {
            flag_still_unsafe = true;
            restart_nums++;
          }

        } else {
          flag_still_unsafe = true;
          restart_nums++;
        }

        if (!flag_success && restart_nums < 3) {
          std::cout << "碰撞障碍物:" << flag_still_unsafe << std::endl;
          std::cout << "Preparing for restart " << restart_nums + 1
                    << " of max 3..." << std::endl;
        }
      }
      while (flag_still_unsafe && restart_nums < 3)
        ;

      delete[] p_params;
      delete[] t_params;
      return flag_success;
    }

void PolyTrajOptimizerCeres::setParam(ros::NodeHandle &nh) {
  nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
  nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
  nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
  nh.param("optimization/weight_swarm", wei_swarm_, -1.0);
  // 通用可行性权重（可能被覆盖）
  nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
  // 独立可行性权重（若未设置则使用通用值）
  nh.param("optimization/weight_velocity_feasibility", wei_vel_feas_,
           wei_feas_);
  nh.param("optimization/weight_acceleration_feasibility", wei_acc_feas_,
           wei_feas_);
  nh.param("optimization/weight_jerk_feasibility", wei_jerk_feas_, wei_feas_);
  nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
  nh.param("optimization/weight_time", wei_time_, -1.0);
  nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
  nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
  nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/max_jer", max_jer_, -1.0);

  wei_swarm_mod_ = wei_swarm_;
}

void PolyTrajOptimizerCeres::setEnvironment(const GridMap::Ptr &map) {
  grid_map_ = map;
  if (a_star_ == nullptr) {
    a_star_.reset(new AStar);
  }
  a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
}

void PolyTrajOptimizerCeres::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) {
  swarm_trajs_ = swarm_trajs_ptr;
}

void PolyTrajOptimizerCeres::setDroneId(const int drone_id) {
  drone_id_ = drone_id;
}
} // namespace ego_planner