// collision_detection.cpp
#include "../include/optimizer/opt.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;
namespace ego_planner {

#define VERBOSE_OUTPUT false
#define PRINTF_COND(STR, ...)                                                  \
  if (VERBOSE_OUTPUT)                                                          \
  printf(STR, __VA_ARGS__)

bool PolyTrajOptimizerCeres::computePointsToCheck(poly_traj::Trajectory &traj,
                                                  int id_cps_end,
                                                  PtsChk_t &points_check) {
  points_check.clear();
  points_check.resize(id_cps_end);

  if (grid_map_ == nullptr) {
    ROS_ERROR("栅格地图未初始化！");
    return false;
  }

  const double RES = grid_map_->getResolution();
  Eigen::VectorXd durations = traj.getDurations();

  // 输出轨迹基本信息
  printf("[computePointsToCheck] 轨迹段数 = %d, 需要检查的约束点数量 = %d\n",
         (int)durations.size(), id_cps_end);
  for (int i = 0; i < durations.size(); i++) {
    printf("  段[%d] 时长 = %f\n", i, durations(i));
  }

  // 计算各段起始时间
  Eigen::VectorXd t_seg_start(durations.size() + 1);
  t_seg_start(0) = 0;
  for (int i = 0; i < durations.size(); i++) {
    t_seg_start(i + 1) = t_seg_start(i) + durations(i);
  }
  printf("[computePointsToCheck] 各段起始时间:\n");
  for (int i = 0; i <= durations.size(); i++) {
    printf("  t_seg_start[%d] = %f\n", i, t_seg_start(i));
  }

  const double DURATION = durations.sum();
  printf("[computePointsToCheck] 轨迹总时长 = %f\n", DURATION);

  // 计算采样步长
  double t_step;
  if (max_vel_ > 0) {
    t_step = min(RES / max_vel_,
                 durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5);
  } else {
    t_step = durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5;
  }
  printf("[computePointsToCheck] 采样步长 t_step = %f (地图分辨率 = %f, "
         "最大速度 = %f, "
         "最小段时长 = %f, 每段约束点数 = %d)\n",
         t_step, RES, max_vel_, durations.minCoeff(), cps_num_prePiece_);

  int id_cps_curr = 0, id_piece_curr = 0;
  double t = 0.0;
  int added_points_total = 0;

  // 沿轨迹采样
  while (true) {
    if (t > DURATION) {
      printf(
          "[computePointsToCheck] 当前时间 t = %f 超过总时长 %f，采样结束。\n",
          t, DURATION);

      // 处理未分配的约束点：将剩余约束点用轨迹终点填充
      if (id_cps_curr < id_cps_end) {
        printf("[computePointsToCheck] 填充未分配的约束点（索引 %d 到 "
               "%d），使用轨迹终点。\n",
               id_cps_curr, id_cps_end - 1);
        while (id_cps_curr < id_cps_end) {
          Eigen::Vector3d pt_end = traj.getPos(DURATION);
          points_check[id_cps_curr].emplace_back(DURATION, pt_end);
          printf("  已填充约束点 %d，时间 = %f，位置 = (%f, %f, %f)\n",
                 id_cps_curr, DURATION, pt_end.x(), pt_end.y(), pt_end.z());
          id_cps_curr++;
        }
      }

      // 清理末尾的空点
      while (points_check.size() > 0 && points_check.back().size() == 0) {
        printf("[computePointsToCheck] 移除末尾的空采样点列表。\n");
        points_check.pop_back();
      }

      if (points_check.size() <= 0) {
        ROS_ERROR("获取待检查点列表失败 (0x02)。points_check.size() = %d",
                  (int)points_check.size());
        return false;
      } else {
        printf("[computePointsToCheck] 成功收集点信息，points_check.size() = "
               "%d，总共添加的采样点个数 = %d\n",
               (int)points_check.size(), added_points_total);
        return true;
      }
    }

    // 计算下一个控制点对应的时间
    double next_t_stp = 0;
    if (cps_num_prePiece_ > 0 && durations.size() > id_piece_curr) {
      next_t_stp = t_seg_start(id_piece_curr) +
                   durations(id_piece_curr) / cps_num_prePiece_ *
                       ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
    }

    if (t >= next_t_stp) {
      // 移动到下一个控制点/段
      if (cps_num_prePiece_ > 0 &&
          id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1)) {
        ++id_piece_curr;
        printf("[computePointsToCheck] 进入下一段，当前段索引 id_piece_curr = "
               "%d\n",
               id_piece_curr);
      }
      id_cps_curr++;
      printf("[computePointsToCheck] 移动到下一个约束点，当前约束点索引 "
             "id_cps_curr = %d\n",
             id_cps_curr);
      if (id_cps_curr >= id_cps_end) {
        printf("[computePointsToCheck] 已达到需检查的最大约束点索引 "
               "%d，退出循环。\n",
               id_cps_end);
        break;
      }
    }

    Eigen::Vector3d pt = traj.getPos(t);

    // 固定时间步长采样，保证采样更均匀
    points_check[id_cps_curr].emplace_back(t, pt);
    added_points_total++;
    printf("[computePointsToCheck] 添加采样点：约束点 %d，时间 t = %f，"
           "位置 = (%f, %f, %f)，当前该约束点采样点个数 = %d\n",
           id_cps_curr, t, pt.x(), pt.y(), pt.z(),
           (int)points_check[id_cps_curr].size());

    t += t_step;
  }

  // 正常退出（id_cps_curr达到id_cps_end）
  printf("[computePointsToCheck] 循环正常结束，id_cps_curr = %d, id_cps_end = "
         "%d。\n",
         id_cps_curr, id_cps_end);
  printf("[computePointsToCheck] 总共添加的采样点个数 = %d\n",
         added_points_total);
  // 再次清理末尾可能存在的空点（理论上不会）
  while (points_check.size() > 0 && points_check.back().size() == 0) {
    printf("[computePointsToCheck] 移除末尾的空采样点列表。\n");
    points_check.pop_back();
  }
  return true;
}
// PolyTrajOptimizerCeres::CHK_RET
// PolyTrajOptimizerCeres::finelyCheckAndSetConstraintPoints(
//     std::vector<std::pair<int, int>> &segments,
//     const poly_traj::MinJerkOpt &pt_data,
//     const bool flag_first_init /*= true*/) {

//   // ---------- 入口打印 ----------
//   std::cout << "\n========== finelyCheckAndSetConstraintPoints =========="
//             << std::endl;
//   std::cout << "flag_first_init = " << flag_first_init << std::endl;
//   std::cout << "cps_num_prePiece_ = " << cps_num_prePiece_ << std::endl;

//   Eigen::MatrixXd init_points =
//       pt_data.getInitConstraintPoints(cps_num_prePiece_);
//   std::cout << "init_points.cols() = " << init_points.cols() << std::endl;

//   poly_traj::Trajectory traj = pt_data.getTraj();

//   if (flag_first_init) {
//     cps_.resize_cp(init_points.cols());
//     cps_.points = init_points;
//     std::cout << "cps_ 重新分配大小: " << cps_.cp_size << std::endl;
//   }

//   /*** 根据障碍物分割初始轨迹 ***/
//   vector<std::pair<int, int>> segment_ids;
//   constexpr int ENOUGH_INTERVAL = 2;
//   int in_id = -1, out_id = -1;
//   int same_occ_state_times = ENOUGH_INTERVAL + 1;
//   bool occ, last_occ = false;
//   bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
//   int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_);
//   PtsChk_t points_check;
//   if (!computePointsToCheck(traj, i_end, points_check)) {
//     ROS_ERROR("computePointsToCheck failed");
//     return CHK_RET::ERR;
//   }
  
//   // ---------- points_check 统计 ----------
//   std::cout << "points_check.size() = " << points_check.size() << std::endl;
//   for (int i = 0; i < std::min(5, (int)points_check.size()); ++i) {
//     std::cout << "  points_check[" << i
//               << "] 采样点数: " << points_check[i].size() << std::endl;
//   }

//   for (int i = 0; i < i_end; ++i) {
//     if (!points_check[i].empty()) {
//       double t = points_check[i][0].first; // 第一个采样点的时间
//       cps_.times[i] = t;

//       // 计算所属段索引和归一化时间
//       double cum_time = 0.0;
//       int seg = -1;
//       for (int k = 0; k < traj.getPieceNum(); ++k) {
//         double dur = traj.getDurations()(k);
//         if (t >= cum_time - 1e-9 && t <= cum_time + dur + 1e-9) {
//           seg = k;
//           cps_.normalized_t[i] = (t - cum_time) / dur;
//           break;
//         }
//         cum_time += dur;
//       }
//       if (seg == -1) { // 处理边界情况（t 等于总时间）
//         seg = traj.getPieceNum() - 1;
//         cps_.normalized_t[i] = 1.0;
//       }
//       cps_.segment_idx[i] = seg;
//     } else {
//       cps_.times[i] = 0.0;
//       cps_.segment_idx[i] = 0;
//       cps_.normalized_t[i] = 0.0;
//     }
//   }

//   // ---------- 占据检测循环 ----------
//   for (int i = 0; i < i_end; ++i) {
//     for (size_t j = 0; j < points_check[i].size(); ++j) {
//       occ = grid_map_->getInflateOccupancy(points_check[i][j].second);

//       if (occ && !last_occ) {
//         if (same_occ_state_times > ENOUGH_INTERVAL || i == 0) {
//           in_id = i;
//           flag_got_start = true;
//           std::cout << "[进入障碍物] 起始索引 in_id = " << in_id << std::endl;
//         }
//         same_occ_state_times = 0;
//         flag_got_end_maybe = false;
//       } else if (!occ && last_occ) {
//         out_id = i + 1;
//         flag_got_end_maybe = true;
//         same_occ_state_times = 0;
//         std::cout << "[离开障碍物] 结束索引 out_id = " << out_id << std::endl;
//       } else {
//         ++same_occ_state_times;
//       }

//       if (flag_got_end_maybe &&
//           (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1))) {
//         flag_got_end_maybe = false;
//         flag_got_end = true;
//       }

//       last_occ = occ;

//       if (flag_got_start && flag_got_end) {
//         std::cout << "[新段] in_id=" << in_id << ", out_id=" << out_id
//                   << std::endl;
//         flag_got_start = false;
//         flag_got_end = false;
//         if (in_id < 0 || out_id < 0) {
//           ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
//           return CHK_RET::ERR;
//         }
//         segment_ids.push_back(std::pair<int, int>(in_id, out_id));
//       }
//     }
//   }

//   /* 无碰撞，提前返回 */
//   if (segment_ids.size() == 0) {
//     std::cout << "[检测结果] 无障碍物段，直接返回 OBS_FREE" << std::endl;
//     return CHK_RET::OBS_FREE;
//   }
//   std::cout << "[检测结果] 共 " << segment_ids.size() << " 个障碍物段"
//             << std::endl;

//   /*** A* 搜索（反向） ***/
//   vector<vector<Eigen::Vector3d>> a_star_pathes;
//   for (size_t i = 0; i < segment_ids.size(); ++i) {
//     Eigen::Vector3d in(init_points.col(segment_ids[i].second));
//     Eigen::Vector3d out(init_points.col(segment_ids[i].first));
//     std::cout << "[A*] 段 " << i << " 从 " << in.transpose() << " 到 "
//               << out.transpose() << std::endl;
//     ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
//     if (ret == ASTAR_RET::SUCCESS) {
//       a_star_pathes.push_back(a_star_->getPath());
//       std::cout << "[A* 成功] 段 " << i
//                 << " 路径点数: " << a_star_pathes.back().size() << std::endl;
//     } else {
//       ROS_WARN("A-star error for segment %zu, force return!", i);
//       return CHK_RET::ERR;
//     }
//   }

//   /*** 计算边界 ***/
//   int id_low_bound, id_up_bound;
//   vector<std::pair<int, int>> bounds(segment_ids.size());
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     if (i == 0) { // 第一段
//       id_low_bound = 1;
//       if (segment_ids.size() > 1) {
//         id_up_bound =
//             (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2);
//       } else {
//         id_up_bound = init_points.cols() - 2;
//       }
//     } else if (i == segment_ids.size() - 1) { // 最后一段
//       id_low_bound =
//           (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
//                 2);
//       id_up_bound = init_points.cols() - 2;
//     } else { // 中间段
//       id_low_bound =
//           (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
//                 2);
//       id_up_bound =
//           (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) /
//                 2);
//     }
//     bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
//     std::cout << "[边界] 段 " << i << " 边界 [" << id_low_bound << ","
//               << id_up_bound << "]" << std::endl;
//   }

//   /*** 调整段长度（确保每段至少有总点数的10%） ***/
//   vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
//   constexpr double MINIMUM_PERCENT = 0.1;
//   int minimum_points = round(init_points.cols() * MINIMUM_PERCENT);
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     int num_points = segment_ids[i].second - segment_ids[i].first + 1;
//     std::cout << "[调整前] 段 " << i << " 原始 [" << segment_ids[i].first << ","
//               << segment_ids[i].second << "] 点数=" << num_points << std::endl;
//     if (num_points < minimum_points) {
//       int add_points_each_side =
//           (int)(((minimum_points - num_points) + 1.0f) / 2);
//       adjusted_segment_ids[i].first =
//           segment_ids[i].first - add_points_each_side >= bounds[i].first
//               ? segment_ids[i].first - add_points_each_side
//               : bounds[i].first;
//       adjusted_segment_ids[i].second =
//           segment_ids[i].second + add_points_each_side <= bounds[i].second
//               ? segment_ids[i].second + add_points_each_side
//               : bounds[i].second;
//     } else {
//       adjusted_segment_ids[i].first = segment_ids[i].first;
//       adjusted_segment_ids[i].second = segment_ids[i].second;
//     }
//     std::cout << "[调整后] 段 " << i << " 新 [" << adjusted_segment_ids[i].first
//               << "," << adjusted_segment_ids[i].second << "] 点数="
//               << (adjusted_segment_ids[i].second -
//                   adjusted_segment_ids[i].first + 1)
//               << std::endl;
//   }

//   // 避免重叠，并使两段恰好相邻（无间隙）
//   for (size_t i = 1; i < adjusted_segment_ids.size(); i++) {
//     if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first) {
//       double middle =
//           (adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) /
//           2.0;
//       adjusted_segment_ids[i - 1].second = static_cast<int>(floor(middle));
//       adjusted_segment_ids[i].first = static_cast<int>(ceil(middle));
//       std::cout << "[邻接调整] 段 " << i - 1 << " 右边界变为 "
//                 << adjusted_segment_ids[i - 1].second << ", 段 " << i
//                 << " 左边界变为 " << adjusted_segment_ids[i].first << std::endl;
//     }
//   }

//   vector<std::pair<int, int>> final_segment_ids;

//   /*** 为每段分配避障数据 ***/
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     std::cout << "\n--- 处理段 " << i << " ---" << std::endl;
//     // step 1: 重置标志
//     for (int j = adjusted_segment_ids[i].first;
//          j <= adjusted_segment_ids[i].second; ++j)
//       cps_.flag_temp[j] = false;

//     // step 2: 寻找交点
//     int got_intersection_id = -1;
//     Eigen::Vector3d intersection_point; // 声明在循环外部

//     for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j) {
//       Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) -
//                                    init_points.col(j - 1));
//       int Astar_id = a_star_pathes[i].size() / 2;
//       int last_Astar_id;
//       double val =
//           (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);
//       double init_val = val;

//       std::cout << "  尝试交点 控制点 j=" << j << " 初始val=" << val
//                 << std::endl;

//       while (true) {
//         last_Astar_id = Astar_id;
//         if (val >= 0) {
//           ++Astar_id;
//           if (Astar_id >= (int)a_star_pathes[i].size())
//             break;
//         } else {
//           --Astar_id;
//           if (Astar_id < 0)
//             break;
//         }
//         val =
//             (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);
//         if (val * init_val <= 0 &&
//             (fabs(val) > 1e-9 || fabs(init_val) > 1e-9)) {
//           double t = ctrl_pts_law.dot(init_points.col(j) -
//                                       a_star_pathes[i][Astar_id]) /
//                      ctrl_pts_law.dot(a_star_pathes[i][Astar_id] -
//                                       a_star_pathes[i][last_Astar_id]);
//           intersection_point =
//               a_star_pathes[i][Astar_id] +
//               (a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
//                   t;
//           got_intersection_id = j;
//           std::cout << "  找到交点 at j=" << j
//                     << ", 交点=" << intersection_point.transpose() << std::endl;
//           break; // 跳出while
//         }
//       }

//       if (got_intersection_id >= 0) {
//         // 找到了交点，跳出外层for循环
//         break;
//       }
//     }

//     // 处理找到交点的情况
//     if (got_intersection_id >= 0) {
//       double length =
//           (intersection_point - init_points.col(got_intersection_id)).norm();
//       std::cout << "  交点距离原始点长度: " << length << std::endl;
//       if (length > 1e-5) {
//         cps_.flag_temp[got_intersection_id] = true;
//         for (double a = length; a >= 0.0; a -= grid_map_->getResolution()) {
//           Eigen::Vector3d pt =
//               (a / length) * intersection_point +
//               (1 - a / length) * init_points.col(got_intersection_id);
//           bool occ = grid_map_->getInflateOccupancy(pt);
//           if (occ || a < grid_map_->getResolution()) {
//             if (occ)
//               a += grid_map_->getResolution();
//             cps_.base_point[got_intersection_id].push_back(
//                 (a / length) * intersection_point +
//                 (1 - a / length) * init_points.col(got_intersection_id));
//             cps_.direction[got_intersection_id].push_back(
//                 (intersection_point - init_points.col(got_intersection_id))
//                     .normalized());
//             std::cout << "  [添加方向] 点 " << got_intersection_id
//                       << " 方向数变为 "
//                       << cps_.direction[got_intersection_id].size()
//                       << " 方向向量: "
//                       << cps_.direction[got_intersection_id].back().transpose()
//                       << std::endl;
//             break;
//           }
//         }
//       } else {
//         got_intersection_id = -1; // 长度太小，忽略
//         std::cout << "  交点距离过小，忽略" << std::endl;
//       }
//     }

//     /* 处理极短段（两个控制点） */
//     if (segment_ids[i].second - segment_ids[i].first == 1 &&
//         got_intersection_id < 0) {
//       std::cout << "  [极短段处理] 段 " << i << " 范围 ["
//                 << segment_ids[i].first << "," << segment_ids[i].second << "]"
//                 << std::endl;
//       // 极短段且尚未找到交点
//       Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) -
//                                    init_points.col(segment_ids[i].first));
//       Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) +
//                                       init_points.col(segment_ids[i].first)) /
//                                      2;
//       int Astar_id = a_star_pathes[i].size() / 2;
//       int last_Astar_id;
//       double val =
//           (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);
//       double init_val = val;

//       std::cout << "    极短段初始val=" << val << std::endl;

//       while (true) {
//         last_Astar_id = Astar_id;
//         if (val >= 0) {
//           ++Astar_id;
//           if (Astar_id >= (int)a_star_pathes[i].size())
//             break;
//         } else {
//           --Astar_id;
//           if (Astar_id < 0)
//             break;
//         }
//         val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);
//         if (val * init_val <= 0 &&
//             (fabs(val) > 1e-9 || fabs(init_val) > 1e-9)) {
//           double t =
//               ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) /
//               ctrl_pts_law.dot(a_star_pathes[i][Astar_id] -
//                                a_star_pathes[i][last_Astar_id]);
//           Eigen::Vector3d intersection_point_short =
//               a_star_pathes[i][Astar_id] +
//               (a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
//                   t;
//           if ((intersection_point_short - middle_point).norm() > 0.01) {
//             cps_.flag_temp[segment_ids[i].first] = true;
//             cps_.base_point[segment_ids[i].first].push_back(
//                 init_points.col(segment_ids[i].first));
//             cps_.direction[segment_ids[i].first].push_back(
//                 (intersection_point_short - middle_point).normalized());
//             std::cout << "    [极短段添加方向] 点 " << segment_ids[i].first
//                       << " 方向数变为 "
//                       << cps_.direction[segment_ids[i].first].size()
//                       << " 方向向量: "
//                       << cps_.direction[segment_ids[i].first].back().transpose()
//                       << std::endl;
//             got_intersection_id = segment_ids[i].first;
//           }
//           break;
//         }
//       }
//     }

//     // step 3: 传播约束（不复制时间，每个点的时间已在之前设置）
//     if (got_intersection_id >= 0) {
//       for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second;
//            ++j)
//         if (!cps_.flag_temp[j]) {
//           cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
//           cps_.direction[j].push_back(cps_.direction[j - 1].back());
//           std::cout << "  [传播约束] 点 " << j
//                     << " 添加方向，现在大小: " << cps_.direction[j].size()
//                     << std::endl;
//         }
//       for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first;
//            --j)
//         if (!cps_.flag_temp[j]) {
//           cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
//           cps_.direction[j].push_back(cps_.direction[j + 1].back());
//           std::cout << "  [传播约束] 点 " << j
//                     << " 添加方向，现在大小: " << cps_.direction[j].size()
//                     << std::endl;
//         }
//       final_segment_ids.push_back(adjusted_segment_ids[i]);
//     } else {
//       std::cout << "  段 " << i << " 未找到交点，跳过方向添加" << std::endl;
//     }
//   }

//   last_a_star_pathes_ = a_star_pathes;
//   segments = final_segment_ids;

//   // ---------- 最终方向统计 ----------
//   std::cout << "\n[最终方向统计] 共 " << cps_.cp_size << " 个约束点"
//             << std::endl;
//   for (int i = 0; i < cps_.cp_size; ++i) {
//     if (cps_.direction[i].size() > 0) {
//       std::cout << "  点 " << i << " 方向数: " << cps_.direction[i].size()
//                 << std::endl;
//     }
//   }
//   std::cout << "========== 函数结束 ==========\n" << std::endl;

//   // 获取控制点（所有路径点）
//   Eigen::MatrixXd positions = traj.getPositions(); // 3 x (piece_num+1)
//   int num_waypoints = positions.cols();
//   int num_inner_pts = piece_num_ - 1;

//   std::cout << "\n===== 控制点（路径点）=====" << std::endl;
//   std::cout << "路径点数量: " << num_waypoints << std::endl;
//   for (int i = 0; i < num_waypoints; ++i) {
//     std::cout << "  路径点 " << i << ": " << positions.col(i).transpose()
//               << std::endl;
//   }
//   std::cout << "内部控制点（除起点终点外）:" << std::endl;
//   for (int i = 1; i < num_waypoints - 1; ++i) {
//     std::cout << "  内部点 " << i - 1 << ": " << positions.col(i).transpose()
//               << std::endl;
//   }

//   std::cout << "\n===== 约束点 (cps_.points) =====" << std::endl;
//   std::cout << "约束点数量: " << cps_.cp_size << std::endl;
//   for (int i = 0; i < cps_.cp_size; ++i) {
//     std::cout << "  约束点 " << i << ": " << cps_.points.col(i).transpose();
//     if (i < (int)cps_.times.size() && cps_.times[i] > 0) {
//       std::cout << "  时间=" << cps_.times[i]
//                 << " 段索引=" << cps_.segment_idx[i]
//                 << " 归一化时间=" << cps_.normalized_t[i];
//     }
//     std::cout << std::endl;
//   }

//   std::cout << "\n===== 基点与方向 =====" << std::endl;
//   for (int i = 0; i < cps_.cp_size; ++i) {
//     size_t n_dir = cps_.direction[i].size();
//     std::cout << "约束点 " << i << " 有 " << n_dir
//               << " 个基点-方向对:" << std::endl;
//     for (size_t j = 0; j < n_dir; ++j) {
//       std::cout << "  第 " << j
//                 << " 对: 基点=" << cps_.base_point[i][j].transpose()
//                 << " 方向=" << cps_.direction[i][j].transpose() << std::endl;
//     }
//   }

//   return CHK_RET::FINISH;
// }

// PolyTrajOptimizerCeres::CHK_RET
// PolyTrajOptimizerCeres::finelyCheckAndSetConstraintPoints(
//     std::vector<std::pair<int, int>>
//         &segments, // 输出：最终确定的障碍物段索引范围（用于多拓扑）
//     const poly_traj::MinJerkOpt
//         &pt_data, // 当前轨迹优化数据（包含系数、时间等）
//     const bool
//         flag_first_init /*= true*/) { // 是否为首次初始化（若为真，则重新分配cps_大小）

//   // 从轨迹中获取离散的约束点（采样点），每段采样 cps_num_prePiece_
//   // 个点，总点数为 N*K+1
//   Eigen::MatrixXd init_points =
//       pt_data.getInitConstraintPoints(cps_num_prePiece_);
//   poly_traj::Trajectory traj =
//       pt_data.getTraj(); // 获取轨迹对象，用于后续获取位置、段数等

//   // 如果是首次初始化，调整 cps_ 容器大小，并将采样点保存为约束点
//   if (flag_first_init) {
//     cps_.resize_cp(init_points.cols()); // 设置约束点数量
//     cps_.points = init_points;          // 保存初始采样点坐标
//   }

//   for (int i = 0; i < cps_.cp_size; i++) {
//     cps_.base_point[i].clear();
//     cps_.direction[i].clear();
//     cps_.flag_temp[i] = false;
//   }

//   /*** 1. 检测碰撞段：通过轨迹采样点的占据信息划分 ***/
//   std::vector<std::pair<int, int>>
//       segment_ids; // 存储检测到的碰撞段索引范围 [in_id, out_id]
//   constexpr int ENOUGH_INTERVAL = 2; // 用于消除噪声的连续空闲/占据次数阈值
//   int in_id = -1, out_id = -1; // 当前段的起始和结束索引（约束点索引）
//   int same_occ_state_times = ENOUGH_INTERVAL + 1; // 连续相同占据状态计数
//   bool occ, last_occ = false; // 当前占据状态和上一个状态
//   bool flag_got_start = false, flag_got_end = false,
//        flag_got_end_maybe = false; // 段起始、结束标志
//   // 只检查前2/3的约束点（靠近起点的部分），因为终点附近可能被目标占据，忽略
//   int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_);
//   PtsChk_t points_check; // 每个约束点对应的待检查点列表（时间+位置）
//   if (!computePointsToCheck(traj, i_end, points_check)) {
//     ROS_ERROR("computePointsToCheck failed");
//     return CHK_RET::ERR;
//   }

//   // 记录每个约束点的时间、段索引和归一化时间（用于后续代价计算时定位轨迹上的点）
//   for (int i = 0; i < i_end; i++) {
//     if (!points_check[i].empty()) {
//       double t = points_check[i][0].first; 
//       cps_.times[i] = t;

//       // 计算该时间属于哪个轨迹段，并得到段内归一化时间
//       double cum_time = 0.0;
//       int seg = -1;
//       for (int k = 0; k < traj.getPieceNum(); k++) {
//         double dur = traj.getDurations()(k);
//         if (t >= cum_time - 1e-9 && t <= cum_time + dur + 1e-9) {
//           seg = k;
//           cps_.normalized_t[i] = (t - cum_time) / dur;
//           break;
//         }
//         cum_time += dur;
//       }
//       if (seg == -1) { // 若时间恰好等于总时长，则归入最后一段
//         seg = traj.getPieceNum() - 1;
//         cps_.normalized_t[i] = 1.0;
//       }
//       cps_.segment_idx[i] = seg;
//     } else {
//       // 如果没有采样点（理论上不会发生），填充默认值
//       cps_.times[i] = 0.0;
//       cps_.segment_idx[i] = 0;
//       cps_.normalized_t[i] = 0.0;
//     }
//   }

//   // 遍历每个约束点的采样点，检测占据变化，划分碰撞段
//   for (int i = 0; i < i_end; i++) {
//     for (size_t j = 0; j < points_check[i].size(); j++) {
//       // 查询该采样点是否在膨胀障碍物内
//       occ = grid_map_->getInflateOccupancy(points_check[i][j].second);

//       // 检测状态变化
//       if (occ && !last_occ) { // 自由 -> 障碍
//         if (same_occ_state_times > ENOUGH_INTERVAL || i == 0) {
//           in_id = i; // 记录段起始索引
//           flag_got_start = true;
//         }
//         same_occ_state_times = 0;
//         flag_got_end_maybe = false;
//       } else if (!occ && last_occ) { // 障碍 -> 自由
//         out_id =
//             i + 1; // 记录段结束索引（注意是i+1，因为离开点对应的是下一个点）
//         flag_got_end_maybe = true;
//         same_occ_state_times = 0;
//       } else {
//         ++same_occ_state_times; // 状态连续次数增加
//       }

//       // 如果已经检测到可能离开障碍，并且连续自由次数足够或已到最后一个点，则确认段结束
//       if (flag_got_end_maybe &&
//           (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1))) {
//         flag_got_end_maybe = false;
//         flag_got_end = true;
//       }

//       last_occ = occ;

//       // 当起始和结束都确认时，记录一个碰撞段 [in_id, out_id]
//       if (flag_got_start && flag_got_end) {
//         flag_got_start = false;
//         flag_got_end = false;
//         if (in_id < 0 || out_id < 0) {
//           ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
//           return CHK_RET::ERR;
//         }
//         segment_ids.push_back(std::pair<int, int>(in_id, out_id));
//       }
//     }
//   }

//   // 若无碰撞段，提前返回
//   if (segment_ids.size() == 0) {
//     return CHK_RET::OBS_FREE;
//   }

//   /*** 2. 对每个碰撞段进行 A* 搜索（反向搜索，从段尾到段首） ***/
//   std::vector<std::vector<Eigen::Vector3d>>
//       a_star_pathes; // 每个段对应的A*路径点集
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     Eigen::Vector3d start =
//         init_points.col(segment_ids[i].second); // 段尾（出口点）
//     Eigen::Vector3d goal =
//         init_points.col(segment_ids[i].first); // 段首（入口点）
//     ASTAR_RET ret =
//         a_star_->AstarSearch(grid_map_->getResolution(), start, goal);
//     if (ret == ASTAR_RET::SUCCESS) {
//       a_star_pathes.push_back(a_star_->getPath());
//     } else {
//       ROS_WARN("A-star error for segment %zu, force return!", i);
//       return CHK_RET::ERR;
//     }
//   }

//   /*** 3. 计算每个段的边界索引，防止扩展到其他段 ***/
//   int id_low_bound, id_up_bound;
//   std::vector<std::pair<int, int>> bounds(segment_ids.size());
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     if (i ==
//         0) { // 第一段：左边界固定为1（跳过起点），右边界取本段尾和下一段首的中间
//       id_low_bound = 1;
//       if (segment_ids.size() > 1) {
//         id_up_bound =
//             (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2);
//       } else {
//         id_up_bound = init_points.cols() -
//                       2; // 只有一段，右边界到倒数第二个点（跳过终点）
//       }
//     } else if (
//         i ==
//         segment_ids.size() -
//             1) { // 最后一段：左边界取上一段尾和本段首的中间，右边界到倒数第二个点
//       id_low_bound =
//           (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
//                 2);
//       id_up_bound = init_points.cols() - 2;
//     } else { // 中间段：左边界取上一段尾和本段首的中间，右边界取本段尾和下一段首的中间
//       id_low_bound =
//           (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
//                 2);
//       id_up_bound =
//           (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) /
//                 2);
//     }
//     bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
//   }

//   /*** 4. 调整段长度，确保每段至少有总点数的10% ***/
//   std::vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
//   constexpr double MINIMUM_PERCENT = 0.1;
//   int minimum_points = round(init_points.cols() * MINIMUM_PERCENT);
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     int num_points = segment_ids[i].second - segment_ids[i].first + 1;
//     if (num_points < minimum_points) {
//       int add_points_each_side =
//           (int)(((minimum_points - num_points) + 1.0f) / 2);
//       adjusted_segment_ids[i].first =
//           segment_ids[i].first - add_points_each_side >= bounds[i].first
//               ? segment_ids[i].first - add_points_each_side
//               : bounds[i].first;
//       adjusted_segment_ids[i].second =
//           segment_ids[i].second + add_points_each_side <= bounds[i].second
//               ? segment_ids[i].second + add_points_each_side
//               : bounds[i].second;
//     } else {
//       adjusted_segment_ids[i].first = segment_ids[i].first;
//       adjusted_segment_ids[i].second = segment_ids[i].second;
//     }
//   }

//   // 避免段间重叠：如果调整后相邻段有重叠，取中间分割
//   for (size_t i = 1; i < adjusted_segment_ids.size(); i++) {
//     if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first) {
//       double middle =
//           (adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) /
//           2.0;
//       adjusted_segment_ids[i - 1].second = static_cast<int>(floor(middle));
//       adjusted_segment_ids[i].first = static_cast<int>(ceil(middle));
//     }
//   }

//   std::vector<std::pair<int, int>>
//       final_segment_ids; // 最终需要返回的段（成功设置了基点的段）

//   /*** 5. 为每个段的约束点生成基点和方向 ***/
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     // 重置本段内所有点的临时标志
//     for (int j = adjusted_segment_ids[i].first;
//          j <= adjusted_segment_ids[i].second; j++)
//       cps_.flag_temp[j] = false;

//     // 5.1 寻找一个交点，作为后续搜索方向的参考
//     int got_intersection_id = -1;       // 找到交点的约束点索引
//     Eigen::Vector3d intersection_point; // 交点的坐标

//     // 遍历段内约束点（跳过首尾点，因为需要前后点计算切线）
//     for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; j++) {
//       // 切线方向：使用相邻约束点近似（j+1 和 j-1 的差）
//       Eigen::Vector3d tangent =
//           (init_points.col(j + 1) - init_points.col(j - 1)).normalized();
//       int astar_mid = a_star_pathes[i].size() / 2; // 从A*路径中点开始搜索
//       int last_astar = astar_mid;
//       // 计算当前A*路径点相对于约束点在切线方向上的投影值
//       double val =
//           (a_star_pathes[i][astar_mid] - init_points.col(j)).dot(tangent);
//       double init_val = val;

//       while (true) {
//         last_astar = astar_mid;
//         if (val >= 0) {
//           ++astar_mid;
//           if (astar_mid >= (int)a_star_pathes[i].size())
//             break;
//         } else {
//           --astar_mid;
//           if (astar_mid < 0)
//             break;
//         }
//         val = (a_star_pathes[i][astar_mid] - init_points.col(j)).dot(tangent);
//         // 当投影值变号时，说明A*路径穿过了该约束点对应的法平面，找到交点
//         if (val * init_val <= 0 &&
//             (fabs(val) > 1e-9 || fabs(init_val) > 1e-9)) {
//           // 线性插值求精确交点（A*路径上两点之间的插值）
//           double t =
//               (tangent.dot(init_points.col(j) - a_star_pathes[i][astar_mid])) /
//               (tangent.dot(a_star_pathes[i][astar_mid] -
//                            a_star_pathes[i][last_astar]));
//           intersection_point =
//               a_star_pathes[i][astar_mid] +
//               (a_star_pathes[i][astar_mid] - a_star_pathes[i][last_astar]) * t;
//           got_intersection_id = j;
//           break;
//         }
//       }
//       if (got_intersection_id >= 0)
//         break; // 只要找到一个交点即可
//     }

//     // 5.2 如果找到交点，尝试为该交点对应的约束点生成基点和方向
//     if (got_intersection_id >= 0) {
//       double length =
//           (intersection_point - init_points.col(got_intersection_id)).norm();
//       if (length > 1e-5) {
//         // 从约束点向交点方向搜索第一个占据点作为基点
//         Eigen::Vector3d search_dir =
//             (intersection_point - init_points.col(got_intersection_id))
//                 .normalized();
//         double step = grid_map_->getResolution(); // 地图分辨率作为步长
//         double max_search = length + 5 * step; // 搜索范围稍大于交点距离
//         double d = step;
//         Eigen::Vector3d base_pt;
//         bool found = false;
//         for (; d <= max_search; d += step) {
//           Eigen::Vector3d pt =
//               init_points.col(got_intersection_id) + d * search_dir;
//           if (grid_map_->getInflateOccupancy(pt)) {
//             base_pt = pt;
//             found = true;
//             break;
//           }
//         }
//         if (found) {
//           cps_.flag_temp[got_intersection_id] = true;
//           // 方向：从基点指向约束点（即安全方向，指向轨迹内部）
//           cps_.base_point[got_intersection_id].push_back(base_pt);
//           cps_.direction[got_intersection_id].push_back(
//               (init_points.col(got_intersection_id) - base_pt).normalized());
//         } else {
//           got_intersection_id = -1; // 未找到占据点，跳过
//         }
//       } else {
//         got_intersection_id = -1;
//       }
//     }

//     // 5.3 处理极短段（两个约束点构成的段）
//     if (segment_ids[i].second - segment_ids[i].first == 1 &&
//         got_intersection_id < 0) {
//       // 类似上述过程，但使用中点代替单个约束点
//       Eigen::Vector3d tangent = (init_points.col(segment_ids[i].second) -
//                                  init_points.col(segment_ids[i].first))
//                                     .normalized();
//       Eigen::Vector3d middle = (init_points.col(segment_ids[i].second) +
//                                 init_points.col(segment_ids[i].first)) /
//                                2;
//       int astar_mid = a_star_pathes[i].size() / 2;
//       int last_astar = astar_mid;
//       double val = (a_star_pathes[i][astar_mid] - middle).dot(tangent);
//       double init_val = val;

//       while (true) {
//         last_astar = astar_mid;
//         if (val >= 0) {
//           ++astar_mid;
//           if (astar_mid >= (int)a_star_pathes[i].size())
//             break;
//         } else {
//           --astar_mid;
//           if (astar_mid < 0)
//             break;
//         }
//         val = (a_star_pathes[i][astar_mid] - middle).dot(tangent);
//         if (val * init_val <= 0 &&
//             (fabs(val) > 1e-9 || fabs(init_val) > 1e-9)) {
//           double t = (tangent.dot(middle - a_star_pathes[i][astar_mid])) /
//                      (tangent.dot(a_star_pathes[i][astar_mid] -
//                                   a_star_pathes[i][last_astar]));
//           Eigen::Vector3d intersection =
//               a_star_pathes[i][astar_mid] +
//               (a_star_pathes[i][astar_mid] - a_star_pathes[i][last_astar]) * t;
//           if ((intersection - middle).norm() > 0.01) {
//             // 从中点向交点方向搜索占据点
//             Eigen::Vector3d search_dir = (intersection - middle).normalized();
//             double step = grid_map_->getResolution();
//             double max_search = (intersection - middle).norm() + 5 * step;
//             double d = step;
//             Eigen::Vector3d base_pt;
//             bool found = false;
//             for (; d <= max_search; d += step) {
//               Eigen::Vector3d pt = middle + d * search_dir;
//               if (grid_map_->getInflateOccupancy(pt)) {
//                 base_pt = pt;
//                 found = true;
//                 break;
//               }
//             }
//             if (found) {
//               cps_.flag_temp[segment_ids[i].first] = true;
//               cps_.base_point[segment_ids[i].first].push_back(base_pt);
//               cps_.direction[segment_ids[i].first].push_back(
//                   (middle - base_pt).normalized());
//               got_intersection_id = segment_ids[i].first;
//             }
//           }
//           break;
//         }
//       }
//     }

//     // 5.4 传播：将已得到的基点和方向沿段内传播到其他约束点
//     if (got_intersection_id >= 0) {
//       if (cps_.base_point[got_intersection_id].empty()) {
//         ROS_ERROR("Base point empty at got_intersection_id=%d",
//                   got_intersection_id);
//         continue;
//       }
//       // 向前传播（索引增大方向）
//       for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second;
//            j++) {
//         if (!cps_.flag_temp[j]) {
//           if (cps_.base_point[j - 1].empty()) {
//             ROS_ERROR("Base point empty at j-1=%d", j - 1);
//             break;
//           }
//           // 复制前一个点的基点和方向
//           cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
//           cps_.direction[j].push_back(cps_.direction[j - 1].back());
//           cps_.flag_temp[j] = true; // 标记已设置
//         }
//       }
//       // 向后传播（索引减小方向）
//       for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first;
//            j--) {
//         if (!cps_.flag_temp[j]) {
//           if (cps_.base_point[j + 1].empty()) {
//             ROS_ERROR("Base point empty at j+1=%d", j + 1);
//             break;
//           }
//           cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
//           cps_.direction[j].push_back(cps_.direction[j + 1].back());
//           cps_.flag_temp[j] = true;
//         }
//       }
//       final_segment_ids.push_back(
//           adjusted_segment_ids[i]); // 记录成功设置基点的段
//     }
//   }

//   last_a_star_pathes_ =
//       a_star_pathes; // 保存最后使用的A*路径（可能用于后续可视化或调试）
//   segments = final_segment_ids; // 输出最终段索引

//   // ---------- 调试输出：控制点、约束点、基点方向 ----------
//   // 以下代码仅用于输出，帮助用户检查数据是否正确，不影响功能
//   Eigen::MatrixXd positions = traj.getPositions(); // 3 x (piece_num+1)
//   int num_waypoints = positions.cols();

//   std::cout << "\n===== 控制点（路径点）=====" << std::endl;
//   std::cout << "路径点数量: " << num_waypoints << std::endl;
//   for (int i = 0; i < num_waypoints; i++) {
//     std::cout << "  路径点 " << i << ": " << positions.col(i).transpose()
//               << std::endl;
//   }
//   std::cout << "内部控制点（除起点终点外）:" << std::endl;
//   for (int i = 1; i < num_waypoints - 1; i++) {
//     std::cout << "  内部点 " << i - 1 << ": " << positions.col(i).transpose()
//               << std::endl;
//   }

//   std::cout << "\n===== 约束点 (cps_.points) =====" << std::endl;
//   std::cout << "约束点数量: " << cps_.cp_size << std::endl;
//   for (int i = 0; i < cps_.cp_size; i++) {
//     std::cout << "  约束点 " << i << ": " << cps_.points.col(i).transpose();
//     std::cout << "cps_.times[i]"<< cps_.times[i] << std::endl;
//     if (i <= (int)cps_.times.size() &&
//         cps_.times[i] > 0)
//     {
//       std::cout << "  时间=" << cps_.times[i]
//                 << " 段索引=" << cps_.segment_idx[i]
//                 << " 归一化时间=" << cps_.normalized_t[i];
//     }
//     std::cout << std::endl;
//   }

//   std::cout << "\n===== 基点与方向 =====" << std::endl;
//   for (int i = 0; i < cps_.cp_size; i++) {
//     size_t n_dir = cps_.direction[i].size();
//     std::cout << "约束点 " << i << " 有 " << n_dir
//               << " 个基点-方向对:" << std::endl;
//     for (size_t j = 0; j < n_dir; j++) {
//       std::cout << "  第 " << j
//                 << " 对: 基点=" << cps_.base_point[i][j].transpose()
//                 << " 方向=" << cps_.direction[i][j].transpose() << std::endl;
//     }
//   }

//   return CHK_RET::FINISH;
// }

PolyTrajOptimizerCeres::CHK_RET
PolyTrajOptimizerCeres::finelyCheckAndSetConstraintPoints(
    std::vector<std::pair<int, int>> &segments,
    const poly_traj::MinJerkOpt &pt_data,
    const bool flag_first_init /*= true*/) {

  // 从轨迹中获取离散的约束点（采样点）
  Eigen::MatrixXd init_points =
      pt_data.getInitConstraintPoints(cps_num_prePiece_);
  poly_traj::Trajectory traj = pt_data.getTraj();

  // 首次初始化，调整cps_容器大小并保存初始约束点坐标
  if (flag_first_init) {
    cps_.resize_cp(init_points.cols());
    cps_.points = init_points;
  }

  // 清空旧数据
  for (int i = 0; i < cps_.cp_size; i++) {
    cps_.base_point[i].clear();
    cps_.direction[i].clear();
    cps_.flag_temp[i] = false;
  }

  /*** 1. 检测碰撞段：通过轨迹采样点的占据信息划分 ***/
  std::vector<std::pair<int, int>> segment_ids;
  constexpr int ENOUGH_INTERVAL = 2;
  int in_id = -1, out_id = -1;
  int same_occ_state_times = ENOUGH_INTERVAL + 1;
  bool occ, last_occ = false;
  bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
  int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_);
  PtsChk_t points_check;
  if (!computePointsToCheck(traj, i_end, points_check)) {
    ROS_ERROR("computePointsToCheck failed");
    return CHK_RET::ERR;
  }

  // 记录每个约束点是否在占据区域（用于确保基点/方向与碰撞点一一对应）
  std::vector<bool> cp_in_occ(i_end, false);
  for (int i = 0; i < i_end; i++) {
    for (size_t j = 0; j < points_check[i].size(); j++) {
      if (grid_map_->getInflateOccupancy(points_check[i][j].second)) {
        cp_in_occ[i] = true;
        break;
      }
    }
  }

  // 记录每个约束点的时间、段索引和归一化时间
  for (int i = 0; i < i_end; i++) {
    if (!points_check[i].empty()) {
      double t = points_check[i][0].first;
      cps_.times[i] = t;
      double cum_time = 0.0;
      int seg = -1;
      for (int k = 0; k < traj.getPieceNum(); k++) {
        double dur = traj.getDurations()(k);
        if (t >= cum_time - 1e-9 && t <= cum_time + dur + 1e-9) {
          seg = k;
          cps_.normalized_t[i] = (t - cum_time) / dur;
          break;
        }
        cum_time += dur;
      }
      if (seg == -1) {
        seg = traj.getPieceNum() - 1;
        cps_.normalized_t[i] = 1.0;
      }
      cps_.segment_idx[i] = seg;
    } else {
      cps_.times[i] = 0.0;
      cps_.segment_idx[i] = 0;
      cps_.normalized_t[i] = 0.0;
    }
  }

  // 遍历每个约束点的采样点，检测占据变化，划分碰撞段
  for (int i = 0; i < i_end; i++) {
    for (size_t j = 0; j < points_check[i].size(); j++) {
      occ = grid_map_->getInflateOccupancy(points_check[i][j].second);
      if (occ && !last_occ) {
        if (same_occ_state_times > ENOUGH_INTERVAL || i == 0) {
          in_id = i;
          flag_got_start = true;
        }
        same_occ_state_times = 0;
        flag_got_end_maybe = false;
      } else if (!occ && last_occ) {
        out_id = i + 1;
        flag_got_end_maybe = true;
        same_occ_state_times = 0;
      } else {
        ++same_occ_state_times;
      }
      if (flag_got_end_maybe &&
          (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1))) {
        flag_got_end_maybe = false;
        flag_got_end = true;
      }
      last_occ = occ;
      if (flag_got_start && flag_got_end) {
        flag_got_start = false;
        flag_got_end = false;
        if (in_id < 0 || out_id < 0) {
          ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
          return CHK_RET::ERR;
        }
        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }
  }

  if (segment_ids.size() == 0) {
    return CHK_RET::OBS_FREE;
  }

  /*** 2. 对每个碰撞段进行 A* 搜索（反向） ***/
  std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
  for (size_t i = 0; i < segment_ids.size(); i++) {
    Eigen::Vector3d start = init_points.col(segment_ids[i].second);
    Eigen::Vector3d goal = init_points.col(segment_ids[i].first);
    ASTAR_RET ret =
        a_star_->AstarSearch(grid_map_->getResolution(), start, goal);
    if (ret == ASTAR_RET::SUCCESS) {
      a_star_pathes.push_back(a_star_->getPath());
    } else {
      ROS_WARN("A-star error for segment %zu, force return!", i);
      return CHK_RET::ERR;
    }
  }

  /*** 3. 计算每个段的边界索引 ***/
  int id_low_bound, id_up_bound;
  std::vector<std::pair<int, int>> bounds(segment_ids.size());
  for (size_t i = 0; i < segment_ids.size(); i++) {
    if (i == 0) {
      id_low_bound = 1;
      if (segment_ids.size() > 1) {
        id_up_bound =
            (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2);
      } else {
        id_up_bound = init_points.cols() - 2;
      }
    } else if (i == segment_ids.size() - 1) {
      id_low_bound =
          (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
                2);
      id_up_bound = init_points.cols() - 2;
    } else {
      id_low_bound =
          (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
                2);
      id_up_bound =
          (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) /
                2);
    }
    bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
  }

  /*** 4. 段范围保持原始碰撞区间，仅做边界裁剪，避免把无碰撞点强行纳入约束 ***/
  std::vector<std::pair<int, int>> adjusted_segment_ids = segment_ids;
  for (size_t i = 0; i < adjusted_segment_ids.size(); i++) {
    adjusted_segment_ids[i].first = std::max(adjusted_segment_ids[i].first, bounds[i].first);
    adjusted_segment_ids[i].second = std::min(adjusted_segment_ids[i].second, bounds[i].second);
    if (adjusted_segment_ids[i].first > adjusted_segment_ids[i].second) {
      adjusted_segment_ids[i] = segment_ids[i];
    }
  }

  // 避免段间重叠
  for (size_t i = 1; i < adjusted_segment_ids.size(); i++) {
    if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first) {
      double middle =
          (adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) /
          2.0;
      adjusted_segment_ids[i - 1].second = static_cast<int>(floor(middle));
      adjusted_segment_ids[i].first = static_cast<int>(ceil(middle));
    }
  }

  std::vector<std::pair<int, int>> final_segment_ids; // 存储成功设置了基点的段

  /*** 5. 为每个段的**每一个**约束点独立生成基点和方向 ***/
  for (size_t i = 0; i < segment_ids.size(); i++) {
    // 重置本段内所有点的临时标志
    for (int j = adjusted_segment_ids[i].first;
         j <= adjusted_segment_ids[i].second; j++)
      cps_.flag_temp[j] = false;

    bool any_point_success = false; // 记录本段是否有至少一个点成功生成基点

    // 遍历段内所有约束点（包括首尾），为每个点独立生成
    for (int j = adjusted_segment_ids[i].first;
         j <= adjusted_segment_ids[i].second; j++) {
      if (j < 0 || j >= i_end || !cp_in_occ[j])
        continue;
      // 跳过最边缘点？为了简单，我们尝试为所有点生成，但如果点太靠近边界可能无法获得切线。
      // 如果无法生成，就跳过，不影响其他点。
      // 生成基点和方向的局部函数（lambda），输入点索引
      // j，输出是否成功及基点、方向
      auto generateForPoint = [&](int idx, Eigen::Vector3d &base_pt,
                                  Eigen::Vector3d &dir) -> bool {
        // 计算切线方向：对于内部点使用前后点，对于边界点使用单侧
        Eigen::Vector3d tangent;
        if (idx > segment_ids[i].first && idx < segment_ids[i].second) {
          tangent = (init_points.col(idx + 1) - init_points.col(idx - 1))
                        .normalized();
        } else if (idx == segment_ids[i].first) {
          // 段首点：用下一点减当前点
          if (idx + 1 <= segment_ids[i].second)
            tangent =
                (init_points.col(idx + 1) - init_points.col(idx)).normalized();
          else
            return false; // 段只有一个点，无法定义切线
        } else {          // idx == segment_ids[i].second
          // 段尾点：用当前点减上一点
          tangent =
              (init_points.col(idx) - init_points.col(idx - 1)).normalized();
        }

        // 在A*路径上寻找交点
        int astar_mid = a_star_pathes[i].size() / 2;
        int last_astar = astar_mid;
        double val =
            (a_star_pathes[i][astar_mid] - init_points.col(idx)).dot(tangent);
        double init_val = val;
        bool found_intersection = false;
        Eigen::Vector3d intersection_pt;

        while (true) {
          last_astar = astar_mid;
          if (val >= 0) {
            ++astar_mid;
            if (astar_mid >= (int)a_star_pathes[i].size())
              break;
          } else {
            --astar_mid;
            if (astar_mid < 0)
              break;
          }
          val =
              (a_star_pathes[i][astar_mid] - init_points.col(idx)).dot(tangent);
          if (val * init_val <= 0 &&
              (fabs(val) > 1e-9 || fabs(init_val) > 1e-9)) {
            double t = (tangent.dot(init_points.col(idx) -
                                    a_star_pathes[i][astar_mid])) /
                       (tangent.dot(a_star_pathes[i][astar_mid] -
                                    a_star_pathes[i][last_astar]));
            intersection_pt =
                a_star_pathes[i][astar_mid] +
                (a_star_pathes[i][astar_mid] - a_star_pathes[i][last_astar]) *
                    t;
            found_intersection = true;
            break;
          }
        }

        if (!found_intersection) {
          // 退化情况：取A*路径上离约束点最近的点作为交会参考，避免该点直接丢失约束
          double best_d2 = std::numeric_limits<double>::infinity();
          int best_id = -1;
          for (int pi = 0; pi < (int)a_star_pathes[i].size(); ++pi) {
            double d2 = (a_star_pathes[i][pi] - init_points.col(idx)).squaredNorm();
            if (d2 < best_d2) {
              best_d2 = d2;
              best_id = pi;
            }
          }
          if (best_id >= 0) {
            intersection_pt = a_star_pathes[i][best_id];
            found_intersection = true;
          }
        }

        if (!found_intersection)
          return false;

        double length = (intersection_pt - init_points.col(idx)).norm();
        if (length <= 1e-5)
          return false;

        // 沿约束点->交点方向搜索占据状态跃迁，用跃迁附近点作为“障碍物表面”基点
        Eigen::Vector3d search_dir =
            (intersection_pt - init_points.col(idx)).normalized();
        double step = grid_map_->getResolution();
        double max_search = length + 5 * step;
        bool found_base = false;
        Eigen::Vector3d base_candidate = init_points.col(idx);

        bool prev_occ = grid_map_->getInflateOccupancy(init_points.col(idx));
        Eigen::Vector3d prev_pt = init_points.col(idx);
        for (double d = step; d <= max_search; d += step) {
          Eigen::Vector3d pt = init_points.col(idx) + d * search_dir;
          bool occ = grid_map_->getInflateOccupancy(pt);

          if (occ != prev_occ) {
            // free->occ 时通常取 prev_pt（贴近障碍表面且在自由侧），
            // 但若跃迁发生在首个采样步，prev_pt 仍是约束点本身，
            // 会导致后续方向向量归一化出现零向量；此时改取 pt。
            // occ->free 时取 pt（刚离开占据区，贴近障碍表面）。
            if (!prev_occ && (prev_pt - init_points.col(idx)).squaredNorm() < 1e-12) {
              base_candidate = pt;
            } else {
              base_candidate = prev_occ ? pt : prev_pt;
            }
            found_base = true;
            break;
          }

          prev_occ = occ;
          prev_pt = pt;
        }

        // 兜底：若没有发生状态跃迁，但沿线曾有自由点（典型是起点在占据区），取首个自由点
        if (!found_base && prev_occ) {
          for (double d = step; d <= max_search; d += step) {
            Eigen::Vector3d pt = init_points.col(idx) + d * search_dir;
            if (!grid_map_->getInflateOccupancy(pt)) {
              base_candidate = pt;
              found_base = true;
              break;
            }
          }
        }

        if (!found_base)
          return false;

        Eigen::Vector3d diff = init_points.col(idx) - base_candidate;
        if (diff.squaredNorm() < 1e-12)
          return false;

        base_pt = base_candidate;
        dir = diff.normalized(); // 从基点指向约束点
        return true;
      };

      Eigen::Vector3d base, dir;
      if (generateForPoint(j, base, dir)) {
        cps_.flag_temp[j] = true;
        cps_.base_point[j].push_back(base);
        cps_.direction[j].push_back(dir);
        any_point_success = true;
      }
    }

    // 如果本段至少有一个点成功生成了基点，则记录该段
    if (any_point_success) {
      final_segment_ids.push_back(adjusted_segment_ids[i]);
    }
  }

  last_a_star_pathes_ = a_star_pathes;
  segments = final_segment_ids;

  // 以下为调试输出，可保留或删除
  // ... (原有输出代码)

  return CHK_RET::FINISH;
}
// PolyTrajOptimizerCeres::finelyCheckAndSetConstraintPoints(
//     std::vector<std::pair<int, int>> &segments,
//     const poly_traj::MinJerkOpt &pt_data, const bool flag_first_init) {

//   if (grid_map_ == nullptr) {
//     ROS_ERROR("Grid map not initialized!");
//     return CHK_RET::ERR;
//   }

//   Eigen::MatrixXd init_points =
//       pt_data.getInitConstraintPoints(cps_num_prePiece_);
//   poly_traj::Trajectory traj = pt_data.getTraj();

//   // 第一次初始化约束点
//   if (flag_first_init) {
//     cps_.resize_cp(init_points.cols());
//     cps_.points = init_points;
//   }

//   /*** 根据障碍物分割初始轨迹 ***/
//   vector<std::pair<int, int>> segment_ids;
//   constexpr int ENOUGH_INTERVAL = 2; // 足够间隔点
//   int in_id = -1, out_id = -1;
//   int same_occ_state_times = ENOUGH_INTERVAL + 1;
//   bool occ, last_occ = false;
//   bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe =
//   false;

//   int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_);

//   PtsChk_t points_check;
//   if (!computePointsToCheck(traj, i_end, points_check)) {
//     return CHK_RET::ERR;
//   }

//   // 检测碰撞段 [in_id, out_id]
//   for (int i = 0; i < i_end; ++i) {
//     for (size_t j = 0; j < points_check[i].size(); ++j) {
//       occ = grid_map_->getInflateOccupancy(points_check[i][j].second);

//       // 状态转移: 自由 -> 碰撞
//       if (occ && !last_occ) {
//         if (same_occ_state_times > ENOUGH_INTERVAL || i == 0) {
//           in_id = i;
//           flag_got_start = true;
//         }
//         same_occ_state_times = 0;
//         flag_got_end_maybe = false;
//       }
//       // 状态转移: 碰撞 -> 自由
//       else if (!occ && last_occ) {
//         out_id = i + 1;
//         flag_got_end_maybe = true;
//         same_occ_state_times = 0;
//       } else {
//         ++same_occ_state_times;
//       }

//       // 确认段结束
//       if (flag_got_end_maybe &&
//           (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1))) {
//         flag_got_end_maybe = false;
//         flag_got_end = true;
//       }

//       last_occ = occ;

//       // 找到完整碰撞段 [in_id, out_id]
//       if (flag_got_start && flag_got_end) {
//         flag_got_start = false;
//         flag_got_end = false;
//         if (in_id < 0 || out_id < 0) {
//           ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
//           return CHK_RET::ERR;
//         }
//         segment_ids.push_back(std::pair<int, int>(in_id, out_id));
//       }
//     }
//   }

//   /* 无碰撞，提前返回 */
//   if (segment_ids.size() == 0) {
//     return CHK_RET::OBS_FREE;
//   }

//   /*** A*搜索避障路径 ***/
//   vector<vector<Eigen::Vector3d>> a_star_pathes;
//   for (size_t i = 0; i < segment_ids.size(); ++i) {
//     // 从段尾到段头搜索（反向搜索）
//     Eigen::Vector3d in(init_points.col(segment_ids[i].second)),
//         out(init_points.col(segment_ids[i].first));
//     ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in,
//     out); if (ret == ASTAR_RET::SUCCESS) {
//       a_star_pathes.push_back(a_star_->getPath());
//     } else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) {
//       segment_ids[i].second = segment_ids[i + 1].second;
//       segment_ids.erase(segment_ids.begin() + i + 1);
//       --i;
//       ROS_WARN("A corner case 2, I have never exeam it.");
//     } else {
//       ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");
//       return CHK_RET::ERR;
//     }
//   }

//   int id_low_bound, id_up_bound;
//   vector<std::pair<int, int>> bounds(segment_ids.size());
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     // 边界计算确保有足够控制点处理避障
//     if (i == 0) // 第一段
//     {
//       id_low_bound = 1;
//       if (segment_ids.size() > 1) {
//         id_up_bound =
//             (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) /
//             2);
//       } else {
//         id_up_bound = init_points.cols() - 2;
//       }
//     } else if (i == segment_ids.size() - 1) // 最后一段
//     {
//       id_low_bound =
//           (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
//                 2);
//       id_up_bound = init_points.cols() - 2;
//     } else // 中间段
//     {
//       id_low_bound =
//           (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) /
//                 2);
//       id_up_bound =
//           (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) /
//                 2);
//     }

//     bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
//   }

//   /*** 调整段长度确保有足够点 ***/
//   vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
//   constexpr double MINIMUM_PERCENT = 0.0; // 每段最少点数百分比
//   int minimum_points = round(init_points.cols() * MINIMUM_PERCENT),
//   num_points; for (size_t i = 0; i < segment_ids.size(); i++) {
//     num_points = segment_ids[i].second - segment_ids[i].first + 1;
//     if (num_points < minimum_points) {
//       // 向两端扩展段
//       double add_points_each_side =
//           (int)(((minimum_points - num_points) + 1.0f) / 2);

//       adjusted_segment_ids[i].first =
//           segment_ids[i].first - add_points_each_side >= bounds[i].first
//               ? segment_ids[i].first - add_points_each_side
//               : bounds[i].first;

//       adjusted_segment_ids[i].second =
//           segment_ids[i].second + add_points_each_side <= bounds[i].second
//               ? segment_ids[i].second + add_points_each_side
//               : bounds[i].second;
//     } else {
//       adjusted_segment_ids[i].first = segment_ids[i].first;
//       adjusted_segment_ids[i].second = segment_ids[i].second;
//     }
//   }

//   // 避免段重叠
//   for (size_t i = 1; i < adjusted_segment_ids.size(); i++) {
//     if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
//     {
//       double middle = (double)(adjusted_segment_ids[i - 1].second +
//                                adjusted_segment_ids[i].first) /
//                       2.0;
//       adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
//       adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
//     }
//   }

//   // 最终段ID（用于返回）
//   vector<std::pair<int, int>> final_segment_ids;

//   /*** 为每段分配避障数据 ***/
//   for (size_t i = 0; i < segment_ids.size(); i++) {
//     // 步骤1: 重置标志
//     for (int j = adjusted_segment_ids[i].first;
//          j <= adjusted_segment_ids[i].second; ++j)
//       cps_.flag_temp[j] = false;

//     // 步骤2: 计算交点
//     int got_intersection_id = -1;
//     for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j) {
//       // 控制点切线方向
//       Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) -
//                                    init_points.col(j - 1)),
//           intersection_point;
//       int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
//       double val = (a_star_pathes[i][Astar_id] - init_points.col(j))
//                        .dot(ctrl_pts_law),
//              init_val = val;

//       // 在A*路径上寻找交点
//       while (true) {
//         last_Astar_id = Astar_id;

//         if (val >= 0) {
//           ++Astar_id;
//           if (Astar_id >= (int)a_star_pathes[i].size()) {
//             break;
//           }
//         } else {
//           --Astar_id;
//           if (Astar_id < 0) {
//             break;
//           }
//         }

//         val =
//             (a_star_pathes[i][Astar_id] -
//             init_points.col(j)).dot(ctrl_pts_law);

//         // 找到符号变化的交点
//         if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) {
//           // 线性插值求精确交点
//           intersection_point =
//               a_star_pathes[i][Astar_id] +
//               ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])
//               *
//                (ctrl_pts_law.dot(init_points.col(j) -
//                                  a_star_pathes[i][Astar_id]) /
//                 ctrl_pts_law.dot(a_star_pathes[i][Astar_id] -
//                                  a_star_pathes[i][last_Astar_id])));
//           got_intersection_id = j;
//           break;
//         }
//       }

//       if (got_intersection_id >= 0) {
//         double length = (intersection_point - init_points.col(j)).norm();
//         if (length > 1e-5) {
//           cps_.flag_temp[j] = true;
//           // 从交点向控制点方向搜索安全点
//           for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
//           {
//             bool occ = grid_map_->getInflateOccupancy(
//                 (a / length) * intersection_point +
//                 (1 - a / length) * init_points.col(j));

//             if (occ || a < grid_map_->getResolution()) {
//               if (occ)
//                 a += grid_map_->getResolution();
//               // 设置基点和方向向量
//               cps_.base_point[j].push_back((a / length) * intersection_point
//               +
//                                            (1 - a / length) *
//                                                init_points.col(j));
//               cps_.direction[j].push_back(
//                   (intersection_point - init_points.col(j)).normalized());
//               break;
//             }
//           }
//         } else {
//           got_intersection_id = -1;
//         }
//       }
//     }

//     /* 特殊情况处理: 段长度太短 */
//     if (segment_ids[i].second - segment_ids[i].first == 1) {
//       Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) -
//                                    init_points.col(segment_ids[i].first)),
//           intersection_point;
//       Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second)
//       +
//                                       init_points.col(segment_ids[i].first))
//                                       /
//                                      2;
//       int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
//       double val =
//                  (a_star_pathes[i][Astar_id] -
//                  middle_point).dot(ctrl_pts_law),
//              init_val = val;

//       while (true) {
//         last_Astar_id = Astar_id;

//         if (val >= 0) {
//           ++Astar_id;
//           if (Astar_id >= (int)a_star_pathes[i].size()) {
//             break;
//           }
//         } else {
//           --Astar_id;
//           if (Astar_id < 0) {
//             break;
//           }
//         }

//         val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

//         if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) {
//           intersection_point =
//               a_star_pathes[i][Astar_id] +
//               ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])
//               *
//                (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) /
//                 ctrl_pts_law.dot(a_star_pathes[i][Astar_id] -
//                                  a_star_pathes[i][last_Astar_id])));

//           if ((intersection_point - middle_point).norm() > 0.01) {
//             cps_.flag_temp[segment_ids[i].first] = true;
//             cps_.base_point[segment_ids[i].first].push_back(
//                 init_points.col(segment_ids[i].first));
//             cps_.direction[segment_ids[i].first].push_back(
//                 (intersection_point - middle_point).normalized());
//             got_intersection_id = segment_ids[i].first;
//           }
//           break;
//         }
//       }
//     }

//     // 步骤3: 传播避障约束到相邻控制点
//     if (got_intersection_id >= 0) {
//       // 向前传播
//       for (int j = got_intersection_id + 1; j <=
//       adjusted_segment_ids[i].second;
//            ++j)
//         if (!cps_.flag_temp[j]) {
//           cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
//           cps_.direction[j].push_back(cps_.direction[j - 1].back());
//         }

//       // 向后传播
//       for (int j = got_intersection_id - 1; j >=
//       adjusted_segment_ids[i].first;
//            --j)
//         if (!cps_.flag_temp[j]) {
//           cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
//           cps_.direction[j].push_back(cps_.direction[j + 1].back());
//         }

//       final_segment_ids.push_back(adjusted_segment_ids[i]);
//     } else {
//       // 忽略此段
//     }
//   }

//   segments = final_segment_ids;
//   return CHK_RET::FINISH;
// }

/*
 * 粗略檢查約束點 - 用於反彈優化
 * 快速檢測新出現的障礙物並更新避障約束
 */
bool PolyTrajOptimizerCeres::roughlyCheckConstraintPoints(void) {
  if (grid_map_ == nullptr) {
    ROS_ERROR("Grid map not initialized!");
    return false;
  }

  /*** 檢查並根據新障礙物分割軌跡 ***/
  int in_id, out_id;
  std::vector<std::pair<int, int>> segment_ids;
  bool flag_new_obs_valid = false;
  int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_);

  for (int i = 1; i <= i_end; i++) {
    bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

    /*** 檢查新碰撞是否有效（不在現有避障約束內） ***/
    if (occ) {
      for (size_t k = 0; k < cps_.direction[i].size(); k++) {
        // 檢查控制點是否在所有避障約束外
        if ((cps_.points.col(i) - cps_.base_point[i][k])
                .dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) {
          occ = false;
          break;
        }
      }
    }

    if (occ) {
      flag_new_obs_valid = true;

      // 尋找段起始（自由點）
      int j;
      for (j = i - 1; j >= 0; j--) {
        occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
        if (!occ) {
          in_id = j;
          break;
        }
      }
      if (j < 0) {
        ROS_ERROR("The drone is in obstacle. It means a crash in "
                  "real-world.");
        in_id = 0;
      }

      // 尋找段結束（自由點）
      for (j = i + 1; j < cps_.cp_size; j++) {
        occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
        if (!occ) {
          out_id = j;
          break;
        }
      }
      if (j >= cps_.cp_size) {
        ROS_WARN("Local target in collision, skip this planning.");
        force_stop_type_ = STOP_FOR_ERROR;
        return false;
      }

      i = j + 1;
      segment_ids.push_back(std::pair<int, int>(in_id, out_id));
    }
  }

  if (flag_new_obs_valid) {
    // A*搜索和約束設置（類似精細檢查但簡化）
    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); i++) {
      Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)),
          out(cps_.points.col(segment_ids[i].first));

      // 簡化處理：直接使用直線路徑
      std::vector<Eigen::Vector3d> path;
      path.push_back(in);

      // 簡單的直線插值
      int steps = 10;
      for (int k = 1; k < steps; k++) {
        double ratio = (double)k / steps;
        Eigen::Vector3d point = in * (1 - ratio) + out * ratio;
        path.push_back(point);
      }

      path.push_back(out);
      a_star_pathes.push_back(path);
    }

    // 避免重疊和設置約束
    for (size_t i = 1; i < segment_ids.size(); i++) {
      if (segment_ids[i - 1].second >= segment_ids[i].first) {
        double middle =
            (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
        segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    /*** 為每段分配參數 ***/
    for (size_t i = 0; i < segment_ids.size(); i++) {
      for (int j = segment_ids[i].first; j <= segment_ids[i].second; j++)
        cps_.flag_temp[j] = false;

      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; j++) {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) -
                                     cps_.points.col(j - 1)),
            intersection_point;

        if (a_star_pathes[i].size() == 0) {
          continue;
        }

        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
        double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j))
                         .dot(ctrl_pts_law),
               init_val = val;

        while (true) {
          last_Astar_id = Astar_id;

          if (val >= 0) {
            ++Astar_id;
            if (Astar_id >= (int)a_star_pathes[i].size()) {
              break;
            }
          } else {
            --Astar_id;
            if (Astar_id < 0) {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - cps_.points.col(j))
                    .dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] -
                  a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) -
                                   a_star_pathes[i][Astar_id]) /
                  ctrl_pts_law.dot(a_star_pathes[i][Astar_id] -
                                   a_star_pathes[i][last_Astar_id])));
            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0) {
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5) {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution()) {
              bool occ = grid_map_->getInflateOccupancy(
                  (a / length) * intersection_point +
                  (1 - a / length) * cps_.points.col(j));

              if (occ || a < grid_map_->getResolution()) {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point +
                                             (1 - a / length) *
                                                 cps_.points.col(j));
                cps_.direction[j].push_back(
                    (intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          } else {
            got_intersection_id = -1;
          }
        }
      }

      // 傳播約束
      if (got_intersection_id >= 0) {
        for (int j = got_intersection_id + 1; j <= segment_ids[i].second; j++)
          if (!cps_.flag_temp[j]) {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= segment_ids[i].first; j--)
          if (!cps_.flag_temp[j]) {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
    }

    force_stop_type_ = STOP_FOR_REBOUND;
    return true;
  }

  return false;
}

/*
 * 判斷是否允許反彈優化
 */
bool PolyTrajOptimizerCeres::allowRebound(void) {
  // 準則1: 迭代次數
  if (iter_num_ < 3)
    return false;

  // 準則2: 軌跡曲率檢查
  double min_product = 1;
  for (int i = 3; i <= cps_.points.cols() - 4; i++) // 忽略首尾
  {
    // 計算相鄰段方向夾角的餘弦值
    double product =
        ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized())
            .dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
    if (product < min_product) {
      min_product = product;
    }
  }
  // 如果夾角太小（軌跡太直），不允許反彈
  if (min_product < 0.87) // cos(30°) ≈ 0.866
    return false;

  // 準則3: 多拓撲軌跡的初始避障檢查
  if (multitopology_data_.use_multitopology_trajs) {
    if (!multitopology_data_.initial_obstacles_avoided) {
      bool avoided = true;
      // 檢查所有控制點是否在避障約束的正確方向
      for (int i = 1; i < cps_.points.cols() - 1; i++) {
        if (cps_.base_point[i].size() > 0) {
          // 檢查: (p_i - b_i) · d_i > 0 表示在安全方向
          if ((cps_.points.col(i) - cps_.base_point[i][0])
                  .dot(cps_.direction[i][0]) < 0) {
            avoided = false;
            break;
          }
        }
      }

      multitopology_data_.initial_obstacles_avoided = avoided;
    }

    if (!multitopology_data_.initial_obstacles_avoided) {
      return false;
    }
  }

  // 所有準則通過
  return true;
}

/*
 * 多拓撲軌跡生成
 */
std::vector<ConstraintPoints> PolyTrajOptimizerCeres::distinctiveTrajs(
    std::vector<std::pair<int, int>> segments) {
  // 無碰撞段，返回原始軌跡
  if (segments.size() == 0) {
    std::vector<ConstraintPoints> oneSeg;
    oneSeg.push_back(cps_);
    return oneSeg;
  }

  constexpr int MAX_TRAJS = 8; // 最大軌跡數
  constexpr int VARIS = 2;     // 每段2種避障方向
  int seg_upbound =
      std::min((int)segments.size(),
               static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
  std::vector<ConstraintPoints> control_pts_buf;
  control_pts_buf.reserve(MAX_TRAJS);

  if (grid_map_ == nullptr) {
    ROS_ERROR("Grid map not initialized!");
    return control_pts_buf;
  }

  const double RESOLUTION = grid_map_->getResolution();
  const double CTRL_PT_DIST =
      (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() /
      (cps_.cp_size - 1);

  // 步驟1: 為每段找到相反的向量和基點
  std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;
  for (int i = 0; i < seg_upbound; i++) {
    std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
    ConstraintPoints RichInfoOneSeg_temp;
    cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
    RichInfoOneSeg.first = RichInfoOneSeg_temp;  // 原始方向
    RichInfoOneSeg.second = RichInfoOneSeg_temp; // 相反方向
    RichInfoSegs.push_back(RichInfoOneSeg);
  }

  // 為每段計算相反方向的避障約束
  for (int i = 0; i < seg_upbound; i++) {
    if (RichInfoSegs[i].first.cp_size > 1) {
      // 1.1 找到段的起始和結束碰撞點
      int occ_start_id = -1, occ_end_id = -1;
      Eigen::Vector3d occ_start_pt, occ_end_pt;

      // 向前搜索找到第一個碰撞點
      for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++) {
        double step_size = RESOLUTION /
                           (RichInfoSegs[i].first.points.col(j) -
                            RichInfoSegs[i].first.points.col(j + 1))
                               .norm() /
                           2;
        for (double a = 1; a > 0; a -= step_size) {
          Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) +
                             (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
          if (grid_map_->getInflateOccupancy(pt)) {
            occ_start_id = j;
            occ_start_pt = pt;
            goto exit_multi_loop1;
          }
        }
      }
    exit_multi_loop1:;

      // 向後搜索找到最後一個碰撞點
      for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--) {
        double step_size =
            RESOLUTION / (RichInfoSegs[i].first.points.col(j) -
                          RichInfoSegs[i].first.points.col(j - 1))
                             .norm();
        for (double a = 1; a > 0; a -= step_size) {
          Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) +
                             (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
          if (grid_map_->getInflateOccupancy(pt)) {
            occ_end_id = j;
            occ_end_pt = pt;
            goto exit_multi_loop2;
          }
        }
      }
    exit_multi_loop2:;

      // 雙重檢查
      if (occ_start_id == -1 || occ_end_id == -1) {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
        continue;
      }

      // 1.2 反轉向量並找到新的基點
      for (int j = occ_start_id; j <= occ_end_id; j++) {
        Eigen::Vector3d base_pt_reverse, base_vec_reverse;
        if (RichInfoSegs[i].first.base_point[j].size() != 1) {
          ROS_ERROR("Wrong number of base_points!!! Should not be "
                    "happen!.");
          std::vector<ConstraintPoints> blank;
          return blank;
        }

        base_vec_reverse = -RichInfoSegs[i].first.direction[j][0]; // 反轉方向

        // 起始和結束點特殊處理
        if (j == occ_start_id) {
          base_pt_reverse = occ_start_pt;
        } else if (j == occ_end_id) {
          base_pt_reverse = occ_end_pt;
        } else {
          // 保持相同距離但相反方向
          base_pt_reverse =
              RichInfoSegs[i].first.points.col(j) +
              base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] -
                                  RichInfoSegs[i].first.points.col(j))
                                     .norm();
        }

        // 如果新基點在障礙物內，向外搜索
        if (grid_map_->getInflateOccupancy(base_pt_reverse)) {
          double l_upbound = 5 * CTRL_PT_DIST;
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION) {
            Eigen::Vector3d base_pt_temp =
                base_pt_reverse + l * base_vec_reverse;
            if (!grid_map_->getInflateOccupancy(base_pt_temp)) {
              RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound) {
            // 無法找到合適基點，刪除此段
            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
            goto exit_multi_loop3;
          }
        } else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j))
                       .norm() >= RESOLUTION) {
          // 直接使用反轉的基點
          RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
        } else {
          // 基點和控制點太近，刪除此段
          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
          goto exit_multi_loop3;
        }
      }

      // 1.3 將基點傳播到段的其餘部分
      if (RichInfoSegs[i].second.cp_size) {
        for (int j = occ_start_id - 1; j >= 0; j--) {
          RichInfoSegs[i].second.base_point[j][0] =
              RichInfoSegs[i].second.base_point[occ_start_id][0];
          RichInfoSegs[i].second.direction[j][0] =
              RichInfoSegs[i].second.direction[occ_start_id][0];
        }
        for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++) {
          RichInfoSegs[i].second.base_point[j][0] =
              RichInfoSegs[i].second.base_point[occ_end_id][0];
          RichInfoSegs[i].second.direction[j][0] =
              RichInfoSegs[i].second.direction[occ_end_id][0];
        }
      }

    exit_multi_loop3:;
    } else {
      // 單點段的處理
      Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
      Eigen::Vector3d base_pt_reverse =
          RichInfoSegs[i].first.points.col(0) +
          base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] -
                              RichInfoSegs[i].first.points.col(0))
                                 .norm();

      if (grid_map_->getInflateOccupancy(base_pt_reverse)) {
        double l_upbound = 5 * CTRL_PT_DIST;
        double l = RESOLUTION;
        for (; l <= l_upbound; l += RESOLUTION) {
          Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
          if (!grid_map_->getInflateOccupancy(base_pt_temp)) {
            RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
            RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
            break;
          }
        }
        if (l > l_upbound) {
          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      } else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0))
                     .norm() >= RESOLUTION) {
        RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
        RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
      } else {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
      }
    }
  }

  // 步驟2: 組合每段生成新的控制點序列
  if (seg_upbound == 0) {
    std::vector<ConstraintPoints> oneSeg;
    oneSeg.push_back(cps_);
    return oneSeg;
  }

  // 2.1 生成選擇表（二進制組合）
  std::vector<int> selection(seg_upbound);
  std::fill(selection.begin(), selection.end(), 0);
  selection[0] = -1;
  int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));

  for (int i = 0; i < max_traj_nums; i++) {
    // 更新選擇表（類似二進制計數器）
    int digit_id = 0;
    selection[digit_id]++;
    while (digit_id < seg_upbound && selection[digit_id] >= VARIS) {
      selection[digit_id] = 0;
      digit_id++;
      if (digit_id >= seg_upbound) {
        ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id,
                  seg_upbound);
      }
      selection[digit_id]++;
    }

    ConstraintPoints cpsOneSample;
    cpsOneSample.resize_cp(cps_.cp_size);
    int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;

    while (cp_id < cps_.cp_size) {
      if (seg_id >= seg_upbound || cp_id < segments[seg_id].first ||
          cp_id > segments[seg_id].second) {
        // 非碰撞段使用原始參數
        cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
        cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
        cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
      } else if (cp_id >= segments[seg_id].first &&
                 cp_id <= segments[seg_id].second) {
        if (!selection[seg_id]) // 選擇原始方向
        {
          cpsOneSample.points.col(cp_id) =
              RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
          cpsOneSample.base_point[cp_id] =
              RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
          cpsOneSample.direction[cp_id] =
              RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
          cp_of_seg_id++;
        } else // 選擇相反方向
        {
          if (RichInfoSegs[seg_id].second.cp_size) {
            cpsOneSample.points.col(cp_id) =
                RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] =
                RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] =
                RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
            cp_of_seg_id++;
          } else {
            // 放棄此軌跡
            goto abandon_this_trajectory;
          }
        }

        if (cp_id == segments[seg_id].second) {
          cp_of_seg_id = 0;
          seg_id++;
        }
      } else {
        ROS_ERROR("Should not happen!!!!");
      }

      cp_id++;
    }

    control_pts_buf.push_back(cpsOneSample);

  abandon_this_trajectory:;
  }

  return control_pts_buf;
}

} // namespace ego_planner
