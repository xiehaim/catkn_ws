# folder_path=$(pwd)
# # 循环判断.bag文件体积是否超过50GB且数量大于6
# while true; do
#     total_size=$(du -c $folder_path/*.bag 2>/dev/null | grep total | awk '{print $1}')

#     file_count=$(ls -l $folder_path/*.bag 2>/dev/null | wc -l)

#     if [ -z "$total_size" ]; then
#         total_size=0
#     fi

#     if [ $total_size -le 52428800 ] && [ $file_count -le 6 ]; then
#         echo "文件夹中.bag文件体积小于等于5GB且文件数量不超过3个，停止删除操作"
#         break
#     fi

#     if [ $total_size -gt 52428800 ] && [ $file_count -gt 6 ]; then
#         oldest_file=$(ls -t $folder_path/*.bag | tail -1)
#         rm $oldest_file
#         echo "删除文件：$oldest_file"
#     else
#         largest_file=$(ls -S $folder_path/*.bag | head -1)
#         rm $largest_file
#         echo "删除文件：$largest_file"
#     fi
# done

rosbag record --tcpnodelay /debugL_ctrl \
/drone_0_ego_planner_node/grid_map/occupancy_inflate \
/drone_0_ego_planner_node/a_star_list \
/drone_0_ego_planner_node/failed_list \
/drone_0_ego_planner_node/global_list \
/drone_0_ego_planner_node/goal_point \
/drone_0_ego_planner_node/init_list \
/drone_0_ego_planner_node/mandatory_stop \
/drone_0_ego_planner_node/optimal_list \
/drone_0_ego_planner_node/optimal_list_path \
/drone_0_ego_planner_node/planning/heartbeat \
/drone_0_planning/data_display \
/drone_0_planning/trajectory \
/ekf_fuser/kf \
/ekf_fuser/odom \
/goal \
/ground_height_measurement \
/laserMapping/odometry \
/laserMapping/path \
/livox/imu \
/mavlink/from \
/mavlink/gcs_ip \
/mavlink/to \
/mavros/actuator_control \
/mavros/altitude \
/mavros/battery \
/mavros/estimator_status \
/mavros/extended_state \
/mavros/geofence/waypoints \
/mavros/global_position/compass_hdg \
/mavros/global_position/global \
/mavros/global_position/gp_lp_offset \
/mavros/global_position/gp_origin \
/mavros/global_position/local \
/mavros/global_position/raw/fix \
/mavros/global_position/raw/gps_vel \
/mavros/global_position/raw/satellites \
/mavros/global_position/rel_alt \
/mavros/global_position/set_gp_origin \
/mavros/hil/actuator_controls \
/mavros/hil/controls \
/mavros/hil/gps \
/mavros/hil/imu_ned \
/mavros/hil/optical_flow \
/mavros/hil/rc_inputs \
/mavros/hil/state \
/mavros/home_position/home \
/mavros/home_position/set \
/mavros/imu/data \
/mavros/imu/data_raw \
/mavros/imu/diff_pressure \
/mavros/imu/mag \
/mavros/imu/static_pressure \
/mavros/imu/temperature_baro \
/mavros/imu/temperature_imu \
/mavros/local_position/accel \
/mavros/local_position/odom \
/mavros/local_position/pose \
/mavros/local_position/pose_cov \
/mavros/local_position/velocity_body \
/mavros/local_position/velocity_body_cov \
/mavros/local_position/velocity_local \
/mavros/manual_control/control \
/mavros/manual_control/send \
/mavros/mission/reached \
/mavros/mission/waypoints \
/mavros/nav_controller_output \
/mavros/param/param_value \
/mavros/radio_status \
/mavros/rallypoint/waypoints \
/mavros/rc/in \
/mavros/rc/out \
/mavros/rc/override \
/mavros/setpoint_accel/accel \
/mavros/setpoint_attitude/cmd_vel \
/mavros/setpoint_attitude/thrust \
/mavros/setpoint_position/global \
/mavros/setpoint_position/global_to_local \
/mavros/setpoint_position/local \
/mavros/setpoint_raw/attitude \
/mavros/setpoint_raw/global \
/mavros/setpoint_raw/local \
/mavros/setpoint_raw/target_attitude \
/mavros/setpoint_raw/target_global \
/mavros/setpoint_raw/target_local \
/mavros/setpoint_trajectory/desired \
/mavros/setpoint_trajectory/local \
/mavros/setpoint_velocity/cmd_vel \
/mavros/setpoint_velocity/cmd_vel_unstamped \
/mavros/state \
/mavros/statustext/recv \
/mavros/statustext/send \
/mavros/sys_status \
/mavros/target_actuator_control \
/mavros/time_reference \
/mavros/timesync_status \
/mavros/vfr_hud \
/mavros/wind_estimation \
/nouse1 \
/odom_visualization/covariance \
/odom_visualization/covariance_velocity \
/odom_visualization/fov_visual \
/odom_visualization/height \
/odom_visualization/path \
/odom_visualization/pose \
/odom_visualization/robot \
/odom_visualization/sensor \
/odom_visualization/trajectory \
/odom_visualization/velocity \
/planning/yaw \
/l_ctrl/takeoff_land \
/rosout \
/rosout_agg \
/setpoints_cmd \
/tf \
/tf_static \
/traj_start_trigger \
/vins_estimator/extrinsic