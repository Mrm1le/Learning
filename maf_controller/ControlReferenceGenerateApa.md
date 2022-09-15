- 从规划的轨迹点中查找match point
    - ego pose中转化来的自车开机坐标
      ```
      Eigen::Vector2d cur_pos_i = transform::Vector2dFrom3d(measures_ptr_->pos_i_);
      ```
    - 规划给出的轨迹点依次求解距离，找出最近点
      ```
      for (int i = 0; i < static_cast<int>(path_points.size()) - 1; ++i) {
        ref_pos_i << path_points[i].position_enu.x, path_points[i].position_enu.y; Eigen::Vector2d dp = cur_pos_i - ref_pos_i;
        double dist = dp.norm();
        if (dist < match_point_dist_) {
          match_point_dist_ = dist;
          ref_info_ptr_->match_point_index_ = i;
        }
      }
      ```
- 从最近点开始，重新生成目标轨迹点序列，并计算车体坐标系下的dx和dy序列
  - 重新生成目标轨迹点序列
    ```
    for (int i = match_point_search_start_index;
         i < static_cast<int>(path_points.size()); ++i) {
      Eigen::Vector2d ref_pos_i = Eigen::Vector2d::Zero();
      ref_pos_i << path_points[i].position_enu.x, path_points[i].position_enu.y;
    ``` 
  - 计算目标点序列与当前位置的差值
    ```
    Eigen::Vector2d dis_pos_b = rotm2d.transpose() * dis_pos_i;
    ```
  - 计算xy两个维度下的旋转矩阵，根据欧拉角中的横摆角求旋转矩阵
    ```
    Eigen::Matrix2d rotm2d = pnc::transform::Angle2Rotm2d(measures_ptr_->euler_angle_[0]);
    ```
  - 创建到当前点的距离序列s_fit_vec，dx，dy序列
    ```
    s_fit_vec.push_back(s_fit_vec.back() + dp.norm());
    dx_vec.push_back(dis_pos_b.x());
    dy_vec.push_back(dis_pos_b.y());
    ```
- 求解三次样条曲线
  - 根据距离序列s_fit_vec，dx，dy序列拟合三次样条曲线
    ```
    dx_spline.set_points(s_fit_vec, dx_vec);
    dy_spline.set_points(s_fit_vec, dy_vec);
    ```
  - 求解样条曲线的中间点，相当于最近点
    ```
    double project_pos_x = dx_spline(s_proj);
    double project_pos_y = dy_spline(s_proj);
    project_pos << project_pos_x, project_pos_y
    ```
  - 求解样条曲线上的终点
    ```
    int s_final_index = (int)path_points.size() - PARKING_PATH_EXTEND_POINTS - ref_info_ptr_->match_point_index_;
    ```
  - 求解样条曲线上到最终点的**剩余距离**
    ```
    s_without_extend = s_fit_vec[s_final_index];

    double remain_s_plan = s_without_extend - s_proj;
    ```
- 根据uss信息更新剩余距离
- 计算MPC参考输入
  - MPC参考输入的点数(mpc预测步长 / mpc帧率 / plan点的时距)
    ```
    int N = static_cast<int>(param_ptr_->mpc_horizon_ / param_ptr_->mpc_fs_ / param_ptr_->plannig_ts_) + param_ptr_->traj_size_buffer_;
    ```
  - 将dx_vec dy_vec s_fit_vec补齐到N个点，根据最后两点的差值推算
  - MPC输入共计四个参考序列dx_ref, dy_ref, vel_ref, and dphi_ref
    - 计算tmp_vel_spline 基本上可以认为是当前车速
    - 根据s在样条线上获取新的dy和dx
      ```
      auto tmp_dy_spline = dy_spline(s);
      auto tmp_dx_spline = dx_spline(s);
      ```
    - 在样条上取相邻点差值dx_diff和dy_diff,计算dphi
      ```
      double heading = std::atan2(dy_diff, dx_diff);
      ref_ptr_->dphi_ref_vec_.push_back(heading);
      ```
    - 更新s
      ```
      s += tmp_vel_spline * 1.0 / param_ptr_->mpc_fs_;
      ```

