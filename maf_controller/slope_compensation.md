- 坡度值获取
  - ego_pose 获取四元数 
  ```
    measures_ptr_->q_ =
        Eigen::Quaterniond(ego_pose.orientation.quaternion_local.w,
                           ego_pose.orientation.quaternion_local.x,
                           ego_pose.orientation.quaternion_local.y,
                           ego_pose.orientation.quaternion_local.z);
  ```                         
  - 转化为旋转矩阵
  - 四元数与[旋转矩阵](https://blog.csdn.net/weixin_45632220/article/details/117735223?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522166313461416800180613261%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=166313461416800180613261&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-117735223-null-null.142^v47^body_digest,201^v3^control_1&utm_term=%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5&spm=1018.2226.3001.4187)的转化
  ```
   measures_ptr_->rotm_ = transform::Quat2Rotm(measures_ptr_->q_);
  ```
  - 转化为坡度值，并进行滤波处理
  ```c
    Eigen::Vector3d X_b_all = measures_ptr_->rotm_.col(0);
    Eigen::Vector3d G_vector(0.0, 0.0, -GRAVITY_ACC);
    measures_ptr_->raw_slope_acc_ = G_vector.dot(X_b_all);
    // slope lowpass filter
    slope_acc_lp_.Update(measures_ptr_->raw_slope_acc_);
    measures_ptr_->slope_acc_ = slope_acc_lp_.GetOutput();

    if (state_m_ptr_->reverse_flag_) {
      measures_ptr_->slope_acc_ *= -1.0;
    }
  ```
- 坡度补偿
  ```
  lon_vel_ctrl_.disturbance_observer_.Update( measures_ptr_->vel_, lon_vel_state.out_, measures_ptr_->slope_acc_);

  lon_vel_state.dist_ = lon_vel_ctrl_.disturbance_observer_.GetDisturbanceInput();// 这里貌似是输出/增益， 后面还要详细看一下扰动观测器

  lon_vel_state.dist_ = mathlib::Limit(lon_vel_state.dist_, 1.0);

  lon_vel_state.raw_out_ -= lon_vel_state.dist_;
  ```

- 坡度对斜坡滤波器的影响，根据坡度形成的acc对斜坡滤波器设置初值。 即斜坡产生的加速度为目标加速度的一个固定值。当上acc为1的坡时，给定的目标加速度不能与1差距太大
  ```
  lon_vel_ctrl_.start_acc_slope_filter_.SetState(-measures_ptr_->slope_acc_);

  lon_vel_ctrl_.out_slope_filter_.Update(lon_vel_state.raw_out_);

  lon_vel_state.out_ = lon_vel_ctrl_.out_slope_filter_.GetOutput();
  ```

