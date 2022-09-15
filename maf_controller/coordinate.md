## introduction
controller中主要用到了两种坐标系，惯性坐标系和车体坐标系。目测目前用到的开机坐标系就属于一种惯性坐标系，其与车辆的惯性坐标系可以通过平移获得，车辆惯性坐标系与车体坐标系通过旋转获得。
> 凡是牛顿运动定律成立的参考系，称为惯性参考系，简称惯性系。惯性坐标系是为了简化世界坐标系到物体坐标系的转化而产生的。惯性坐标系的原点与物体坐标系的原点重合，惯性坐标系的轴平行于世界坐标系的轴。引入了惯性坐标系之后，物体坐标系转换到惯性坐标系只需旋转，从惯性坐标系转换到世界坐标系只需平移。

## content
- 开机坐标系下的位置坐标（local） 
  ```
  measures_ptr_->pos_i_ << ego_pose.position.position_local.x,
  ego_pose.position.position_local.y, ego_pose.position.position_local.z;
  ```
- 车体坐标系下的角速度
  ```
    measures_ptr_->ang_vel_b_ << ego_pose.angular_velocity.angvelocity_local.vx,ego_pose.angular_velocity.angvelocity_local.vy, ego_pose.angular_velocity.angvelocity_local.vz;
  ```
  - 转化为惯性坐标系要**左乘旋转矩阵**
  ```
    measures_ptr_->ang_vel_i_ = measures_ptr_->rotm_ * measures_ptr_->ang_vel_b_;
  ```
- 惯性坐标系下的速度
  ```
  measures_ptr_->vel_i_ << ego_pose.velocity.velocity_local.vx, ego_pose.velocity.velocity_local.vy, 0.0 * ego_pose.velocity.velocity_local.vz; // z component is set to zero
  ```
  - 转化为车体坐标系需要**左乘旋转矩阵的转置**,这里能明确的看出旋转矩阵在车体坐标系和惯性坐标系之间的换算关系
    ```
    measures_ptr_->vel_b_ =
    measures_ptr_->rotm_.transpose() * measures_ptr_->vel_i_;
    ```
- wheelreport 获取轮速
- 高速下的车速用egopose，低速下用轮速和车轮半径计算
- 车体坐标系下的加速度和惯性坐标系下的加速度，同上不再赘述
  ```
  measures_ptr_->acc_b_ << GRAVITY_ACC * ego_pose.acceleration.acceleration_local.ax, GRAVITY_ACC * ego_pose.acceleration.acceleration_local.ay, 0.0 * GRAVITY_ACC * (ego_pose.acceleration.acceleration_local.az - 1.0); // z component is set to zero temporarily

  measures_ptr_->acc_i_ = measures_ptr_->rotm_ * measures_ptr_->acc_b_;
  ```
- 用二阶差分求取加加速度（jerk）
  ```
  jerk_ego_dif_.Update(measures_ptr_->acc_ego_);
  measures_ptr_->jerk_ego_ = jerk_ego_dif_.GetOutput();
  ```
- 侧偏角（beta），这里的转换关系有点迷惑，不过不影响整体思路。是以速度、角速度和旋转矩阵（车辆位姿）求取的侧偏角。*点乘和叉乘分别可以求解$cos\beta$ 和$sin\beta$*
  ```
  measures_ptr_->vel_cent_i_ =  measures_ptr_->vel_i_ + measures_ptr_->ang_vel_b_.cross(d_rear2center_i);
  measures_ptr_->vel_cent_b_ = measures_ptr_->rotm_.transpose() * measures_ptr_->vel_cent_i_;

  if (measures_ptr_->vel_cent_i_.norm() > 1.5) {
    double cos_theta = Xbody_i.dot(measures_ptr_->vel_cent_i_) /
                       measures_ptr_->vel_cent_i_.norm();
    Eigen::Vector3d cross_XV = Xbody_i.cross(measures_ptr_->vel_cent_i_);
    double sin_theta = cross_XV.norm() / measures_ptr_->vel_cent_i_.norm();
  ```
- 坡度值获取（见slope_compensation）

## appendix
- 曲率因数获取，根据速度和转向传动比、轮距等计算曲率因子.猜测曲率因子可能会影响到横向控制的阈值
  ```
  measures_ptr_->curv_factor_ = vehicle_model_ptr_->curvature_factor(measures_ptr_->vel_) * param_ptr_->curv_factor_gain_;
  ```