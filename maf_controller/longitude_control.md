# pilot
## - Pos control
```
  auto &lon_pos_state = lon_pos_ctrl_.loop_state_;
  // error filter
  lon_pos_state.raw_err = ref_ptr_->lon_err_;

  // raw pos error should be less than 2.0 m
  lon_pos_state.raw_err = pnc::mathlib::Limit(lon_pos_state.raw_err, 2.0);

  lon_pos_ctrl_.loop_lowpass_filter_.Update(lon_pos_state.raw_err);

  lon_pos_ctrl_.loop_notch_filter_.Update(
      lon_pos_ctrl_.loop_lowpass_filter_.GetOutput());

  lon_pos_state.err_ = lon_pos_ctrl_.loop_notch_filter_.GetOutput();

  // feedback control
  lon_pos_ctrl_.loop_compensator_.Update(lon_pos_state.err_);

  std::vector<double> vel_tab = {0.0, 3.0, 10.0, 100.0};
  std::vector<double> gain_tab = {2.5, 2.5, 1.0, 1.0};

  lon_pos_ctrl_.pos_loop_dynamic_gain_ =
      mathlib::Interp1(vel_tab, gain_tab, measures_ptr_->vel_);

  lon_pos_state.fdbk_out_ = lon_pos_ctrl_.pos_loop_dynamic_gain_ *
                            lon_pos_ctrl_.loop_compensator_.GetOutput();

  lon_pos_state.raw_ffwd_out_ = ref_ptr_->vel_ref_;
  lon_pos_state.ffwd_out_ = lon_pos_state.raw_ffwd_out_;

  lon_pos_state.raw_out_ = lon_pos_state.fdbk_out_ + lon_pos_state.ffwd_out_;

  // slope filter
  lon_pos_ctrl_.out_slope_filter_.Update(lon_pos_state.raw_out_);
  lon_pos_state.out_ = lon_pos_ctrl_.out_slope_filter_.GetOutput();
  ```

  - 低通滤波器和限波滤波器得到位置误差
  - 前馈+反馈获得 目标速度
  - 斜坡滤波器 获得最终目标速度

## - Vel control

- 低通滤波器处理目标速度序列
```
  lon_vel_ctrl_.cmd_lowpass_filter_.Update(lon_vel_state.raw_cmd_);
  lon_vel_state.cmd_ = lon_vel_ctrl_.cmd_lowpass_filter_.GetOutput();
```
- 扰动观测器？？？
- 计算速度误差
```
lon_vel_state.raw_err = lon_vel_state.cmd_ - lon_vel_state.state_;
```
- 低通滤波器处理速度误差
```
lon_vel_ctrl_.loop_lowpass_filter_.Update(lon_vel_state.raw_err);
lon_vel_state.err_ = lon_vel_ctrl_.loop_lowpass_filter_.GetOutput();
```
- 根据速度、坡度等对Kp和Ki进行插值计算（**动态调整Kp， Ki**)
- **前馈**+反馈控制 获得目标加速度
```
  lon_vel_state.fdbk_out_ = lon_vel_ctrl_.loop_compensator_.GetOutput() *
                            lon_vel_ctrl_.vel_loop_dynamic_gain_;

  // feedforward control should be limited with 85% vel_out_acc_limit
  lon_vel_state.raw_ffwd_out_ =
      mathlib::Clamp(ref_ptr_->acc_ref_, param_ptr_->vel_out_acc_min_,
                     param_ptr_->vel_out_acc_max_);

  lon_vel_ctrl_.ffwd_compensator_.Update(lon_vel_state.raw_ffwd_out_);
  lon_vel_state.ffwd_out_ = lon_vel_ctrl_.ffwd_compensator_.GetOutput();
```
- 坡度补偿已经嵌入到扰动观测器中？？
- 根据扰动观测器的输出计算目标加速度
```
lon_vel_state.raw_out_ -= lon_vel_state.dist_;
```
- 斜坡滤波器 更新得到最终的目标加速度
```
  lon_vel_ctrl_.out_slope_filter_.Update(lon_vel_state.raw_out_);
  lon_vel_state.out_ = lon_vel_ctrl_.out_slope_filter_.GetOutput();
```
## actuator control
```
  Actuator update(const double &steer_angle_deg, const double &acceleration,
                  const double &torque, const double &velocity_mps,
                  const double &real_steer_angle_rad,
                  const uint64_t &current_timestamp_us) 
``` 
主要的转换是
```
    if (convert_ != nullptr) {
      throttle_brake =
          convert_(result_acceleration, velocity_mps, last_throttle_brake_);
    }
```
convert函数的核心是减速则输出减速度，加速则做加速度与扭矩的转化
```
    double m4 = 0;
    if (acceleration < slide_acceleration) {
      m4 = acceleration;
    } else if (acceleration >= 0) {
      m4 = compute_output_gas_interpolation(data, g_iw_, g_b0_, g_lw2_, g_b2_,
                                            g_minmax_, g_comp_);
    } else {
      auto new_data = (Eigen::Matrix<double, 2, 1>() << 0, speed).finished();
      auto thegb_acc_is0 = compute_output_gas_interpolation(
          new_data, g_iw_, g_b0_, g_lw2_, g_b2_, g_minmax_, g_comp_);
      m4 = math::interpolate(acceleration, {slide_acceleration, 0},
                             {0, thegb_acc_is0});
    }
    return m4;
```

# parking
## pos control
```
bool ControlLoop::LongitudePosControlApa() {
  // pos ctrl
  auto &lon_pos_state = lon_pos_ctrl_.loop_state_;

  // error filter
  lon_pos_state.raw_err = ref_ptr_->remain_s_;

  lon_pos_ctrl_.loop_lowpass_filter_.Update(lon_pos_state.raw_err);

  lon_pos_state.err_ = lon_pos_ctrl_.loop_lowpass_filter_.GetOutput();

  // feedback control
  lon_pos_ctrl_.loop_compensator_.Update(lon_pos_state.err_);

  lon_pos_state.fdbk_out_ = lon_pos_ctrl_.loop_compensator_.GetOutput();

  lon_pos_state.raw_ffwd_out_ = 0.0;
  lon_pos_state.ffwd_out_ = lon_pos_state.raw_ffwd_out_;

  lon_pos_state.raw_out_ = lon_pos_state.fdbk_out_ + lon_pos_state.ffwd_out_;

  // slope filter
  lon_pos_ctrl_.out_slope_filter_.Update(lon_pos_state.raw_out_);
  lon_pos_state.out_ = lon_pos_ctrl_.out_slope_filter_.GetOutput();

  return true;
}
```
- 纵向误差初值为remain_s_
- 低通滤波器处理纵向误差
- 前馈+反馈获取目标加速度
- 斜坡滤波器处理目标加速度
## vel control
- 低通滤波器处理目标速度序列
``` 
  lon_vel_ctrl_.cmd_lowpass_filter_.Update(lon_vel_state.raw_cmd_);
  lon_vel_state.cmd_ = lon_vel_ctrl_.cmd_lowpass_filter_.GetOutput();
```
- 计算安全停车状态和对应的安全停车加速度
```  
  double safe_stop_acc =
      (std::pow(lon_vel_state.cmd_, 2) - std::pow(measures_ptr_->vel_, 2)) /
      ref_ptr_->remain_s_;
  safe_stop_acc = std::min(-0.2, safe_stop_acc);
```
- 前馈和反馈计算目标加速度
  前馈分两项，分别是速度项前馈和转向导致的前馈
- 速度项前馈：
```
  lon_vel_ctrl_.ffwd_differentiator_.Update(lon_vel_state.cmd_);

  lon_vel_state.ffwd_out_ = lon_vel_ctrl_.ffwd_differentiator_.GetOutput() *
                            param_ptr_->vel_ffwd_gain_;
```
  专项导致的前馈
```
  lon_vel_state.steer_ffwd_out_ =
      std::tan(
          (std::abs(pnc::mathlib::Rad2Deg(measures_ptr_->steering_angle_)) /
           vehicle_model_ptr_->get_max_steer_degree()) *
          (3.1415926 / 4.0)) *
      param_ptr_->steer_ffwd_gain_;
```
- 反馈项：
  安全停车的反馈项为安全停车减速度
```
    safe_stop_acc =
        std::max(safe_stop_acc,
                 lon_vel_state.fdbk_out_ +
                     param_ptr_->lon_stopping_acc_rate_min_ / param_ptr_->fs_);
    lon_vel_state.fdbk_out_ = safe_stop_acc;
```
  其他时候的反馈项由pid算法得到
```
    lon_vel_state.fdbk_out_ = lon_vel_ctrl_.loop_compensator_.GetOutput();
```
- 斜坡滤波器处理目标加速度
```
  lon_vel_ctrl_.out_slope_filter_.Update(lon_vel_state.raw_out_);
  lon_vel_state.out_ = lon_vel_ctrl_.out_slope_filter_.GetOutput();
```
- 