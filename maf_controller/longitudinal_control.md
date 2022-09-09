函数解析
```
  LongitudeActuator
  update(const Planning &plan, const ChassisReport &chassis_report,
         const WheelReport &wheel_report, const MLALocalization &ego_pose,
         const double &velocity_mps, const uint64_t &current_timestamp_us,
         LonDynamicState &lon_state, UssOocCalculator::UssDataType 
         &uss_data, std::vector<math::Vec2d> &obstacle_from_triangulation,
         mjson::Json::object &extra_json_object)
```
- controller通过接收planning中的gear command信号来判断当前是否是在倒车
- 速度序列or目标速度 判断当前是否是real time， real time下要自行计算目标速度和目标加速度， 不用remaining_s_；  非real time下要计算remaining_s_， 直接获取目标速度，位置误差为0，即不用位置误差
- 坡度补偿的坡度角来自于egopose 四元数的计算
```
double pitch = std::asin(2 * (w * y - x * z));
```
- 倒车下坡和上坡时， 坡度slope_compensation_raw是负值， 正值时被过滤为0. fix备注为修复上坡制动不连续？ 应该是上坡mibs考虑了坡度补偿，所以坡度补偿会超调。但是其他工况也都会有超调的情况存在
- 计算当前纵向控制的状态
```
    LongitudeControlState current_status =
        state_transform(module_stopped,velocity_target_acceleration_target_,
        runner_config, rate, extra_json_object);
```                        
- 计算standstill的状态
- 根据纵向控制状态选择对应的控制函数
```
    if (current_status == LongitudeControlState::OFF) {
      result_throttle_brake_value = run_when_off(runner_config);
    } else if (current_status == LongitudeControlState::PID) {
      result_throttle_brake_value =
          run_when_pid(runner_config, &loc_torque, lon_state);
    } else if (current_status == LongitudeControlState::STARTING) {
      result_throttle_brake_value = run_when_starting(runner_config);
    } else if (current_status == LongitudeControlState::STOPPING) {
      result_throttle_brake_value = run_when_stopping(runner_config);
    } else {
      mph_assert(false);
    }
```
- 