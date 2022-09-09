class MsquareTaskImpl : public mtaskflow::FlowTask
有on init(), on_running()函数，其中on_running()函数受周期性调度
```
on running()
{
    meta_.succeed =
        run_once(meta_.trigger_msg_id, control_command,         mpc_trajectory_result);
}
```
run_once()首先调用update()函数更新外围信息，然后调用control_interface_类中的update函数进行横纵向控制算法的调用
```
    control_interface_->Update(plan_, chassis_report_, wheel_report_, ego_pose_, uss_data_,obstacle_from_triangulation_,                               obstacle_from_common_);
```
Update()函数在调用control_loop_类中的Update()函数
```
control_loop_.Update();
```
Update()函数中调用纵向控制算法/横向控制算法/计算执行器指令
```
    LongitudePosControl();
    LongitudeVelControl();
    LateralMpcControl();
    LateralActuatorControl();
    
      ...

    optimized_actuator_ = actuator_optimizer_->update(
    pnc::mathlib::Rad2Deg(control_output.steering_angle_cmd_),
    control_output.accelaration_, 0.0, velocity_mps_, real_steer_angle_rad,
    timestamp_us);

```

