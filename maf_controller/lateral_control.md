# parking
横向控制采用osqp求解器，参考量为 $dx, dy, d\phi$

- 通过调用osqp库函数，求解mpc方程
```
  lateral_ctrl_.solver_flag_ =
      lateral_ctrl_.lateral_mpc_solver_.UpdateBySerialRef(lateral_ctrl_.x0_,
                                                          XRef);
```
  - 该调用主要包了两步，分别为
    - 更新梯度，即计算 $Q_{ref}$
    ```
    CastMPCToQPGradientBySerialRef
    ```
    - 迭代求解
      - 更新边界值
      - 更新梯度序列
      - 调用osqp的api求解
        ```
          solver_.updateBounds(lower_bound_, upper_bound_);
          solver_.updateGradient(gradient_);
          bool flag = solver_.solve();
          if (flag) {
            QP_solution_ = solver_.getSolution();
            u_ =
                QP_solution_.block(state_size_ * (mpc_horizon_ + 1), 0, input_size_, 1);
            u_dot_ = (u_ - u0_) * fs_;
            Setu0(u_);
        ```