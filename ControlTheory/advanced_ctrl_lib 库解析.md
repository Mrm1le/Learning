# ctrl_lib
## 1. 动态系统知识点
### 1.1. 状态空间
- 一般形式
$\dot{x}(t) = f(x,u,t)$
$y = h(x,u,t)$
- 连续线性时变系统
 $\dot{x}(t) = A(t)x(t) + B(t)u(t)$
 $y(t) = C(t)x(t) + D(t)u(t)$

- 连续线性时不变系统
 $\dot{x}(t) = Ax(t) + Bu(t)$
 $y(t) = Cx(t) + Du(t)$

### 1.2. 传递函数
- 一般形式
 $$G(s) = \frac{b_n s^n + b_{n-1}s^{n-1} + \cdots + b_1 s + b_0}{s^n + a_{n-a} s^{n-1} + \cdots + a_1 s + a_0}$$
- 标准形式
 $$G(s) = \frac{c_{n-1}s^{n-1} + \cdots + c_1 s + c_0}{s^n + a_{n-a} s^{n-1} + \cdots + a_1 s + a_0} + d$$
其中，d 可以看成并联子系统，传递函数分析时可以不考虑
### 1.3. 传递函数推导状态方程
- 分子为1形式
 $$G(s) = \frac{1}{s^3 + a_2 s^2 + a_1 s + a_0}$$
$U(s)$为输入，$Y(s)$为输出，可以得到：
$$(s^3 + a_2 s^2 + a_1 s^1 + a_0)Y(s) = U(s)$$
对应微分方程：
$$y'''(t) + a_2 y''(t) + a_1 y'(t) + a_0 y(t) = u(t)$$
定义新的变量 $x_1 = y, x_2 = y', x_3 = y''$：
可以得到：
$$\dot{x}_1 = x_2 \\ \dot{x}_2 = x_3 \\ \dot{x}_x = -a_0 x_1 - a_1 x_2 -a_2 x_3 + u \\ y = x_1$$
可以得到状态空间方程：
$$\begin{bmatrix}
 \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3
 \end{bmatrix} = 
\begin{bmatrix}
 0 & 1 & 0 \\ 0 & 0 & 1 \\ -a_0 & -a_1 & -a_2
 \end{bmatrix}
\begin{bmatrix}
 x_1 \\ x_2 \\ x_3
 \end{bmatrix} + 
\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} u$$
$$y = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix} \begin{bmatrix}
 x_1 \\ x_2 \\ x_3 \end{bmatrix}$$
- 分子不为1形式
 $$G(s) = \frac{b_2 s^2 + b_1 s + b_0}{s^3 + a_2 s^2 + a_1 s + a_0} = \frac{b(s)}{a(s)}$$
可以看成：
$$Y(s) = \frac{b(s)}{a(s)} U(s) = b(s)[\frac{1}{a(s)}U(s)]$$
对于内层：
$$Y_1(s) = \frac{1}{a(s)} U(s)$$
传递函数：
$$G_1(s) = \frac{1}{a(s)} = \frac{1}{(s^3 + a_2 s^2 + a_1 s^1 + a_0)}$$
对于外层：
$$Y_1(s) = b(s)Y_1(s) = b_2 s^2 X_1(s) + b_1 s X_1(s) + b_0 X_1(s)$$
转换成微分方程：
$$y = b_2 \ddot{x_1} + b_1 \dot{x_1} + b_0 x_1$$
定义状态变量：
$$x_1 = x_1 \\ 
 x_2 = \dot{x_1} \\ 
 x_3 = \ddot{x_1} \\
 y = b_2 \ddot{x_1} + b_1 \dot{x_1} + b_0 x_1 = b_2 x_3 + b_1 x_2 + b_0 x_1$$
则可以得到：
$$\begin{bmatrix}
 \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3
 \end{bmatrix} = 
\begin{bmatrix}
 0 & 1 & 0 \\  0 & 0 & 1 \\ -a_0 & -a_1 & -a_2
 \end{bmatrix}
\begin{bmatrix}
 x_1 \\ x_2 \\ x_3
 \end{bmatrix} + 
\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} u$$
$$y = \begin{bmatrix} b_0 & b_1 & b_2 \end{bmatrix}
 \begin{bmatrix} x_1 \\ x_2 \\ x_3 \end{bmatrix}$$
### 1.4. 连续系统离散化
- [前向欧拉](https://zhuanlan.zhihu.com/p/326930724)
$$\dot{x}(k)=Ax(k)+Bu(k)\\
 \frac{x(k+1)-x(k)}{T}=\dot{x}(k)=Ax(k)+Bu(k)$$
可以得到
$$x(k+1)=(AT+I)x(k)+BTu(k)\\
 y(k+1)=C(AT+I)x(k)+CBTu(k)+Du(k+1)\\$$
或者按照泰勒展开的角度
$$z=1+sT$$
即可以看做**算子**为
$$s=\frac{z-1}{T}$$
有频域上的连续方程和离散方程分别为
$$sX(z)=AX(s)+BX(s)$$
$$zX(z)=A_dX(z)+B_dX(z)$$
带入可得
$$\frac{z-1}{T}X(z)=AX(z)+BX(z)$$
$$zX(z)=A(T+1)X(z)+BTX(z)$$
- [后向欧拉, 双线性变换同理](https://blog.csdn.net/weixin_44041199/article/details/119538048?utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~aggregatepage~first_rank_ecpm_v1~rank_v31_ecpm-6-119538048-null-null.pc_agg_new_rank&utm_term=%E7%8A%B6%E6%80%81%E6%96%B9%E7%A8%8B%E7%A6%BB%E6%95%A3%E5%8C%96%E5%A4%84%E7%90%86&spm=1000.2123.3001.4430)

双线性变换本质上是分子分母分别作泰勒展开，去掉高阶项．双变换即两个线性变换．双线性变换是自然对数函数的一阶估算方法，也就是将s平面映射到z平面上
 $$z = e^{sT} = \frac{e^{sT/2}}{e^{-sT/2}} \approx 
 \frac{1+sT/2}{1-sT/2}$$
 可以得到：
 $$s=\frac{2}{T}\frac{z-1}{z+1}$$
 带入到 
 $$sX(z)=AX(s)+BX(s)$$
对于状态方程：
$$\dot{x} = Ax + Bu + c$$
进行双线性变换:
$$G = (I + \frac{1}{2}AT)(I - \frac{1}{2}AT)^{-1} \\
H = (I - \frac{1}{2}AT)^{-1} B T \\
E = C (I + \frac{1}{2}AT)(I - \frac{1}{2}AT)^{-1} \\
F_0 = \frac{1}{2} C (I - \frac{1}{2}AT)^{-1} B T \\
F_1 = D + \frac{1}{2} C (I - \frac{1}{2}AT)^{-1} B T$$
可以得到：
$$x(k+1) = G x(k) + H u(k)$$
$$y(k+1) = E x(k) + F_0 u(k) + F_1 u(k+1)$$
**线性变换与泰勒展开的关系?**
泰勒展开得到z和s之间的关系即算子,带入求解
## 2. 补偿器设计
### 2.1. LeadSys
- 传递函数形式：
 $$G(s) = \frac{T_1s+1}{T_2s+1}$$
 其中，$T_1$为超前时间常数，$T_2$为滞后时间常数，$T_1 > T_2$ 为超前控制器，反之为滞后控制器；
- 代码中$T_2 = T_1 * lead_{gain}$, $lead_{gain}$越小，超前控制越明显：
  $$coef_z[0] = 1.0f / (2.0f * C_PI_F * lead_fc) * kp;\\
  coef_z[1] = 1.0f * kp;\\
  coef_p[0] = 1.0f / (2.0f * C_PI_F * lead_{fc} * lead_{gain});\\
  coef_p[1] = 1.0f;$$
- 补偿器设计关键参数：
  - $lead_{fc}$： 采样周期
  - $lead_{gain}$： 超前系数
  - $k_p$：比例系数
### 2.2. LeadboostSys
- 超前滞后补偿器的二阶形式，本质上是两个一阶超前滞后补偿器串联，解决超前控制器高频噪声大问题；
- 原理类似，代码里面没有使用暂不解析；
### 2.3. IntSubsys
- 积分系统，作为PI控制器的I使用
- 基于一阶系统设计
 $$G(s) = \frac{1}{s}$$
### 2.4. PISys
- PI控制器
 $$G(s) = K_p + \frac{K_i}{s}$$
### 2.5. PILeadSys
- 包含PI控制算法超前控制器
- 具体使用时，PI控制器作用在超前控制器的输出上

```
  u_ = u;
  lead_sys_.Update(u);
  kcomp_term_ = lead_sys.GetOutput();
  pi_sys_.Update(kcomp_term_);
  kp_term_ = pi_sys_.GetkpTerm();
  ki_term_ = pi_sys_.GetkiTerm();
  y_ = pi_sys_.GetOutput();
```

### 2.6. PILeadboostSys
- 超前滞后补偿器二阶形式 + PI控制器
- 代码中暂未使用


## 3 滤波器设计
### 3.1. ButterworthFilter
- 巴特沃斯滤波器
巴特沃斯滤波器是一种在通带和阻带都平坦的滤波器，通带率响应曲线最平坦，没有起伏，阻带频带则逐渐下降为零，下降慢，在过渡带上很容易造成失真。


- 振幅的平方对频率表达式：
$$ |H_a(jw)|^2 = \frac{1}{1+(\frac{w}{w_c})^{2n}} = \frac{1}{1 + \epsilon^2(\frac{w}{w_p})^{2n}}$$
其中，$n$为滤波器阶数，$w_c$为截止频率（振幅下降-3dB频率）， $w_p$为通带边缘频率
- 传递函数形式：
$$H(s) = \frac{w_c^N}{a_0 w_c^N + a_a w_c ^{N-1}s + \cdots + a_{n-1}w_c^1 s^{N-1} + s^N}$$
- 双线性变化离散形式：
$$H(z) = \frac{num0^0 + num1^1 + \cdots + num[n]z^n}
 {den0^0 + den1^1 + \cdots + den[n]z^n}$$
- 滤波器设计关键参数：
  - Nz：滤波器阶数
  - fp：通带边界频率
  - fz：阻带边界频率
  - fs：采样频率

### 3.2. SlopeFilter
- 主要作用是约束信号变化范围及变化速度
- 滤波器设计关键参数：
  - min_update_rate： 变化速度下限
  - max_update_rate： 变化速度上限
  - min_limit： 变化范围下限
  - max_limit： 变化范围上限
### 3.3. Differentiator
- 微分器 + 滤波器
- 关键参数
  - fc：截止频率
  - fs：采样频率
### 3.4. NotchFilter
- 陷波器，滤取指定波段信号
 $$H(s) = \frac{s^2 + w_0^2}{s^2 + 2w\epsilon s + w_0^2}$$
- 陷波器设计关键参数
  - fn: 陷波频率
  - bw: 3dB带宽
  - gain: 系数
  - fs: 采样频率
## 4. 扰动观测器
### 4.1. DOBwithIdealModel
- 扰动观测器的基本思想是将外部干扰及模型参数变化造成的实际模型与理想模型输出的差异统统等效为控制输入，即观测出等效干扰，在控制中引入等量的补偿，实现对干扰完全抑制。
- 横向扰动观测器
  - 输入u: lat_dob_input (两种模式，实际前轮转角或目标前轮转角)
  - 输出y: measures_ptr_->yaw_ 
  - 扰动观测器模型X:
  - 扰动观测器模型Y：[state_hat, dist_hat]
  - 扰动观测器模型U:[y,u,dist]
  - 使用:
```
dist_hat_ -> steering_angle_bias_ -> steering_angle_dist_ -> steering_angle_cmd_
lateral_ctrl_.steering_angle_bias_ = dist_hat_ / model_gain_
lateral_ctrl_.steering_angle_dist_ = 
Clamp <- lateral_ctrl_.steering_angle_bias_
control_output_ptr_->steering_angle_cmd_ =
lateral_ctrl_.steering_angle_cmd_ - lateral_ctrl_.steering_angle_dist_
```
- 纵向扰动观测器
  - 输入u: lon_vel_state.out_ (acceleration_cmd)
  - 输出y: measures_ptr_->vel 
  - 扰动观测器模型X:
  - 扰动观测器模型Y：[state_hat, dist_hat]
  - 扰动观测器模型U:[y,u,dist]
  - 使用：
```
lon_vel_state.dist_ -> lon_vel_state.raw_out_ -> final_out -> lon_vel_state.out_
lon_vel_state.raw_out_ -= lon_vel_state.dist_
control_output_ptr_->accelaration_ = lon_vel_ctrl_.loop_state_.out_
```
### 4.2. KalmanFilter
