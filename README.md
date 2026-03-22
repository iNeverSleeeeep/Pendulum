# WHEELTEC IP570 STM32 Project

STM32 register-level project for the WHEELTEC IP570 linear inverted pendulum.

## Structure

- `USER/`: application logic and controller modules
- `BALANCE/`: balance control, observers, filters, device logic
- `HARDWARE/`: board peripherals and drivers
- `SYSTEM/`: system support code
- `third_party/daqp/`: DAQP quadratic programming solver

## 现状（待补充数据）
### 硬件条件
- 导轨长度约50cm，电压输出死区约+-1.3v（上限+-11.5v）

### 状态观测器
- 板载仿真以1000hz跑在每个循环的开始，对本帧状态进行预测。
- 状态估计采用2个卡尔曼滤波，以1000hz的频率进行预测和更新，使用板载仿真值作为预测值。

### 控制器（LQR）
- LQR控制目前负责全状态反馈，同时稳定位置和角度，根据MATLAB仿真设置的增益，更新频率为200hz。
- 性能：位置跟踪，上升时间 0.35s，超调 7.5%，稳态误差 2.5%。

### 控制器（外环MPC+内环SMC）
- MPC和SMC目前是配合使用，MPC负责位置环输出目标角度给SMC做内环角度控制，MPC为20hz，SMC为200hz。SMC有效控制了抖动，跟踪性能一般（角度跟踪上升时间0.3s）。
- 性能：位置跟踪，上升时间 1.5s，超调 5%，稳态误差 5%。
- 目前存在问题 1. SMC仿真和实际运行的带宽有接近3倍差距。 2. 位置跟踪缓慢，稍微降低R则会产生明显震荡。
- 目前思路 1. 一定要把仿真和实际为什么不同找到。 2. 计算理论性能上限是多少。3.MPC预测与实际差距有多大。

### 控制器（L1）
- 计划是与SMC并联，以测试在磁铁侧面吸附摆杆产生一个额外干扰力矩时的稳定性，因后续经过查资料L1在实际工程中因为较难验证确定性使用并不广泛而暂停推进。
- 目前存在的问题 1. 目前使用的L1是对电压进行补偿，在我的导轨长度比较短+磁铁比较重的情况，无法进行验证。
- 目前思路 1. 改成对目标角度进行补偿，如果还是保持目标角度theta为0，则为了保持力矩平衡小车一定会有横向的加速度，在现实中这个方案无法达到稳态。

## 后续计划

- H-Inf
- SMC+NDOB
- ADRC