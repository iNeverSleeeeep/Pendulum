% ---------------------------
% 1. 定义系统参数（根据实际情况修改）
% ---------------------------
% 物理参数
m = 0.0923;      % 摆杆质量
M = 0.1945;      % 小车质量
K_e = 0.232;     % 反电动势常数
K_m = 0.011;     % 力矩常数
l = 0.1950;        % 摆杆长度
J = 4/3*m*l*l * 0.30;   % 摆杆绕端点的转动惯量
r = 0.018;         % 电机轴半径
g = 9.80;          % 重力加速度
I = 7.083e-06;     % 电机转动惯量
R = 3.75;          % 电枢电阻
Q_eq = m*J+(J+m*l^2)*(M+I/r^2);
K = 1;

% ---------------------------
% 2. 构建状态矩阵 A 和输入矩阵 B
% ---------------------------
% 状态矩阵 A (4x4)
A_22 = -(K_m*K_e*(J+m*l^2))/(Q_eq*R*r^2);
A_23 = -(m^2*l^2*g)/Q_eq;
A_42 = (m*l*K_m*K_e)/(Q_eq*R*r^2);
A_43 = (m*g*l*(M+m+I/r^2))/Q_eq;
B_21 = K*(K_m*(J+m*l^2))/(Q_eq*R*r);
B_41 = -K*(m*l*K_m)/(Q_eq*R*r);

A = [0 1 0 0; 0 A_22 A_23 0; 0 0 0 1; 0 A_42 A_43 0];
B = [0; B_21; 0; B_41];
C = [1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1];
D = [0 0 0 0]';

sys_ss = ss(A, B, C, D);

%% ===================== 2. 滑模控制参数设置 =====================
% 滑模面系数矩阵C（维度：1×4，需保证C*B≠0，工程常用[λ^2, 2λ, 0, 0]，聚焦摆角控制）
Sc = [0, 2, 20, 3];  % 滑模面 s = C*e

% 鲁棒增益（抗扰+保证可达性，可调，5~20）
Sk = 10;       

% 边界层厚度（防抖振，可调，0.05~0.2）
delta = 0.2;  

%% ===================== 3. 仿真参数设置 =====================
t_start = 0;
t_end = 10;    % 仿真时长
dt = 0.005;   % 仿真步长
ratio = 10;
t = t_start:dt:t_end;

% 期望状态（摆杆竖直、小车静止）
x_d = [0, 0, 0.0, 0]';  

% 初始状态（摆角10°，其余为0）
x = [1, 0, 0.2, 0]';  

% 存储数据
x_history = zeros(4, length(t));  % 状态历史
x_d_history = zeros(4, length(t));  % 状态历史
u_history = zeros(1, length(t));  % 控制输入历史
u_eq_history = zeros(1, length(t));  % 控制输入历史
u_sw_history = zeros(1, length(t));  % 控制输入历史
e_history = zeros(4, length(t));  % 误差历史
m_history = zeros(4, length(t));  % M
s_history = zeros(1, length(t));  % s
fv_history = zeros(1, length(t));  % s

theta_dot_filtered = 0; % 滤波后的角速度
alpha = 0.05;
fv = 0;

%% ===================== 4. 滑模控制主循环 =====================
for i = 1:length(t)
    %theta_dot_filtered = alpha * x(4) + (1 - alpha) * theta_dot_filtered;
    %x(4) = theta_dot_filtered;
    if mod(i-1, ratio) == 0
        [x_d,fv] = MPC_Solve(10, x); 
    %if (i > 200)
    %    x_d = [0,0,0.1,0]';
    end
    % 步骤1：计算跟踪误差
    e = x - x_d;  
    
    % 步骤2：计算滑模面s
    s = Sc * e;  
    
    % 步骤3：饱和函数（替代符号函数，消抖振）
    if abs(s) <= delta
        sat_s = s / delta;
    else
        sat_s = sign(s);
    end
    
    % 步骤4：计算滑模控制律（核心：基于状态方程推导）
    % 等效控制 u_eq：令 ds/dt = Sc*(Ae + B*u_eq) = 0 → u_eq = -(Sc*B)^(-1)*Sc*A*e
    Mb = inv(Sc * B);
    Ma = Sc * A;
    MMM = -Mb * Ma;
    u_eq = MMM * e; 

    u_sw = (Sk * sat_s * sat_s * sat_s);
    u = u_eq + u_sw;
    if u > 11.5
        u = 11.5;
    end
    if u < -11.5
        u = -11.5;
    end
    
    % 步骤5：状态更新（欧拉积分，基于状态方程 dx/dt = Ax + Bu）
    dx = A * x + B * u;  
    x = x + dx * dt;  
    
    % 存储数据
    x_history(:, i) = x;
    x_d_history(:, i) = x_d;
    e_history(:, i) = e;
    u_history(i) = u;
    u_eq_history(i) = u_eq;
    u_sw_history(i) = u_sw;
    m_history(:, i) = MMM;
    s_history(i) = s;
    fv_history(i) = fv;
end

%% ===================== 5. 结果可视化 =====================
figure('Color','w');
% 子图1：摆角响应
subplot(3,2,1);
plot(t, x_history(1,:), 'b-', 'LineWidth',1.5);
xlabel('时间 (s)'); ylabel('');
title('x0'); grid on;

% 子图2：摆角速度
subplot(3,2,2);
plot(t, x_history(2,:), 'r-', 'LineWidth',1.5);
xlabel('时间 (s)'); ylabel('');
title('x1'); grid on;

% 子图3：u_eq
subplot(3,2,3);
plot(t, x_history(3,:), 'g-', 'LineWidth',1.5);
xlabel('时间 (s)'); ylabel('');
title('x2'); grid on;

% 子图4：控制输入
subplot(3,2,4);
plot(t, x_history(4,:), 'k-', 'LineWidth',1.5);
xlabel('时间 (s)'); ylabel('');
title('x3'); grid on;

% 子图4：控制输入
subplot(3,2,5);
plot(t, x_d_history(3,:), 'k-', 'LineWidth',1.5);
xlabel('时间 (s)'); ylabel('');
title('xd2'); grid on;

% 子图4：控制输入
subplot(3,2,6);
plot(t, u_history(:), 'k-', 'LineWidth',1.5);
xlabel('时间 (s)'); ylabel('');
title('u'); grid on;

sgtitle('倒立摆滑模控制仿真结果（基于状态方程）','FontSize',14);

