clear
clc
m = 0.0923;

M = 0.1945;      % 小车质量
K_e = 0.232;     % 反电动势常数
K_m = 0.011;     % 力矩常数
l = 0.1950;      % 摆杆长度
J = 4/3*m*l*l * 0.30;   % 摆杆绕端点的转动惯量
r = 0.018;       % 电机轴半径
g = 9.80;        % 重力加速度
I = 7.083e-06;   % 电机转动惯量
R = 3.75;        % 电枢电阻
Q_eq = m*J+(J+m*l^2)*(M+I/r^2);
A_22 = -(K_m*K_e*(J+m*l^2))/(Q_eq*R*r^2);
A_23 = -(m^2*l^2*g)/Q_eq;
A_42 = (m*l*K_m*K_e)/(Q_eq*R*r^2);
A_43 = (m*g*l*(M+m+I/r^2))/Q_eq;
B_21 = (K_m*(J+m*l^2))/(Q_eq*R*r);
B_41 = -(m*l*K_m)/(Q_eq*R*r);
A = [0 1 0 0; 0 A_22 A_23 0; 0 0 0 1; 0 A_42 A_43 0];
B = [0; B_21; 0; B_41];
C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D = [0; 0; 0; 0];

% 1. 定义物理模型 G
G = ss(A, B, C, D); 
G.InputName = {'u'};
G.OutputName = {'pos', 'pos_dot','theta','theta_dot'};

% 2. 定义权重函数 (给信号加“惩罚”)
W1_pos = tf([1 1],[1 0.5]);   % 位置优先稳定：保留低频约束，但不过分压低频误差
W1_pos.InputName = 'e_pos';  W1_pos.OutputName = 'z_pos';
W1_theta = tf([1 5],[1 1]);   % 角度稳定：主要压制中高频摆动，低频要求适度
W1_theta.InputName = 'e_theta';  W1_theta.OutputName = 'z_theta';
% bode(W1_pos, W1_theta);
% grid on;
% legend('W1_pos','W1_theta');

W2 = tf(1);                 % 放松控制量惩罚，避免为省力矩牺牲稳定性
W2.InputName = 'u';  W2.OutputName = 'z_u';

% 3. 定义加法器 (计算误差)
SumPos = sumblk('e_pos = pos - r_pos');
SumTheta = sumblk('e_theta = theta');

% 5. 组装 P
P = connect(G, W1_pos, W1_theta, W2, SumPos, SumTheta, ...
            {'r_pos', 'u'}, ...
            {'z_pos', 'z_theta', 'z_u', 'e_pos', 'pos_dot', 'e_theta', 'theta_dot'});

% 6. 求解
[K, clp, gamma] = hinfsyn(P, 4, 1);
disp(gamma);

CL = lft(P, K);
isstable(CL)
pole(CL)

% 1. 定义采样时间
Ts = 0.005; 

% 2. 使用 c2d 函数进行离散化
% 推荐使用 'tustin' (双线性变换)，因为它在频率响应上比 'zoh' 更保真
K_discrete = c2d(K, Ts, 'tustin');

% 3. 提取离散状态空间矩阵
[Ak, Bk, Ck, Dk] = ssdata(K_discrete);
fprintf('float Ak[%d][%d] = \n{\n', size(Ak,1), size(Ak,2));
for i = 1:size(Ak,1)
    fprintf('    {');
    fprintf('%f, ', Ak(i,:));
    fprintf('},\n');
end
fprintf('\n};\n');
fprintf('float Bk[%d][%d] = \n{\n', size(Bk,1), size(Bk,2));
for i = 1:size(Bk,1)
    fprintf('    {');
    fprintf('%f, ', Bk(i,:));
    fprintf('},\n');
end
fprintf('\n};\n');
fprintf('float Ck[%d][%d] = \n{\n', size(Ck,1), size(Ck,2));
for i = 1:size(Ck,1)
    fprintf('    {');
    fprintf('%f, ', Ck(i,:));
    fprintf('},\n');
end
fprintf('\n};\n');
fprintf('float Dk[%d][%d] = \n{\n', size(Dk,1), size(Dk,2));
for i = 1:size(Dk,1)
    fprintf('    {');
    fprintf('%f, ', Dk(i,:));
    fprintf('},\n');
end
fprintf('\n};\n');

%% ===================== 3. 仿真参数设置 =====================
t_start = 0;
t_end = 10;    % 仿真时长
dt = 0.005;   % 仿真步长
t = t_start:dt:t_end;

% 期望状态（摆杆竖直、小车静止）
x_d = [0, 0, 0.0, 0]';  

% 初始状态（摆角10°，其余为0）
x = [0, 0, 0.1, 0]';

x_ctrl = zeros(size(Ak,1), 1);
x_ctrl_next = zeros(size(Ak,1), 1);

% 存储数据
x_history = zeros(4, length(t));  % 状态历史
x_d_history = zeros(4, length(t));  % 状态历史
u_history = zeros(1, length(t));  % 控制输入历史

for i = 1:length(t)
    u = Ck * x_ctrl + Dk * x;
    x_ctrl_next = Ak * x_ctrl + Bk * x;
    x_ctrl = x_ctrl_next;

    if abs(u) < 1e-6
        u = 0;
    elseif u > 0 && u < 1.31
        u = 1.31;
    elseif u < 0 && u > -1.31
        u = - 1.31;
    end
    if u > 10
        u = 10;
    end
    if u < -10
        u = -10;
    end
    if u > -1.3 && u < 1.3
        u = 0;
    end
    
    dx = A * x + B * u;
    x = x + dx * dt;

    x_history(:, i) = x;
    x_d_history(:, i) = x_d;
    u_history(i) = u;
end

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