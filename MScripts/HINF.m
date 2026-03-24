clc
m = 0.0923; 
%m = ureal('m', m0, 'Range', [0.0922, 0.0923 * 2]); % 摆杆质量

M = 0.1945;      % 小车质量
K_e = 0.232;     % 反电动势常数
K_m = 0.011;     % 力矩常数
l = 0.1950;      % 摆杆长度
J = 4/3*m*l*l * 0.30;% + (m-m0)*4*l*l;   % 摆杆绕端点的转动惯量
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
C = [1 0 0 0; 0 0 0 1];
D = [0; 0];

% 1. 定义物理模型 G
G = ss(A, B, C, D); 
G.InputName = {'u'};
G.OutputName = {'pos','theta_dot'};

% 2. 定义权重函数 (给信号加“惩罚”)
W1_pos =tf([1/2, 0.5], [1, 0.0001]);   % 位置权重：强化低频，消除位置偏离
W1_pos.InputName = 'e_pos';  W1_pos.OutputName = 'z_pos';
W1_theta_dot = tf([1, 0], [1, 10]) * 0.1 + 0.01;   % 弱化低频，只抑制晃动（限制角速度）
W1_theta.InputName = 'e_theta_dot';  W1_theta.OutputName = 'z_theta_dot';

W2 = tf(0.1);                 % 惩罚控制量 u (防止电机过载)
W2.InputName = 'u';  W2.OutputName = 'z_u';

% 3. 定义加法器 (计算误差)
SumPos = sumblk('e_pos = r_pos - pos');
SumThetaDot = sumblk('e_theta_dot = r_theta_dot - theta_dot');

% 5. 组装 P
% 外部输入 w = [r_pos; r_theta_dot; u]
% 评价信号 z = [z_pos; z_theta_dot; z_u]
% 测量信号 y = [e_pos; e_theta_dot] (反馈给控制器的误差)
P = connect(G, W1_pos, W1_theta, W2, SumPos, SumThetaDot, ...
            {'r_pos', 'r_theta_dot', 'u'}, ...
            {'z_pos', 'z_theta_dot', 'z_u', 'e_pos', 'e_theta_dot'});

% 6. 求解
% 2个测量值 (e_pos, e_theta_dot), 1个控制量 (u)
[K, clp, stats] = hinfsyn(P, 2, 1);

disp(K)
% fprintf('得到的鲁棒稳定裕度: %.2f\n', stats.RobustPerf);

% % 假设 K 是 musyn 算出来的高阶控制器
% K_reduced = reduce(K, 4); % 尝试将其降到 4 阶，同时保留核心动力学特性

% 1. 定义采样时间
Ts = 0.005; 

% 2. 使用 c2d 函数进行离散化
% 推荐使用 'tustin' (双线性变换)，因为它在频率响应上比 'zoh' 更保真
K_discrete = c2d(K, Ts, 'tustin');

% 3. 提取离散状态空间矩阵
[Ak, Bk, Ck, Dk] = ssdata(K_discrete);
fprintf('float Ak[%d][%d] = \n{', size(Ak,1), size(Ak,2));
for i = 1:size(Ak,1)
    fprintf('{');
    fprintf('%f, ', Ak(i,:));
    fprintf('},\n');
end
fprintf('};\n');