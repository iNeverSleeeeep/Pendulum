clc
m = 0.0923;      % 摆杆质量
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
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];
Ts = 0.02; % 采样间隔
t = 0:Ts:6;
[G,H] = c2d(A,B,Ts); % 将连续系统变为离散系统
x0 = [0; 0; 0.1745; 0]; % 初始状态
x_d = [1; 0; 0; 0]; % 目标状态：[位置 速度 角度 角速度]
Tc = ctrb(G,H);

if rank(Tc) == 4
    fprintf('此系统是可控的！\n');
    Q = [12000 0 0 0; 0 0 0 0; 0 0 2000 0; 0 0 0 0]; % Q 矩阵
    R = 1; % R 矩阵
    K = dlqr(G,H,Q,R);

    % 标准参考跟踪控制律：u(k) = -K * (x(k) - x_d)
    x = zeros(length(t),4);
    y = zeros(length(t),2);
    u = zeros(length(t),1);
    x(1,:) = x0';
    y(1,:) = (C * x0)';

    for k = 1:length(t)-1
        e_k = x(k,:)' - x_d;
        u(k) = -K * e_k;
        
        if u(k) > -1.5 && u(k) < 1.5
            u(k) = 0;
        end
        x(k+1,:) = (G * x(k,:)' + H * u(k))';
        y(k+1,:) = (C * x(k+1,:)')';
    end
    u(end) = -K * (x(end,:)' - x_d);

    figure;
    subplot(3,1,1)
    plot(t,y(:,1),'b','LineWidth',1.5);
    hold on
    plot(t,x_d(1) * ones(size(t)),'r--','LineWidth',1.0);
    hold off
    xlabel('Time(s)');
    ylabel('Displacement(m)');
    legend('x','x_{ref}');
    grid on

    subplot(3,1,2)
    plot(t,y(:,2),'b','LineWidth',1.5);
    hold on
    plot(t,x_d(3) * ones(size(t)),'r--','LineWidth',1.0);
    hold off
    xlabel('Time(s)');
    ylabel('Angle(rad)');
    legend('theta','theta_{ref}');
    grid on

    subplot(3,1,3)
    plot(t,u,'k','LineWidth',1.5);
    xlabel('Time(s)');
    ylabel('Control Input');
    grid on

    % LQR 状态反馈对应的开环回路传递函数 L(z) = K * (zI - G)^(-1) * H
    loop_lqr = ss(G,H,K,0,Ts);
    figure;
    margin(loop_lqr);
    grid on
    title('Discrete LQR Loop Margin Plot');

    [Gm_lqr,Pm_lqr,Wcg_lqr,Wcp_lqr] = margin(loop_lqr);
    fprintf('LQR loop: GM = %.4f, PM = %.4f deg, Wcg = %.4f rad/s, Wcp = %.4f rad/s\n', ...
        Gm_lqr, Pm_lqr, Wcg_lqr, Wcp_lqr);
else
    fprintf('此系统不可控！\n');
end
