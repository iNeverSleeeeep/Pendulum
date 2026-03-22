function [x_d,f_v] = MPC_Solve(Np, x)
    A = [0 1 0;0 0 9.8;0 0 -2];
    B = [0 0 2]';
    
    Ts = 0.05;
    sys_c = ss(A,B,eye(3),zeros(3,1));
    sys_d = c2d(sys_c, Ts);
    A = sys_d.A;
    B = sys_d.B;


    Q = diag([130/0.36 10/0.36 10/0.25]);
    R = 5000;

    [K,F,~] = dlqr(A,B,Q,R);
    disp("------------")
    disp(A)
    disp(B)
    disp(F)

    n = size(A, 1);
    p = size(B, 2);
    M = zeros(Np*n, n);
    C = zeros(Np*n, Np*p);
    for i=1:Np
        M((i-1)*n+1:i*n, :) = A^i;
        for j = 1:i
            C((i-1)*n+1:i*n, (j-1)*p+1:j*p) = A^(i-j) * B;
        end
    end

    Q_bar = kron(eye(Np-1),Q);
    Q_bar = blkdiag(Q_bar, F);
    R_bar = kron(eye(Np),R);

    f=2*C'*Q_bar*M*x(1:3);
    H=2*(C'*Q_bar*C + R_bar);
    H = (H + H') / 2; 

    options = optimoptions('quadprog', ...
        'Display', 'off');
    umax = 0.2;
    lb = -umax * ones(Np*p,1);
    ub =  umax * ones(Np*p,1);
    [U_k, FVal] = quadprog(H,f, [], [], [], [], lb, ub, [], options);
    u_k = U_k(1:p,1);
    x_d = [0 0 u_k 0]';
    f_v = FVal;
end

