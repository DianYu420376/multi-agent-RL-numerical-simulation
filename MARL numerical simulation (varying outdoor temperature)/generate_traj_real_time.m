function [x_traj, c_traj, u_traj] = generate_traj_real_time(K, Black_Box_Env,x0,xo_lst, x_set)
T = length(xo_lst);
N = length(K)/3;
K_mat = blkdiag(K{1:N});
b1 = [K{N+1:N+N}]';
b2 = [K{2*N+1:3*N}]';
x_traj = zeros(4, T);
c_traj = zeros(4, T);
u_traj = zeros(4,T);
x = x0;
for t = 1:T
    xo = xo_lst(t);
    b = b1*xo + b2;
    u = K_mat*x + b;
    [c, x] = Black_Box_Env(x, u, xo, x_set);
    x_traj(:,t) = x; c_traj(:,t) = c;u_traj(:,t) = u;
end
end