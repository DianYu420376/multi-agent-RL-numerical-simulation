function [x_traj, c_traj, u_traj] = generate_traj(K, Black_Box_Env,x0, T)
K_mat = blkdiag(K{1:end-1});
b = K{end};
x_traj = zeros(4, T);
c_traj = zeros(4, T);
u_traj = zeros(4,T);
x = x0;
for t = 1:T
    u = K_mat*x + b;
    [c, x] = Black_Box_Env(x, u);
    x_traj(:,t) = x; c_traj(:,t) = c;u_traj(:,t) = u;
end
end