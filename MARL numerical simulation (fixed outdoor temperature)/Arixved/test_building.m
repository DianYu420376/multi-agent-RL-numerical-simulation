%% test whether the Building function is correct
load('Building-4-room.mat')
sigma = 0.5; alpha = 0.5; delta_t = 0.1; N = 4;
Black_Box_Env = @(u, x) Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t);
% get trajectory
TJ = 1000;
x_traj = zeros(4, TJ);
c_traj = zeros(4, TJ);
x0 = ones(N,1)*xo;
x = x0; u = -10*ones(N,1);
for i = 1:TJ
    [c, x] = Black_Box_Env(u, x);
    x_traj(:,i) = x; c_traj(:,i) = c;
end
figure(1);plot(c_traj');title('cost')
figure(2);plot(x_traj');title('temperature')