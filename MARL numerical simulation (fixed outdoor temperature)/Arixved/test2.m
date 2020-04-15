% test the algorithm
addpath('cell_operator')
addpath('History')
clear all
%% specifying hyperparameter:
% Model parameter: A, B, W, sigma, alpha, delta_t
load('Building-4-room.mat')
C = 200;
sigma = 2.5; alpha = 0.01; delta_t = 60; N = 4;
Black_Box_Env = @(x, u) Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t, C);

% initial state and intial controller
x0 = xo*ones(N,1);
K0 = {}; for i = 1:N; K0{i} = zeros(1); end; K0{N+1} = zeros(N,1);%-xo*ones(N,1);

%%
TJ = 300;
TB = 200;
TG = 3000;
r = 0.1;
stepsize = 0.00001;
args.TB = TB; args.r = r;args.TG = TG; args.stepsize = stepsize;
%% specifying sample_estimator and cost_estimator
load('deterministic_run3.mat')
Black_Box_Env = @(x, u) Building(x,xo,x_set,in_door_gain, u, A, B, sigma, alpha, delta_t, C);
stepsize = 0.000003;
r = 0.3;
TJ = 1000;
TB = 20;
TG = 400;
args.TB = TB; args.r = r;args.TG = TG; args.stepsize = stepsize;

sample_generator = @()sample_uniform_sphere(K0, W, TJ);
cost_estimator = @(K) estimate_cost(K, W, x0, Black_Box_Env, TJ);
A_tilde = eye(N) + 1/C*delta_t*A; B_tilde = 1/C*delta_t*B;
Sigma_w = eye(N) * 1/C*1/C*delta_t * sigma^2; Q = eye(N); R = alpha*eye(N);
d = 1/C*delta_t*(xo+in_door_gain);theta = x_set*ones(N,1);
cost_tester = @(K) test_calculate_cost(A_tilde, B_tilde,d,theta, Sigma_w, Q, R, K);

K = K0;
K_history = cell(6,1);
cost_lst = [];
for i = 1:TG
cost = cost_tester(K);
disp(cost)
%if cost < 175
%    break
%end
cost_lst = [cost_lst, cost];
g = estimate_gradient(cost_estimator, sample_generator, K, args);
K = cell_add(K, scalar_cell_mult(-stepsize, g));
%stepsize = stepsize * 0.993;
K_mat = blkdiag(K{1:4});
end
plot(cost_lst)

%% Look at the performance of the K obtained form deterministic gd
K_mat = blkdiag(K{1:end-1});
b = K{end};
mu = zeros(size(x0));J_real = 0;
x_traj = zeros(4, 1000);
c_traj = zeros(4, 1000);
u_traj = zeros(4,1000);
x = x0;
for t = 1:1000
    u = K_mat*x + b;
    [c, x] = Black_Box_Env(x, u);
    x_traj(:,t) = x; c_traj(:,t) = c;u_traj(:,t) = u;
    J_real = J_real + sum(c);
    mu = (t-1)/t*W*mu + 1/t*c;
end
J = (length(K0)-1)*mu/N;
J_real = J_real/1000/N;
figure(2);plot(c_traj');title('cost')
figure(3);plot(x_traj');title('temperature')
figure(4);plot(u_traj');title('controller')