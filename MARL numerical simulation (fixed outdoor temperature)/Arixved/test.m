% test the algorithm
addpath('cell_operator')
clear all
%% specifying hyperparameter:
% Model parameter: A, B, W, sigma, alpha, delta_t
load('Building-4-room.mat')
sigma = 0.5; alpha = 0.5; delta_t = 0.1; N = 4;
Black_Box_Env = @(x, u) Building(x, u, A, B, sigma, alpha, delta_t);

% initial state and intial controller
x0 = zeros(N,1);
K0 = {}; for i = 1:N; K0{i} = -10*ones(1); end

% Algorithmic parameter: TJ, TB,TG, r, stepsize
TJ = 100;
TB = 500;
TG = 200;
r = 0.1;
stepsize = 0.3;
args.TB = TB; args.r = r;args.TG = TG; args.stepsize = stepsize;
%% specifying sample_estimator and cost_estimator
sample_generator = @()sample_uniform_sphere(K0, W, TJ);
cost_estimator = @(K) estimate_cost(K, W, x0, Black_Box_Env, TJ);
A_tilde = eye(N) + delta_t*A; B_tilde = delta_t*B;
Sigma_w = eye(N) * delta_t * sigma^2; Q = eye(N); R = alpha*eye(N);
cost_tester = @(K) test_calculate_cost(A_tilde, B_tilde, Sigma_w, Q, R, K);

K = K0;
cost_lst = [];
for i = 1:TG
cost = cost_tester(blkdiag(K{:}));
disp(cost)
cost_lst = [cost_lst, cost];
g = estimate_gradient(cost_estimator, sample_generator, K, args);
%g = estimate_gradient_coordinate(cost_estimator,sample_generator, K, args);
K = cell_add(K, scalar_cell_mult(-stepsize, g));
K_mat = blkdiag(K{:});
end
plot(cost_lst)
%%
%record_flag = 0;
%[K] = MARL_LQR(cost_estimator, sample_generator, K0,args, record_flag);